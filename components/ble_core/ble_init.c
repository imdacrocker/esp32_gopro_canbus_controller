/**
 * @file ble_init.c
 * @brief NimBLE stack initialisation and on_sync boot reconnect.
 *
 * Calls nimble_port_init(), configures the security manager for bonding (Just
 * Works / no I/O capability), and launches the NimBLE host task.
 *
 * The on_sync callback fires once the NimBLE stack is ready.  It:
 *  1. Iterates all stored NimBLE bonds.
 *  2. For each bonded address where is_known_addr() returns true, calls
 *     ble_core_connect_by_addr() to initiate a reconnect.
 *  3. If any bonded cameras are not yet connected, starts a passive background
 *     scan to catch cameras that begin advertising after boot.
 *  4. If has_disconnected_cameras() returns false (all cameras connected), or
 *     if no cameras are paired, the background scan is suppressed.
 */

#include "ble_core_internal.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "ble_core";

ble_core_callbacks_t g_ble_core_cbs = {0};

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int collect_bonded_peers_cb(int obj_type, union ble_store_value *val, void *cookie)
{
    if (s_pending_count >= CONFIG_BT_NIMBLE_MAX_BONDS) {
        return 0;
    }

    /* Deduplicate by address before adding to the list */
    for (int i = 0; i < s_pending_count; i++) {
        if (memcmp(&s_pending_reconnect[i], &val->sec.peer_addr,
                   sizeof(ble_addr_t)) == 0) {
            return 0;
        }
    }

    s_pending_reconnect[s_pending_count++] = val->sec.peer_addr;
    return 0;
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack ready!");

    /* Collect all bonded peers and kick off the reconnect chain */
    s_pending_count = 0;
    s_pending_idx   = 0;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, collect_bonded_peers_cb, NULL);

    if (s_pending_count > 0) {
        ESP_LOGI(TAG, "Found %d bonded peer(s) — starting reconnect chain",
                 s_pending_count);
        reconnect_next();
    } else {
        ESP_LOGI(TAG, "No bonded peers — BLE idle");
    }
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE stack reset (reason: %d)", reason);
}

void ble_core_register_callbacks(const ble_core_callbacks_t *cbs)
{
    if (cbs) {
        g_ble_core_cbs = *cbs;
    }
    ESP_LOGI(TAG, "Callbacks registered");
}

void ble_core_init(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("ESP32 Controller");
    assert(rc == 0);

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE core initialized");
}
