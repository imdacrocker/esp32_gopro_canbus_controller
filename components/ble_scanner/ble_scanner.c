#include "ble_scanner.h"

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_store.h"

static const char *TAG = "BLE_SCANNER";

/* Forward declarations */
static int  scan_event_cb(struct ble_gap_event *event, void *arg);
static int  connection_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void ble_host_task(void *param);

/* True while a connection attempt is in progress — prevents duplicate connects */
static bool s_connecting = false;

/* --------------------------------------------------------------------------
 * NimBLE host task - runs the BLE stack on its own FreeRTOS task
 * -------------------------------------------------------------------------- */
static void ble_host_task(void *param)
{
    nimble_port_run();              /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* --------------------------------------------------------------------------
 * Sync callback - called when the BLE host and controller are ready
 * -------------------------------------------------------------------------- */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack ready, starting scan...");
    start_scan();
}

/* --------------------------------------------------------------------------
 * Reset callback - called if the BLE stack crashes or resets
 * -------------------------------------------------------------------------- */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE stack reset (reason: %d)", reason);
}

/* --------------------------------------------------------------------------
 * Start a passive scan that runs forever
 * -------------------------------------------------------------------------- */
static void start_scan(void)
{
    struct ble_gap_disc_params disc_params = {
        .itvl             = 0,  /* default: 100 ms */
        .window           = 0,  /* default:  50 ms */
        .filter_policy    = 0,  /* accept all advertisers */
        .limited          = 0,  /* general (not limited) discovery */
        .passive          = 1,  /* passive — no scan requests sent */
        .filter_duplicates = 0, /* report every advertisement packet */
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

/* --------------------------------------------------------------------------
 * Connection event callback - called for connect, pairing, and disconnect
 * -------------------------------------------------------------------------- */
static int connection_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            uint16_t handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected — handle: %d", handle);

            /* Check bond store to log whether this is a first pair or reconnect.
             * We try both peer_id_addr (resolved identity) and peer_ota_addr
             * (on-air address) because the store key depends on which was used. */
            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(handle, &desc);

            struct ble_store_key_sec key;
            struct ble_store_value_sec sec;
            memset(&key, 0, sizeof(key));

            key.peer_addr = desc.peer_id_addr;
            bool bonded = (ble_store_read_peer_sec(&key, &sec) == 0);
            if (!bonded) {
                key.peer_addr = desc.peer_ota_addr;
                bonded = (ble_store_read_peer_sec(&key, &sec) == 0);
            }

            if (bonded) {
                ESP_LOGI(TAG, "Known camera — restoring encryption with saved keys");
            } else {
                ESP_LOGI(TAG, "New camera — initiating first-time pairing");
            }

            /* Always call security_initiate.
             * If bonded: NimBLE uses the saved LTK (no camera prompt).
             * If new:    NimBLE does full SMP pairing (camera shows prompt). */
            ble_gap_security_initiate(handle);
        } else {
            ESP_LOGE(TAG, "Connection failed — status: %d", event->connect.status);
            s_connecting = false;
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Paired successfully — handle: %d", event->enc_change.conn_handle);
        } else {
            ESP_LOGE(TAG, "Pairing failed — status: %d", event->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Fired when the peer wants to pair but we already have a bond stored.
         * Delete the stale bond and allow the new pairing to proceed — this
         * handles the case where the GoPro was reset and lost its keys. */
        ESP_LOGW(TAG, "Repeat pairing detected — deleting stale bond and re-pairing");
        struct ble_gap_conn_desc rp_desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &rp_desc);
        ble_store_util_delete_peer(&rp_desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected — reason: %d", event->disconnect.reason);
        s_connecting = false;
        start_scan();
        break;

    default:
        break;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Scan event callback - called for each received advertisement
 * -------------------------------------------------------------------------- */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    /* Parse the advertisement payload */
    struct ble_hs_adv_fields fields;
    int rc = ble_hs_adv_parse_fields(&fields,
                                     event->disc.data,
                                     event->disc.length_data);
    if (rc != 0) {
        return 0;
    }

    /* Filter: only process devices advertising GoPro service UUID 0xFEA6 */
    bool is_gopro = false;
    for (int i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == 0xFEA6) {
            is_gopro = true;
            break;
        }
    }
    if (!is_gopro) {
        return 0;
    }

    /* Prevent duplicate connection attempts if multiple ad packets arrive */
    if (s_connecting) {
        return 0;
    }
    s_connecting = true;

    const uint8_t *a = event->disc.addr.val;
    ESP_LOGI(TAG, "GoPro found — ADDR: %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm",
             a[5], a[4], a[3], a[2], a[1], a[0],
             event->disc.rssi);

    /* Stop scanning and connect */
    ble_gap_disc_cancel();

    int rc2 = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                              BLE_HS_FOREVER, NULL,
                              connection_event_cb, NULL);
    if (rc2 != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc2);
        s_connecting = false;
        start_scan();
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */
void ble_scanner_init(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    /* Security manager config — required for bonding to work correctly.
     * Without these, NimBLE pairs but doesn't properly exchange or store keys. */
    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;               /* "Just Works" — no display or keyboard */
    ble_hs_cfg.sm_bonding        = 1;                                  /* request key storage after pairing    */
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    nimble_port_freertos_init(ble_host_task);
}

void ble_scanner_start(void)
{
    /* Scanning begins automatically once the stack calls on_sync().
     * Nothing to do here yet — this function exists so main.c has
     * a clear place to trigger scanning in the future. */
    ESP_LOGI(TAG, "Waiting for BLE stack sync...");
}
