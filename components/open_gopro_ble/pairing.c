#include "open_gopro_ble_internal.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "camera_manager.h"
#include "ble_core.h"

static const char *TAG = "open_gopro_ble";

void gopro_on_connected_cb(uint16_t conn_handle, const ble_addr_t *addr)
{
    int slot = camera_manager_find_by_addr(addr);
    if (slot >= 0) {
        camera_manager_on_connected(slot, conn_handle);

        gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
        if (ctx) {
            ctx->conn_handle = conn_handle;
        }
    }

    const uint8_t *a = addr->val;
    ESP_LOGI(TAG, "Connected — handle: %d  %02X:%02X:%02X:%02X:%02X:%02X",
             conn_handle, a[5], a[4], a[3], a[2], a[1], a[0]);
}

void gopro_on_encrypted_cb(uint16_t conn_handle, const ble_addr_t *addr)
{
    int slot = camera_manager_find_by_addr(addr);

    if (slot < 0) {
        /* New camera — register it with a placeholder name */
        const uint8_t *a = addr->val;
        char name[CAMERA_NAME_LEN];
        snprintf(name, sizeof(name), "GoPro %02X%02X", a[1], a[0]);

        void *driver_ctx = open_gopro_ble_create_driver_ctx();
        slot = camera_manager_register_new(addr, name,
                                            open_gopro_ble_get_driver(),
                                            driver_ctx,
                                            CAMERA_TYPE_GOPRO_BLE);
        if (slot < 0) {
            ESP_LOGE(TAG, "No free camera slots — cannot register camera");
            return;
        }

        /* Mark as first-time pairing so control_send_pairing_complete() fires
         * later in the GATT flow (after char discovery, before CCCD setup). */
        gopro_ble_ctx_t *new_ctx = (gopro_ble_ctx_t *)driver_ctx;
        if (new_ctx) {
            new_ctx->is_first_pairing = true;
        }

        camera_manager_save_slot(slot);
        ESP_LOGI(TAG, "Registered new camera in slot %d (%s)", slot, name);
    }

    /* Guarantee on_connected has been called regardless of path */
    if (camera_manager_get_handle(slot) == BLE_HS_CONN_HANDLE_NONE) {
        camera_manager_on_connected(slot, conn_handle);
    }

    /* Ensure driver_ctx carries the correct connection handle */
    gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (ctx) {
        ctx->conn_handle = conn_handle;
    }

    /* Discover all GATT services and subscribe to notifications.
     * Must run on every connection — GoPro does not cache CCCD subscriptions. */
    start_gatt_discovery(conn_handle);

    ESP_LOGI(TAG, "Encryption established for slot %d (handle %d)", slot, conn_handle);
}

void gopro_on_disconnected_cb(uint16_t conn_handle, const ble_addr_t *addr, int reason)
{
    /* Clear driver context BEFORE camera_manager clears bt_handle, so we can
     * still look up the slot by handle. */
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot >= 0) {
        gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
        if (ctx) {
            ctx->conn_handle      = BLE_HS_CONN_HANDLE_NONE;
            ctx->recording_status = CAMERA_RECORDING_UNKNOWN;
            memset(&ctx->gatt, 0, sizeof(ctx->gatt));
        }
    }

    camera_manager_on_disconnected(conn_handle);
    free_gatt_disc_ctx(conn_handle);

    /* Release any in-progress query reassembly context for this handle. */
    gopro_query_free(conn_handle);

    ESP_LOGI(TAG, "Disconnected (slot %d, reason %d)", slot, reason);
}
