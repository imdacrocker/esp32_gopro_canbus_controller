#include "open_gopro_ble_internal.h"

#include <string.h>
#include "esp_log.h"
#include "camera_manager.h"

static const char *TAG = "open_gopro_ble";

#define GP_QUERY_GET_STATUS   0x13
#define GP_STATUS_ID_ENCODING 0x08

/* Parse a raw Query Response notification (from GP-0077) and update the
 * camera's cached recording status.
 *
 * OpenGoPro TLV layout:
 *   [length][query_id=0x13][result=0x00][status_id][value_len][value...]
 */
static void handle_query_response(int slot, const uint8_t *data, uint16_t len)
{
    /* Minimum valid packet: 6 bytes */
    if (slot < 0 || data == NULL || len < 6) {
        return;
    }

    uint8_t query_id = data[1];
    uint8_t result   = data[2];

    if (query_id != GP_QUERY_GET_STATUS) {
        return; /* Not a response we handle yet */
    }

    if (result != 0x00) {
        ESP_LOGW(TAG, "slot %d: status query returned error 0x%02x", slot, result);
        return;
    }

    gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!ctx) {
        return;
    }

    /* Walk the packed TLV status entries that follow the 3-byte header */
    uint16_t idx = 3;
    while (idx + 2 <= len) {
        uint8_t status_id = data[idx++];
        uint8_t value_len = data[idx++];

        if (idx + value_len > len) {
            break; /* Truncated packet */
        }

        if (status_id == GP_STATUS_ID_ENCODING && value_len >= 1) {
            camera_recording_status_t new_status =
                data[idx] ? CAMERA_RECORDING_ACTIVE : CAMERA_RECORDING_IDLE;

            if (ctx->recording_status != new_status) {
                ctx->recording_status = new_status;
                ESP_LOGI(TAG, "slot %d: recording status → %s", slot,
                         new_status == CAMERA_RECORDING_ACTIVE ? "RECORDING" : "IDLE");
            }
        }

        idx += value_len;
    }
}

void gopro_on_notify_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                            const uint8_t *data, uint16_t len)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) return;

    gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!ctx) return;

    /* Command responses are needed during the BLE readiness poll, which runs
     * BEFORE gatt_ready is set.  Route cmd_resp_notify first, before the
     * gatt_ready guard below. */
    if (attr_handle == ctx->gatt.cmd_resp_notify) {
        gopro_readiness_handle_response(conn_handle, data, len);
        return;
    }

    /* All other notifications are only valid once the camera is fully ready. */
    if (!camera_manager_is_gatt_ready(slot)) return;

    if (attr_handle == ctx->gatt.query_resp_notify) {
        handle_query_response(slot, data, len);
    }
    /* Future: handle settings_resp_notify, net_mgmt_resp_notify, etc. */
}
