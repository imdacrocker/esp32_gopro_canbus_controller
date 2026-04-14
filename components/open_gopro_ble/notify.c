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

    /* Network Management responses (GP-0092) arrive both during and after
     * pairing.  Log the raw bytes unconditionally so we can see the
     * ResponseGeneric result code for RequestPairingFinish.
     *
     * Expected GPBS layout for a short response:
     *   Byte 0: GPBS header (length)
     *   Byte 1: 0x03  Feature ID (Network Management)
     *   Byte 2: 0x81  Action ID (0x01 | 0x80 response bit)
     *   Byte 3: 0x08  Protobuf field 1, wiretype 0 (varint)
     *   Byte 4: 0x00  EnumResultGeneric.RESULT_SUCCESS (0 = ok, 1 = error) */
    if (attr_handle == ctx->gatt.net_mgmt_resp_notify) {
        ESP_LOGI(TAG, "slot %d: net_mgmt_resp rx %d bytes:", slot, len);
        ESP_LOG_BUFFER_HEX(TAG, data, len < 32 ? (int)len : 32);

        /* Minimal ResponseGeneric parse — log the result code if present.
         * GPBS single-packet format: [len][feat_id][act_id][protobuf...]
         * Protobuf {result: N}: tag=0x08, value N (0=SUCCESS, 1+=ERROR) */
        if (len >= 5 && (data[0] & 0xE0) == 0x00) {
            uint8_t feat_id = data[1];
            uint8_t act_id  = data[2];
            uint8_t pb_tag  = data[3];
            uint8_t result  = data[4];
            if (feat_id == 0x03 && act_id == 0x81 && pb_tag == 0x08) {
                if (result == 0x00) {
                    ESP_LOGI(TAG, "slot %d: RequestPairingFinish → SUCCESS "
                             "(pairing screen should dismiss)", slot);
                } else {
                    ESP_LOGW(TAG, "slot %d: RequestPairingFinish → ERROR "
                             "result=0x%02x", slot, result);
                }
            }
        }
        return;
    }

    /* All other notifications are only valid once the camera is fully ready. */
    if (!camera_manager_is_gatt_ready(slot)) return;

    if (attr_handle == ctx->gatt.cmd_resp_notify) {
        /* Route command responses to the query handler (e.g. GetHardwareInfo). */
        gopro_query_handle_cmd_response(conn_handle, data, len);
        return;
    }

    if (attr_handle == ctx->gatt.query_resp_notify) {
        handle_query_response(slot, data, len);
        return;
    }
    /* Future: handle settings_resp_notify, etc. */
}
