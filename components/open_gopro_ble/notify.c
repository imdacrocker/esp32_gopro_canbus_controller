/**
 * @file notify.c
 * @brief OpenGoPro ATT notification router.
 *
 * All incoming ATT notifications from connected cameras arrive here via
 * ble_core's on_notify_rx callback.  The router determines which characteristic
 * sent the notification by comparing the attribute handle against the per-camera
 * handle table, then dispatches to the appropriate handler:
 *
 *  GP-0073 (cmd_resp_notify)   → gopro_query_handle_cmd_response()   (query.c)
 *    Handles: SetShutter (0x01), SetDateTime (0x0D), Load Preset (0x40),
 *             GetHardwareInfo (0x3C) responses.
 *
 *  GP-0075 (settings_resp_notify) → logged / ignored (keep-alive ACKs)
 *
 *  GP-0077 (query_resp_notify)  → gopro_presets_handle_*() (presets.c) or
 *                                  gopro_query_handle_query_response() (query.c)
 *    Routing on GP-0077 is based on GPBS frame type and Feature ID:
 *    - Continuation fragment (frame_type ≥ 2): always to presets.c fragment handler.
 *    - First fragment (frame_type = 1, Feature 0xF5): presets.c reassembly start.
 *    - Single packet (frame_type = 0, Feature 0xF5, Action 0xF2): presets.c handler.
 *    - Single packet (frame_type = 0, Feature 0x3C): query.c hw_info response.
 *
 *  GP-0092 (net_mgmt_resp_notify) → logged / not currently used.
 */

#include "open_gopro_ble_internal.h"

#include <string.h>
#include "esp_log.h"
#include "camera_manager.h"

static const char *TAG = "open_gopro_ble";

#define GP_QUERY_GET_STATUS   0x13
#define GP_STATUS_ID_ENCODING 0x08

/* Feature ID for Protobuf Preset responses that arrive on GP-0077.
 * Must match PRESET_FEATURE_ID in presets.c. */
#define GP_PRESET_FEATURE_ID  0xF5
#define GP_PRESET_ACTION_RESP 0xF2

/*
 * Parse a raw Query Response notification (from GP-0077) and route to the
 * correct handler based on GPBS frame type and content.
 *
 * Two response types share GP-0077:
 *
 *   TLV status poll response (query_id = 0x13) — always single-packet:
 *     [GPBS_hdr][0x13][result][status_id][value_len][value...]
 *
 *   Protobuf Preset response (Feature ID = 0xF5) — typically fragmented:
 *     Single-packet:     [GPBS_hdr][0xF5][0xF2][protobuf...]
 *     First fragment:    [GPBS_hdr0][GPBS_hdr1][0xF5][0xF2][protobuf...]
 *     Continuation:      [seq_byte][protobuf...]
 *
 * GPBS frame types (bits [7:5] of byte 0, using 3-bit mask per OpenGoPro):
 *   0b000 (0) — single-packet,   payload at byte 1
 *   0b001 (1) — first fragment,  payload at byte 2
 *   0b010+ (2+) — continuation,  payload at byte 1
 *
 * Recording-status responses (0x13) are always single-packet.  Any
 * fragmented response on GP-0077 must therefore be a Preset response.
 */
static void handle_query_response(uint16_t conn_handle, int slot,
                                   const uint8_t *data, uint16_t len)
{
    if (slot < 0 || data == NULL || len < 2) {
        return;
    }

    uint8_t frame_type = (data[0] >> 5) & 0x07;

    /* ------------------------------------------------------------------
     * Continuation fragment — always a Preset response on GP-0077.
     * ------------------------------------------------------------------ */
    if (frame_type >= 0x02) {
        gopro_presets_handle_query_fragment(conn_handle, data, len);
        return;
    }

    /* ------------------------------------------------------------------
     * Single-packet (0) or first-fragment (1) — identify by Feature/Query ID.
     * For single-packet:  payload at byte 1, so data[1] = Feature/Query ID.
     * For first-fragment: payload at byte 2, so data[2] = Feature/Query ID.
     * ------------------------------------------------------------------ */
    int     payload_off = (frame_type == 0x00) ? 1 : 2;
    uint8_t first_byte  = (len > (uint16_t)payload_off) ? data[payload_off] : 0;

    if (first_byte == GP_PRESET_FEATURE_ID) {
        /* Preset Protobuf response. */
        if (frame_type == 0x00) {
            /* Single-packet: verify Action ID and dispatch directly. */
            if (len >= 4 && data[payload_off + 1] == GP_PRESET_ACTION_RESP) {
                gopro_presets_handle_notify_status(conn_handle, data, len);
            } else {
                ESP_LOGD(TAG, "slot %d: unhandled Preset action=0x%02x",
                         slot, len > (uint16_t)(payload_off + 1)
                               ? data[payload_off + 1] : 0xFF);
            }
        } else {
            /* First fragment: start reassembly. */
            gopro_presets_handle_query_fragment(conn_handle, data, len);
        }
        return;
    }

    /* ------------------------------------------------------------------
     * TLV recording-status poll response — minimum 6 bytes for a valid
     * single-packet with at least one status entry.
     * ------------------------------------------------------------------ */
    if (len < 6) {
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
            camera_recording_status_t old_status = ctx->recording_status;
            camera_recording_status_t new_status =
                data[idx] ? CAMERA_RECORDING_ACTIVE : CAMERA_RECORDING_IDLE;

            if (old_status != new_status) {
                ctx->recording_status = new_status;
                ESP_LOGI(TAG, "slot %d: recording status → %s", slot,
                         new_status == CAMERA_RECORDING_ACTIVE ? "RECORDING" : "IDLE");

                if (new_status == CAMERA_RECORDING_ACTIVE) {
                    /* Camera confirmed it is recording — command delivered.
                     * Clear the pending flag so a future IDLE transition
                     * allows the tick to issue a recovery start command. */
                    ctx->start_cmd_pending = false;
                } else if (old_status == CAMERA_RECORDING_ACTIVE) {
                    /* Camera was recording and has now stopped (while
                     * desired_recording may still be true).  Clear the
                     * pending flag so the camera_manager tick can dispatch
                     * a recovery start command on its next cycle. */
                    ctx->start_cmd_pending = false;
                }
                /* If old_status was UNKNOWN or IDLE and new_status is IDLE,
                 * the camera is still coming up — leave start_cmd_pending
                 * untouched so no duplicate command is sent. */
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

    /* Command responses (GP-0073) must be received even before the camera is
     * fully ready, because GetHardwareInfo responses arrive during the readiness
     * poll and must be routed to query.c to advance the poll state machine. */
    if (attr_handle == ctx->gatt.cmd_resp_notify) {
        gopro_query_handle_cmd_response(conn_handle, data, len);
        return;
    }

    /* All remaining notifications are only valid once the camera is ready. */
    if (!camera_manager_is_camera_ready(slot)) return;

    if (attr_handle == ctx->gatt.query_resp_notify) {
        handle_query_response(conn_handle, slot, data, len);
        return;
    }
    /* Future: handle settings_resp_notify, etc. */
}
