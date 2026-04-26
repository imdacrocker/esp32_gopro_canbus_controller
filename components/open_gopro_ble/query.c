/*
 * query.c — OpenGoPro on-demand query commands
 *
 * Provides send/receive helpers for OpenGoPro query commands that can be
 * issued on demand (not tied to any polling loop).
 *
 * Currently implements:
 *   GetHardwareInfo (cmd 0x3C) — retrieves camera model, firmware version,
 *   serial number, AP SSID, and AP MAC address.
 *
 * GPBS reassembly:
 *   GetHardwareInfoRsp is ~91 bytes.  The camera fragments this across multiple
 *   ATT notifications at the GPBS application layer regardless of the negotiated
 *   ATT MTU.  A lightweight per-connection reassembly buffer handles this.
 *
 * Usage:
 *   Call gopro_query_send_hw_info(conn_handle) whenever you want hardware info.
 *   The response will arrive asynchronously on cmd_resp_notify; notify.c routes
 *   it to gopro_query_handle_cmd_response(), which logs the parsed fields.
 */

#include "open_gopro_ble_internal.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "camera_manager.h"
#include "ble_core.h"

static const char *TAG = "open_gopro_ble";

/* -------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */

#define CMD_GET_HW_INFO        0x3C
#define HW_INFO_STATUS_SUCCESS 0x00

/* -------------------------------------------------------------------------
 * Per-connection GPBS reassembly context
 *
 * Only populated while a fragmented GetHardwareInfoRsp is being collected.
 * Cleared to zero (in_use = false) when the full response has been received
 * or when the connection drops.
 * ------------------------------------------------------------------------- */

typedef struct {
    bool     in_use;
    uint16_t conn_handle;
    bool     rx_assembling;  /**< Collecting continuation fragments. */
    uint16_t rx_total;       /**< Total GPBS payload bytes expected. */
    uint16_t rx_filled;      /**< Bytes written to rx_buf so far. */
    uint8_t  rx_buf[256];    /**< Reassembly buffer (hw_info rsp is ~91 bytes). */
} query_ctx_t;

static query_ctx_t s_ctx[CONFIG_BT_NIMBLE_MAX_BONDS];

/* -------------------------------------------------------------------------
 * Context helpers
 * ------------------------------------------------------------------------- */

static query_ctx_t *find_ctx(uint16_t conn_handle)
{
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_ctx[i].in_use && s_ctx[i].conn_handle == conn_handle) {
            return &s_ctx[i];
        }
    }
    return NULL;
}

static query_ctx_t *alloc_ctx(uint16_t conn_handle)
{
    /* Reuse existing slot for this handle if present. */
    query_ctx_t *ctx = find_ctx(conn_handle);
    if (ctx) {
        return ctx;
    }
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (!s_ctx[i].in_use) {
            memset(&s_ctx[i], 0, sizeof(s_ctx[i]));
            s_ctx[i].in_use      = true;
            s_ctx[i].conn_handle = conn_handle;
            return &s_ctx[i];
        }
    }
    return NULL;
}

static void free_ctx(uint16_t conn_handle)
{
    query_ctx_t *ctx = find_ctx(conn_handle);
    if (ctx) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

/* -------------------------------------------------------------------------
 * Hardware info payload parser
 *
 * These helpers consume length-value fields from the GetHardwareInfoRsp
 * payload that follows the [cmd_id][status] header bytes.
 * ------------------------------------------------------------------------- */

/**
 * Consume a length-prefixed string field from @p data.
 *
 * @param data       Start of the LV field.
 * @param remaining  Bytes remaining from @p data onward.
 * @param out        Destination buffer (null-terminated on return).
 * @param out_size   Capacity of @p out including the null terminator.
 * @return           Bytes consumed (1 + length field), or 0 on error.
 */
static int parse_lv_string(const uint8_t *data, uint16_t remaining,
                            char *out, size_t out_size)
{
    if (remaining < 1) {
        return 0;
    }
    uint8_t field_len = data[0];
    if ((uint16_t)(1 + field_len) > remaining) {
        return 0;
    }
    size_t copy_len = (field_len < out_size - 1) ? field_len : out_size - 1;
    memcpy(out, &data[1], copy_len);
    out[copy_len] = '\0';
    return 1 + field_len;
}

/**
 * Consume a length-prefixed binary field and return its value as uint32_t.
 * Supports field widths 1–4 bytes (big-endian).
 *
 * @return Bytes consumed, or 0 on error.
 */
static int parse_lv_uint(const uint8_t *data, uint16_t remaining, uint32_t *out)
{
    if (remaining < 1) {
        return 0;
    }
    uint8_t field_len = data[0];
    if ((uint16_t)(1 + field_len) > remaining || field_len > 4) {
        return 0;
    }
    uint32_t val = 0;
    for (int i = 0; i < field_len; i++) {
        val = (val << 8) | data[1 + i];
    }
    *out = val;
    return 1 + field_len;
}

/**
 * Parse the GetHardwareInfoRsp payload and log one line per field.
 *
 * @p payload points to the first byte after [cmd_id][status].
 */
static void parse_and_log_hw_info(int slot, const uint8_t *payload,
                                  uint16_t payload_len)
{
    char     model_name[32] = {0};
    char     firmware[32]   = {0};
    char     serial[24]     = {0};
    char     ap_ssid[24]    = {0};
    char     ap_mac[24]     = {0};
    uint32_t model_number   = 0;
    uint16_t idx            = 0;
    int      n;

    n = parse_lv_uint(&payload[idx], payload_len - idx, &model_number);
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    n = parse_lv_string(&payload[idx], payload_len - idx,
                        model_name, sizeof(model_name));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    /* deprecated field — skip */
    if (idx >= payload_len) {
        goto log_partial;
    }
    uint8_t dep_len = payload[idx++];
    if (idx + dep_len > payload_len) {
        goto log_partial;
    }
    idx += dep_len;

    n = parse_lv_string(&payload[idx], payload_len - idx,
                        firmware, sizeof(firmware));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    n = parse_lv_string(&payload[idx], payload_len - idx,
                        serial, sizeof(serial));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    n = parse_lv_string(&payload[idx], payload_len - idx,
                        ap_ssid, sizeof(ap_ssid));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    n = parse_lv_string(&payload[idx], payload_len - idx,
                        ap_mac, sizeof(ap_mac));
    (void)n;

log_partial:
    ESP_LOGI(TAG, "slot %d hardware info: model=%lu (%s)  fw=%s  sn=%s  ssid=%s  mac=%s",
             slot, (unsigned long)model_number, model_name,
             firmware, serial, ap_ssid, ap_mac);

    /* Persist the model name into the camera slot so it can be displayed
     * on the web page and included in camera_slot_info_t. */
    if (model_name[0] != '\0') {
        camera_manager_set_model_name(slot, model_name);
    }
}

/* -------------------------------------------------------------------------
 * OpenGoPro GPBS packet framing
 *
 * Every ATT notification is prefixed with a GPBS header encoding the total
 * message length and whether this is a first or continuation fragment.
 *
 *   frame_type 0b000: 5-bit length in bits[4:0]; payload starts at byte 1.
 *   frame_type 0b001: 13-bit length — high 5 bits in byte 0 bits[4:0],
 *                     low 8 bits in byte 1; payload starts at byte 2.
 *   frame_type 0b010 / 0b011: continuation fragment; byte 0 is a sequence
 *                     number, payload starts at byte 1.
 * ------------------------------------------------------------------------- */

/**
 * Return the GPBS payload offset within @p data.
 *
 * @return  1 for a 5-bit-length header,
 *          2 for a 13-bit-length extended header,
 *         -1 for a continuation fragment or packet too short to parse.
 */
static int gpbs_payload_offset(const uint8_t *data, uint16_t len)
{
    if (len < 1) {
        return -1;
    }
    uint8_t frame_type = (data[0] >> 5) & 0x07;
    switch (frame_type) {
    case 0x00:
        return 1;
    case 0x01:
        return (len >= 2) ? 2 : -1;
    default:
        return -1;   /* continuation */
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void gopro_query_send_hw_info(uint16_t conn_handle)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "query: slot not found for handle %d", conn_handle);
        return;
    }

    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!gctx || gctx->gatt.cmd_write == 0) {
        ESP_LOGW(TAG, "query: cmd_write not available for slot %d", slot);
        return;
    }

    /* GetHardwareInfo: [length=1][cmd_id=0x3C] — no parameters */
    static const uint8_t k_hw_info_pkt[] = { 0x01, CMD_GET_HW_INFO };

    esp_err_t err = ble_core_gatt_write(conn_handle, gctx->gatt.cmd_write,
                                        k_hw_info_pkt, sizeof(k_hw_info_pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "query slot %d: GetHardwareInfo write failed (%s)",
                 slot, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "query slot %d: GetHardwareInfo sent", slot);
}

void gopro_query_handle_cmd_response(uint16_t conn_handle,
                                     const uint8_t *data, uint16_t len)
{
    if (len < 1) {
        return;
    }

    int slot = camera_manager_find_by_handle(conn_handle);
    uint8_t frame_type = (data[0] >> 5) & 0x07;

    /* -----------------------------------------------------------------------
     * Continuation fragment (frame_type 2 or 3)
     * ----------------------------------------------------------------------- */
    if (frame_type >= 0x02) {
        query_ctx_t *ctx = find_ctx(conn_handle);
        if (!ctx || !ctx->rx_assembling) {
            ESP_LOGD(TAG, "query: stray continuation (not assembling) — ignored");
            return;
        }
        if (len < 2) {
            return;
        }

        uint16_t frag_len = (uint16_t)(len - 1);   /* skip 1-byte seq header */
        uint16_t space    = (uint16_t)sizeof(ctx->rx_buf) - ctx->rx_filled;
        uint16_t copy_len = (frag_len < space) ? frag_len : space;
        memcpy(&ctx->rx_buf[ctx->rx_filled], &data[1], copy_len);
        ctx->rx_filled += copy_len;

        ESP_LOGD(TAG, "query: reassembly %u/%u bytes",
                 (unsigned)ctx->rx_filled, (unsigned)ctx->rx_total);

        if (ctx->rx_filled < ctx->rx_total) {
            return;     /* Still waiting for more fragments. */
        }

        /* All fragments received: rx_buf layout is [cmd_id][status][payload...] */
        if (slot >= 0 && ctx->rx_filled >= 2 && ctx->rx_buf[0] == CMD_GET_HW_INFO) {
            uint8_t hw_status = ctx->rx_buf[1];
            gopro_ble_ctx_t *rctx =
                (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
            if (rctx && rctx->readiness_polling) {
                gopro_readiness_handle_hw_info_status(conn_handle, hw_status);
            }
            if (hw_status == HW_INFO_STATUS_SUCCESS) {
                parse_and_log_hw_info(slot, &ctx->rx_buf[2],
                                      (uint16_t)(ctx->rx_filled - 2));
            }
        }
        free_ctx(conn_handle);
        return;
    }

    /* -----------------------------------------------------------------------
     * Start packet (frame_type 0 or 1) — parse GPBS header.
     * ----------------------------------------------------------------------- */
    int payload_off = gpbs_payload_offset(data, len);
    if (payload_off < 0) {
        return;
    }
    if ((int)len < payload_off + 2) {
        ESP_LOGW(TAG, "query: start packet too short (len=%d payload_off=%d)",
                 len, payload_off);
        return;
    }

    uint8_t cmd_id = data[payload_off];
    uint8_t status = data[payload_off + 1];

    /* -----------------------------------------------------------------------
     * Protobuf command responses — Feature ID in the cmd_id byte position.
     *
     * Unlike TLV responses ([cmd_id][status][payload...]), Protobuf responses
     * are framed as [Feature ID][Action ID][protobuf payload].  Feature IDs
     * are in the 0xF0–0xFF range and do not overlap with TLV command IDs.
     * ----------------------------------------------------------------------- */
    if (cmd_id == 0xF1) {
        /* status holds the Action ID for Protobuf responses, not a TLV result. */
        uint8_t action_id = status;
        if (action_id == 0xE9) {
            /* SetCameraControlStatus ResponseGeneric (Feature 0xF1, Action 0xE9).
             * Protobuf payload: { result: N } encoded as tag=0x08 + varint N.
             * N=0 (RESULT_SUCCESS), non-zero = error.  0xFF = parse failure. */
            uint8_t pb_result = 0xFF;   /* default: malformed / absent */
            int     pb_off    = payload_off + 2;
            if (len >= (uint16_t)(pb_off + 2) && data[pb_off] == 0x08) {
                pb_result = data[pb_off + 1];
            }
            gopro_readiness_handle_camera_control_acked(conn_handle, pb_result);
        } else {
            ESP_LOGD(TAG, "query slot %d: unhandled Protobuf cmd response "
                     "feat=0xF1 action=0x%02x", slot, action_id);
        }
        return;
    }

    /* Handle shutter (SetShutter) command response: cmd_id=0x01 */
    if (cmd_id == 0x01) {
        if (status == 0x00) {
            ESP_LOGI(TAG, "query slot %d: SetShutter command accepted by camera", slot);
        } else {
            ESP_LOGW(TAG, "query slot %d: SetShutter command rejected — status=0x%02x",
                     slot, status);
        }
        return;
    }

    /* Handle SetDateTime command response: cmd_id=0x0D */
    if (cmd_id == 0x0D) {
        if (status == 0x00) {
            ESP_LOGI(TAG, "query slot %d: SetDateTime accepted — camera clock updated",
                     slot);
        } else {
            ESP_LOGW(TAG, "query slot %d: SetDateTime rejected — status=0x%02x",
                     slot, status);
        }
        return;
    }

    /* Handle Load Preset command response: cmd_id=0x40 */
    if (cmd_id == 0x40) {
        if (status == 0x00) {
            ESP_LOGI(TAG, "query slot %d: Load Preset accepted — camera in Video mode",
                     slot);
        } else {
            ESP_LOGW(TAG, "query slot %d: Load Preset rejected — status=0x%02x",
                     slot, status);
        }
        return;
    }

    /* Only handle GetHardwareInfo responses; ignore everything else. */
    if (cmd_id != CMD_GET_HW_INFO) {
        ESP_LOGD(TAG, "query slot %d: unhandled cmd response cmd_id=0x%02x status=0x%02x",
                 slot, cmd_id, status);
        return;
    }

    /* If the readiness poll is active, route the status to readiness.c.
     * On status 0 readiness.c marks the camera ready and completes the
     * connection sequence; on any other status it schedules a retry.
     * Either way we must not start reassembly for a non-zero status, since
     * the camera-not-ready response is always short (never fragmented). */
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (gctx && gctx->readiness_polling) {
        gopro_readiness_handle_hw_info_status(conn_handle, status);
        if (status != HW_INFO_STATUS_SUCCESS) {
            return;  /* readiness.c will retry; do not start reassembly */
        }
        /* status == 0: camera is ready — fall through to parse the payload
         * so the model name is populated in camera_manager. */
    } else if (status != HW_INFO_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "query slot %d: GetHardwareInfo status=0x%02x", slot, status);
        return;
    }

    if (frame_type == 0x00) {
        /* Single short packet — entire payload is in this buffer. */
        int payload_start = payload_off + 2;    /* skip [cmd_id][status] */
        if (slot >= 0 && (int)len > payload_start) {
            parse_and_log_hw_info(slot, &data[payload_start],
                                  (uint16_t)(len - payload_start));
        }
        return;
    }

    /* frame_type == 0x01: first fragment of a multi-packet response.
     * Store the bytes received so far and wait for continuation fragments. */
    query_ctx_t *ctx = alloc_ctx(conn_handle);
    if (!ctx) {
        ESP_LOGE(TAG, "query: no free context slots for reassembly");
        return;
    }

    uint16_t total_len = ((uint16_t)(data[0] & 0x1F) << 8) | data[1];
    uint16_t frag_len  = (len > (uint16_t)payload_off)
                         ? (uint16_t)(len - payload_off) : 0;
    uint16_t copy_len  = (frag_len < (uint16_t)sizeof(ctx->rx_buf))
                         ? frag_len : (uint16_t)sizeof(ctx->rx_buf);

    ctx->rx_total      = total_len;
    ctx->rx_assembling = true;
    ctx->rx_filled     = 0;
    memcpy(ctx->rx_buf, &data[payload_off], copy_len);
    ctx->rx_filled     = copy_len;

    ESP_LOGD(TAG, "query: hw_info reassembly started %u/%u bytes",
             (unsigned)ctx->rx_filled, (unsigned)ctx->rx_total);

    /* Edge case: entire response arrived in the first packet.
     * Readiness routing was already handled above (we only reach here with
     * status 0), so just parse and log. */
    if (ctx->rx_filled >= ctx->rx_total) {
        if (slot >= 0 && ctx->rx_filled >= 2 &&
            ctx->rx_buf[0] == CMD_GET_HW_INFO &&
            ctx->rx_buf[1] == HW_INFO_STATUS_SUCCESS) {
            parse_and_log_hw_info(slot, &ctx->rx_buf[2],
                                  (uint16_t)(ctx->rx_filled - 2));
        }
        free_ctx(conn_handle);
    }
}

void gopro_query_free(uint16_t conn_handle)
{
    free_ctx(conn_handle);
}
