/*
 * readiness.c — OpenGoPro BLE Readiness Polling
 *
 * Implements the OpenGoPro "Wait for Camera BLE Readiness" procedure.
 *
 * After GATT discovery and CCCD subscription are complete the camera's BLE
 * command stack may not be fully initialised yet — particularly when resuming
 * from a suspend state.  The spec requires the client to poll Get Hardware
 * Info (0x3C) repeatedly until the camera returns status 0 (success) before
 * sending any other commands.
 *
 * Flow:
 *   1. gopro_readiness_start() is called by gatt.c once CCCD subscriptions
 *      are complete (replacing the direct set_gatt_ready call).
 *   2. A GetHardwareInfo command is written to GP-0072 immediately.
 *   3. A periodic 500 ms timer retries the command while the camera returns
 *      status 2 (not ready) or no response is received.
 *   4. On status 0 (success) the payload is parsed and logged, and
 *      camera_manager_set_gatt_ready(slot, true) is called.
 *   5. On timeout (30 s / 60 attempts) an error is logged and polling stops.
 *      The BLE connection is left open; the slot never becomes gatt_ready.
 *
 * Thread-safety note:
 *   esp_timer callbacks run in the esp_timer task; BLE notifications arrive
 *   in the NimBLE host task.  No mutex is used here, following the pattern
 *   established elsewhere in this component.  If strict correctness is
 *   required a portMUX_TYPE or FreeRTOS mutex should be added.
 *
 * esp_timer restriction:
 *   esp_timer_delete() must NOT be called from within a timer callback.
 *   The timeout path therefore only calls esp_timer_stop() and clears the
 *   active flag; the handle is fully cleaned up by gopro_readiness_free()
 *   which is called from the BLE disconnect handler (pairing.c) or the
 *   success path, both of which run outside the timer callback.
 */

#include "open_gopro_ble_internal.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include "camera_manager.h"
#include "ble_core.h"

static const char *TAG = "open_gopro_ble";

/* -------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */

#define CMD_GET_HW_INFO              0x3C
#define HW_INFO_STATUS_SUCCESS       0x00
#define HW_INFO_STATUS_NOT_READY     0x02

/** Interval between retries in microseconds (500 ms). */
#define HW_INFO_RETRY_INTERVAL_US    (500ULL * 1000ULL)

/** Maximum number of timer-driven retries before giving up (30 s total). */
#define HW_INFO_MAX_RETRIES          60

/* -------------------------------------------------------------------------
 * Per-connection readiness polling context
 * ------------------------------------------------------------------------- */

typedef struct {
    uint16_t           conn_handle;
    bool               active;          /**< Actively polling (retry timer running). */
    bool               timer_valid;     /**< Timer has been created and not yet deleted. */
    int                retry_count;     /**< Number of timer-driven retries so far. */
    esp_timer_handle_t timer;
    /* GPBS reassembly — used when the response spans multiple ATT notifications. *
     * The camera sends GetHardwareInfoRsp (91 bytes) as a first packet + several  *
     * continuation packets even when MTU is large, because GPBS fragmentation is  *
     * an application-layer concern independent of ATT PDU size.                   */
    bool               rx_assembling;   /**< Currently collecting continuation fragments. */
    uint16_t           rx_total;        /**< Total GPBS payload bytes expected. */
    uint16_t           rx_filled;       /**< Bytes written to rx_buf so far. */
    uint8_t            rx_buf[256];     /**< Reassembly buffer (hw_info rsp is ~91 bytes). */
} readiness_ctx_t;

static readiness_ctx_t s_ctx[CONFIG_BT_NIMBLE_MAX_BONDS];

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

/**
 * Find a context that is either actively polling or mid-reassembly.
 *
 * During GPBS reassembly the retry timer is stopped and active=false, but
 * continuation fragments still need to be routed here.  Searching on
 * (active || rx_assembling) covers both states.
 */
static readiness_ctx_t *find_ctx_for_rx(uint16_t conn_handle)
{
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_ctx[i].conn_handle == conn_handle &&
            (s_ctx[i].active || s_ctx[i].rx_assembling)) {
            return &s_ctx[i];
        }
    }
    return NULL;
}

/** Find a slot by conn_handle regardless of active flag (for cleanup). */
static readiness_ctx_t *find_ctx_any(uint16_t conn_handle)
{
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_ctx[i].conn_handle == conn_handle && s_ctx[i].timer_valid) {
            return &s_ctx[i];
        }
    }
    return NULL;
}

/** Write the GetHardwareInfo command packet to the camera's cmd_write handle. */
static void send_hw_info_request(readiness_ctx_t *ctx)
{
    int slot = camera_manager_find_by_handle(ctx->conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "readiness: slot not found for handle %d", ctx->conn_handle);
        return;
    }

    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!gctx || gctx->gatt.cmd_write == 0) {
        ESP_LOGW(TAG, "readiness: cmd_write not available for slot %d", slot);
        return;
    }

    /* GetHardwareInfo: [length=1][cmd_id=0x3C] — no parameters */
    static const uint8_t k_hw_info_pkt[] = { 0x01, CMD_GET_HW_INFO };

    esp_err_t err = ble_core_gatt_write(ctx->conn_handle, gctx->gatt.cmd_write,
                                        k_hw_info_pkt, sizeof(k_hw_info_pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "readiness slot %d: GetHardwareInfo write failed (%s)",
                 slot, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "readiness slot %d: GetHardwareInfo sent (attempt %d/%d)",
                 slot, ctx->retry_count + 1, HW_INFO_MAX_RETRIES + 1);
    }
}

/** Periodic timer callback — retries GetHardwareInfo every 500 ms. */
static void readiness_retry_cb(void *arg)
{
    readiness_ctx_t *ctx = (readiness_ctx_t *)arg;

    /* Stale callback after stop (race between BLE task and timer task). */
    if (!ctx->active) {
        return;
    }

    ctx->retry_count++;

    if (ctx->retry_count >= HW_INFO_MAX_RETRIES) {
        int slot = camera_manager_find_by_handle(ctx->conn_handle);
        ESP_LOGE(TAG, "readiness slot %d: timed out waiting for camera BLE readiness "
                 "(%d attempts over %llu ms) — camera will not become usable",
                 slot, HW_INFO_MAX_RETRIES + 1,
                 (unsigned long long)(HW_INFO_MAX_RETRIES * HW_INFO_RETRY_INTERVAL_US / 1000ULL));

        /* Cannot call esp_timer_delete() from within a timer callback — only stop.
         * gopro_readiness_free() will delete the handle when called from the
         * disconnect path in pairing.c (which runs outside the timer callback). */
        esp_timer_stop(ctx->timer);
        ctx->active = false;
        return;
    }

    send_hw_info_request(ctx);
}

/* -------------------------------------------------------------------------
 * Hardware info payload parser
 * ------------------------------------------------------------------------- */

/**
 * Consume a length-prefixed field from @p data.
 *
 * @param data       Pointer to the start of the LV field.
 * @param remaining  Bytes left in the buffer from @p data onward.
 * @param out        Destination buffer for the value (null-terminated on return).
 * @param out_size   Size of @p out including the null terminator.
 * @return           Number of bytes consumed (1 + length), or 0 on error.
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
 * Consume a length-prefixed binary field and return its value as a uint32_t.
 * Supports field widths 1–4 bytes, big-endian.
 *
 * @return Number of bytes consumed, or 0 on error.
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
 * Parse the GetHardwareInfoRsp payload and emit one log line per field.
 *
 * @p payload points to the first byte after [length][cmd_id][status].
 */
static void parse_and_log_hw_info(int slot, const uint8_t *payload, uint16_t payload_len)
{
    char     model_name[32]  = {0};
    char     firmware[32]    = {0};
    char     serial[24]      = {0};
    char     ap_ssid[24]     = {0};
    char     ap_mac[24]      = {0};
    uint32_t model_number    = 0;
    uint16_t idx             = 0;
    int      n;

    /* model_number — binary uint */
    n = parse_lv_uint(&payload[idx], payload_len - idx, &model_number);
    if (n == 0) {
        ESP_LOGW(TAG, "readiness slot %d: hw info truncated at model_number "
                 "(payload_len=%d idx=%d)", slot, payload_len, idx);
        goto log_partial;
    }
    idx += (uint16_t)n;

    /* model_name — string */
    n = parse_lv_string(&payload[idx], payload_len - idx,
                        model_name, sizeof(model_name));
    if (n == 0) {
        ESP_LOGW(TAG, "readiness slot %d: hw info truncated at model_name "
                 "(payload_len=%d idx=%d next_byte=0x%02x)",
                 slot, payload_len, idx,
                 (idx < payload_len) ? payload[idx] : 0xFF);
        goto log_partial;
    }
    idx += (uint16_t)n;

    /* deprecated — skip */
    if (idx >= payload_len) {
        goto log_partial;
    }
    uint8_t dep_len = payload[idx++];
    if (idx + dep_len > payload_len) {
        goto log_partial;
    }
    idx += dep_len;

    /* firmware_version — string */
    n = parse_lv_string(&payload[idx], payload_len - idx,
                        firmware, sizeof(firmware));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    /* serial_number — string */
    n = parse_lv_string(&payload[idx], payload_len - idx,
                        serial, sizeof(serial));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    /* ap_ssid — string */
    n = parse_lv_string(&payload[idx], payload_len - idx,
                        ap_ssid, sizeof(ap_ssid));
    if (n == 0) {
        goto log_partial;
    }
    idx += (uint16_t)n;

    /* ap_mac_address — string */
    n = parse_lv_string(&payload[idx], payload_len - idx,
                        ap_mac, sizeof(ap_mac));
    if (n == 0) {
        goto log_partial;
    }
    /* idx += n; — reserved 11 bytes follow, not parsed */

log_partial:
    ESP_LOGI(TAG, "slot %d hardware info: model=%lu (%s)  fw=%s  sn=%s  ssid=%s  mac=%s",
             slot, (unsigned long)model_number, model_name,
             firmware, serial, ap_ssid, ap_mac);
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void gopro_readiness_start(uint16_t conn_handle)
{
    /* Find a free slot or reuse an existing slot for this conn_handle. */
    readiness_ctx_t *ctx = NULL;
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_ctx[i].conn_handle == conn_handle) {
            ctx = &s_ctx[i];
            break;
        }
        if (!ctx && !s_ctx[i].active && !s_ctx[i].timer_valid) {
            ctx = &s_ctx[i];
        }
    }

    if (!ctx) {
        ESP_LOGE(TAG, "readiness: no free context slots");
        return;
    }

    /* Clean up any previous timer on this slot. */
    if (ctx->timer_valid) {
        esp_timer_stop(ctx->timer);
        esp_timer_delete(ctx->timer);
        ctx->timer_valid = false;
        ctx->timer       = NULL;
    }

    ctx->conn_handle   = conn_handle;
    ctx->active        = true;
    ctx->timer_valid   = false;
    ctx->retry_count   = 0;
    ctx->rx_assembling = false;
    ctx->rx_total      = 0;
    ctx->rx_filled     = 0;

    const esp_timer_create_args_t timer_args = {
        .callback = readiness_retry_cb,
        .arg      = ctx,
        .name     = "gopro_ready",
    };

    esp_err_t err = esp_timer_create(&timer_args, &ctx->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readiness: failed to create timer (%s)", esp_err_to_name(err));
        ctx->active = false;
        return;
    }
    ctx->timer_valid = true;

    /* Send the first request immediately, then arm the periodic retry timer. */
    send_hw_info_request(ctx);

    err = esp_timer_start_periodic(ctx->timer, HW_INFO_RETRY_INTERVAL_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readiness: failed to start timer (%s)", esp_err_to_name(err));
        /* Timer was created but not started — clean up fully. */
        esp_timer_delete(ctx->timer);
        ctx->timer       = NULL;
        ctx->timer_valid = false;
        ctx->active      = false;
    }
}

/* -------------------------------------------------------------------------
 * OpenGoPro General Purpose Byte Stream (GPBS) packet framing
 *
 * Every ATT notification payload is prefixed with a GPBS header that encodes
 * the total message length and whether this is the first or a continuation
 * fragment.  Two header sizes are used:
 *
 *   Single / first packet, length 0-31  (frame_type bits [7:5] = 0b000):
 *     Byte 0:  0b000_LLLLL   (L = 5-bit message length)
 *     Payload starts at byte 1.
 *
 *   Single / first packet, length 32-8191  (frame_type bits [7:5] = 0b001):
 *     Byte 0:  0b001_HHHHH   (H = high 5 bits of 13-bit length)
 *     Byte 1:  low 8 bits of 13-bit length
 *     Payload starts at byte 2.
 *
 *   Continuation fragment  (frame_type bits [7:5] = 0b010 or 0b011):
 *     Not parsed here — MTU negotiation avoids multi-fragment responses.
 *
 * Note on query responses (GP-0077): they are always short (< 32 bytes) so
 * the handle_query_response() function in notify.c implicitly handles
 * the single-byte header correctly by treating data[0] as the length and
 * data[1] as the first payload byte — which is exactly what frame_type=000
 * produces.
 * ------------------------------------------------------------------------- */

/**
 * Return the offset of the GPBS message payload within @p data.
 *
 * @return  1 for a 5-bit-length header,
 *          2 for a 13-bit-length extended header,
 *         -1 for a continuation fragment or a packet too short to parse.
 */
static int gpbs_payload_offset(const uint8_t *data, uint16_t len)
{
    if (len < 1) {
        return -1;
    }

    uint8_t frame_type = (data[0] >> 5) & 0x07;

    switch (frame_type) {
    case 0x00:                  /* 5-bit length; payload at byte 1 */
        return 1;
    case 0x01:                  /* 13-bit extended length; payload at byte 2 */
        return (len >= 2) ? 2 : -1;
    default:                    /* 0x02 / 0x03 = continuation; not handled here */
        return -1;
    }
}

void gopro_readiness_handle_response(uint16_t conn_handle,
                                      const uint8_t *data, uint16_t len)
{
    /* Accept packets both while actively polling AND while reassembling. */
    readiness_ctx_t *ctx = find_ctx_for_rx(conn_handle);
    if (!ctx) {
        return;
    }

    if (len < 1) {
        return;
    }

    ESP_LOGI(TAG, "readiness: cmd_resp rx %d bytes:", len);
    ESP_LOG_BUFFER_HEX(TAG, data, len < 32 ? len : 32);

    uint8_t frame_type = (data[0] >> 5) & 0x07;

    /* -----------------------------------------------------------------------
     * Continuation fragment (frame_type 2 or 3):
     *   Byte 0 = GPBS continuation header (sequence number in bits[4:0]).
     *   Bytes 1+ = raw payload to append to the reassembly buffer.
     *
     * The camera sends GetHardwareInfoRsp (91 bytes) as a first packet plus
     * several 20-byte continuation packets regardless of the negotiated ATT
     * MTU — GPBS fragmentation is an application-layer protocol, not tied to
     * ATT PDU size.  With a default 20-byte ATT payload, the 91-byte response
     * arrives as five notifications:
     *   [0]  first (20 B): 2-byte GPBS header + 18 bytes of payload
     *   [1–4] continuations (≤20 B each): 1-byte seq header + 19 bytes each
     * ----------------------------------------------------------------------- */
    if (frame_type >= 0x02) {
        if (!ctx->rx_assembling) {
            ESP_LOGD(TAG, "readiness: stray continuation (not assembling) — ignored");
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

        ESP_LOGD(TAG, "readiness: reassembly %u/%u bytes",
                 (unsigned)ctx->rx_filled, (unsigned)ctx->rx_total);

        if (ctx->rx_filled < ctx->rx_total) {
            return;     /* Still waiting for more fragments. */
        }

        /* All fragments received.
         * rx_buf layout: [cmd_id][status][hw_info fields...] */
        int slot = camera_manager_find_by_handle(conn_handle);
        if (slot >= 0 && ctx->rx_filled >= 2) {
            parse_and_log_hw_info(slot, &ctx->rx_buf[2],
                                  (uint16_t)(ctx->rx_filled - 2));
        }
        ctx->rx_assembling = false;
        gopro_readiness_free(conn_handle);
        return;
    }

    /* -----------------------------------------------------------------------
     * Start packet (frame_type 0 or 1) — parse GPBS header, check response.
     * ----------------------------------------------------------------------- */
    int payload_off = gpbs_payload_offset(data, len);
    if (payload_off < 0) {
        return;
    }
    if ((int)len < payload_off + 2) {
        ESP_LOGW(TAG, "readiness: start packet too short (len=%d payload_off=%d)",
                 len, payload_off);
        return;
    }

    uint8_t cmd_id = data[payload_off];
    uint8_t status = data[payload_off + 1];

    if (cmd_id != CMD_GET_HW_INFO) {
        ESP_LOGD(TAG, "readiness: ignoring cmd_id=0x%02x (want 0x%02x)",
                 cmd_id, CMD_GET_HW_INFO);
        return;
    }

    if (status == HW_INFO_STATUS_NOT_READY) {
        ESP_LOGI(TAG, "readiness: camera not ready (status 0x%02x) — will retry", status);
        return;
    }
    if (status != HW_INFO_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "readiness: unexpected status 0x%02x — will retry", status);
        return;
    }

    /* Status = success — stop the retry timer. */
    if (ctx->active) {
        esp_timer_stop(ctx->timer);
        ctx->active = false;
    }

    int slot = camera_manager_find_by_handle(conn_handle);

    if (frame_type == 0x00) {
        /* ---------------------------------------------------------------
         * Single short packet: entire payload is already in this buffer.
         * --------------------------------------------------------------- */
        int payload_start = payload_off + 2;    /* skip [cmd_id][status] */
        if (slot >= 0 && (int)len > payload_start) {
            parse_and_log_hw_info(slot, &data[payload_start],
                                  (uint16_t)(len - payload_start));
        }
        gopro_readiness_free(conn_handle);
        if (slot >= 0) {
            camera_manager_set_gatt_ready(slot, true);
            ESP_LOGI(TAG, "readiness: slot %d camera BLE-ready", slot);
        }
        return;
    }

    /* -----------------------------------------------------------------------
     * frame_type == 0x01: first fragment of a multi-packet response.
     *
     * The 13-bit GPBS length field tells us the total payload size.  We store
     * the payload bytes (starting from cmd_id) into rx_buf and wait for
     * continuation fragments to fill the rest.
     *
     * gatt_ready is set immediately — we have confirmed status=0.  hw_info is
     * logged once the last continuation fragment has been collected.
     * ----------------------------------------------------------------------- */
    {
        uint16_t total_len = ((uint16_t)(data[0] & 0x1F) << 8) | data[1];

        /* Bytes [payload_off .. len-1] are the first chunk of GPBS payload
         * (cmd_id, status, first hw_info bytes). */
        uint16_t frag_len = (len > (uint16_t)payload_off)
                            ? (uint16_t)(len - payload_off) : 0;
        uint16_t copy_len = (frag_len < (uint16_t)sizeof(ctx->rx_buf))
                            ? frag_len : (uint16_t)sizeof(ctx->rx_buf);

        ctx->rx_total      = total_len;
        ctx->rx_assembling = true;
        ctx->rx_filled     = 0;
        memcpy(ctx->rx_buf, &data[payload_off], copy_len);
        ctx->rx_filled     = copy_len;

        if (slot >= 0) {
            camera_manager_set_gatt_ready(slot, true);
            ESP_LOGI(TAG, "readiness: slot %d camera BLE-ready "
                     "(hw_info reassembly started %u/%u bytes)",
                     slot, (unsigned)ctx->rx_filled, (unsigned)ctx->rx_total);
        }

        /* Unlikely, but handle the case where everything arrived at once. */
        if (ctx->rx_filled >= ctx->rx_total) {
            if (slot >= 0 && ctx->rx_filled >= 2) {
                parse_and_log_hw_info(slot, &ctx->rx_buf[2],
                                      (uint16_t)(ctx->rx_filled - 2));
            }
            ctx->rx_assembling = false;
            gopro_readiness_free(conn_handle);
        }
    }
}

void gopro_readiness_free(uint16_t conn_handle)
{
    /* Search regardless of active flag so we can clean up timed-out slots. */
    readiness_ctx_t *ctx = find_ctx_any(conn_handle);
    if (!ctx) {
        return;
    }

    if (ctx->timer_valid) {
        esp_timer_stop(ctx->timer);
        esp_timer_delete(ctx->timer);
        ctx->timer       = NULL;
        ctx->timer_valid = false;
    }

    ctx->active = false;
}
