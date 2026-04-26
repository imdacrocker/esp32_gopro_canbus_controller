/*
 * control.c — OpenGoPro camera control commands and periodic timers.
 *
 * This file groups all active camera-control operations:
 *
 *   Recording commands
 *     control_start_recording() / control_stop_recording()
 *     Written to GP-0072 (Command characteristic).  These are the
 *     implementations behind the camera_driver_t vtable defined in driver.c.
 *
 *   Status poll timer  (5 s interval)
 *     Queries the encoding_active status flag (ID 8) from every GATT-ready
 *     camera by writing to GP-0076 (Query characteristic).  Responses arrive
 *     as notifications on GP-0077 and are handled by notify.c.
 *
 *   Keep-alive timer  (3 s interval)
 *     Sends the OpenGoPro Keep Alive command (ID 0x5B) to GP-0074 (Settings
 *     characteristic) for every GATT-ready camera.  This resets the camera's
 *     internal keep-alive counter and prevents it from auto-sleeping.
 *
 *     Per the OpenGoPro specification the best practice is to start sending
 *     Keep Alive messages every 3.0 seconds after a connection is established.
 *     We start after camera_ready — i.e. after the readiness poll confirms the
 *     camera is operational — because:
 *       1. We have confirmed that settings_write is a valid GATT handle.
 *       2. Using a single global timer that checks camera_ready is simpler and
 *          mirrors the existing status-poll pattern.
 *
 *     The response from the camera arrives on GP-0075 (Settings Response) and
 *     is intentionally ignored (fire-and-forget).  BLE disconnect handling
 *     already covers connection loss.
 *
 * Keep Alive packet format
 * ========================
 * Written to GP-0074 (settings_write handle).
 * OpenGoPro TLV encoding for a Settings command:
 *   [length][setting_id][param_len][param_value]
 *
 *   Byte 0: 0x03  — GPBS single-packet length (3 bytes follow)
 *   Byte 1: 0x5B  — Keep Alive setting ID
 *   Byte 2: 0x01  — parameter length (1 byte)
 *   Byte 3: 0x42  — hard-coded keep-alive value per OpenGoPro spec
 */

#include "open_gopro_ble_internal.h"

#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include "ble_core.h"
#include "camera_manager.h"
#include "can_manager.h"

static const char *TAG = "open_gopro_ble";

/* -------------------------------------------------------------------------
 * Recording commands — camera_driver_t vtable implementations
 * ------------------------------------------------------------------------- */

esp_err_t control_start_recording(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx || gctx->gatt.cmd_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Guard: if a start command is already in flight, do not send another.
     * The system assumes the camera received the first command; recovery is
     * driven by the status poll, not by retrying here. */
    if (gctx->start_cmd_pending) {
        ESP_LOGD(TAG, "conn=%d: start recording already pending — skipping",
                 gctx->conn_handle);
        return ESP_ERR_INVALID_STATE;
    }

    /* OpenGoPro TLV: [length=3][cmd_id=0x01][param_len=1][param=1] */
    uint8_t pkt[4] = { 0x03, 0x01, 0x01, 0x01 };
    ESP_LOGI(TAG, "conn=%d cmd_write=0x%04x: sending Start Recording",
             gctx->conn_handle, gctx->gatt.cmd_write);
    esp_err_t err = ble_core_gatt_write(gctx->conn_handle, gctx->gatt.cmd_write,
                                        pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "conn=%d: Start Recording write failed (%s)",
                 gctx->conn_handle, esp_err_to_name(err));
    } else {
        gctx->start_cmd_pending = true;
    }
    return err;
}

esp_err_t control_stop_recording(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx || gctx->gatt.cmd_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Clear any in-flight start command — we are explicitly stopping. */
    gctx->start_cmd_pending = false;

    /* OpenGoPro TLV: [length=3][cmd_id=0x01][param_len=1][param=0] */
    uint8_t pkt[4] = { 0x03, 0x01, 0x01, 0x00 };
    ESP_LOGI(TAG, "conn=%d cmd_write=0x%04x: sending Stop Recording",
             gctx->conn_handle, gctx->gatt.cmd_write);
    esp_err_t err = ble_core_gatt_write(gctx->conn_handle, gctx->gatt.cmd_write,
                                        pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "conn=%d: Stop Recording write failed (%s)",
                 gctx->conn_handle, esp_err_to_name(err));
    }
    return err;
}

camera_recording_status_t control_get_recording_status(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx) {
        return CAMERA_RECORDING_UNKNOWN;
    }
    return gctx->recording_status;
}

/* -------------------------------------------------------------------------
 * Set Date/Time — sent once per connection after GATT setup completes
 * ------------------------------------------------------------------------- */

#define CMD_SET_DATE_TIME  0x0D

/*
 * control_send_set_date_time()
 *
 * Builds and transmits an OpenGoPro SetDateTime command (ID 0x0D) to the
 * camera on GP-0072 (Command characteristic).  The current UTC is read from
 * can_manager_get_utc_ms(), which extrapolates forward from the last 0x602
 * CAN frame so the value is current regardless of when this is called.
 *
 * If the RaceCapture has not yet acquired GPS lock (no valid UTC available),
 * the command is skipped with a warning.  The caller is responsible for any
 * retry logic.
 *
 * Response arrives on GP-0073 (cmd_resp_notify) and is confirmed by the
 * 0x0D handler in gopro_query_handle_cmd_response() (query.c).
 *
 * OpenGoPro TLV packet layout:
 *
 *   Byte 0: 0x09  GPBS single-packet length (9 bytes follow)
 *   Byte 1: 0x0D  SetDateTime command ID
 *   Byte 2: 0x07  Parameter length (7 bytes)
 *   Bytes 3–4:    year  (uint16, big-endian)
 *   Byte 5:       month (1–12)
 *   Byte 6:       day   (1–31)
 *   Byte 7:       hour  (0–23)
 *   Byte 8:       minute (0–59)
 *   Byte 9:       second (0–59)
 *
 * Example: 2023-01-31 03:04:05 → 09:0D:07:07:E7:01:1F:03:04:05
 */
esp_err_t control_send_set_date_time(uint16_t conn_handle)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "set_date_time: no camera slot for handle %d", conn_handle);
        return ESP_ERR_INVALID_ARG;
    }

    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!gctx || gctx->gatt.cmd_write == 0) {
        ESP_LOGW(TAG, "slot %d: set_date_time: cmd_write handle not available", slot);
        return ESP_ERR_INVALID_STATE;
    }

    /* Fetch the current best-estimate UTC.  Returns false if GPS lock has not
     * yet been established on the RaceCapture. */
    uint64_t epoch_ms;
    if (!can_manager_get_utc_ms(&epoch_ms)) {
        ESP_LOGW(TAG, "slot %d: set_date_time: UTC not yet available — "
                 "skipping (no GPS lock on RaceCapture)", slot);
        return ESP_ERR_INVALID_STATE;
    }

    /* Apply the stored timezone offset so cameras receive local time rather
     * than raw UTC.  Cast through int64_t to handle negative offsets safely. */
    int64_t tz_ms = (int64_t)can_manager_get_tz_offset_hours() * 3600LL * 1000LL;
    epoch_ms = (uint64_t)((int64_t)epoch_ms + tz_ms);

    /* Break epoch into calendar fields. */
    time_t    t = (time_t)(epoch_ms / 1000);
    struct tm ti;
    gmtime_r(&t, &ti);

    uint16_t year   = (uint16_t)(ti.tm_year + 1900);
    uint8_t  month  = (uint8_t)(ti.tm_mon + 1);   /* tm_mon is 0-based */
    uint8_t  day    = (uint8_t)(ti.tm_mday);
    uint8_t  hour   = (uint8_t)(ti.tm_hour);
    uint8_t  minute = (uint8_t)(ti.tm_min);
    uint8_t  second = (uint8_t)(ti.tm_sec);

    uint8_t pkt[10] = {
        0x09,                           /* GPBS length */
        CMD_SET_DATE_TIME,              /* cmd_id */
        0x07,                           /* param length */
        (uint8_t)(year >> 8),           /* year high byte */
        (uint8_t)(year & 0xFF),         /* year low byte  */
        month, day, hour, minute, second
    };

    ESP_LOGI(TAG, "slot %d: SetDateTime → %04d-%02d-%02d %02d:%02d:%02d local (UTC%+d)",
             slot, year, month, day, hour, minute, second,
             (int)can_manager_get_tz_offset_hours());

    esp_err_t err = ble_core_gatt_write(conn_handle, gctx->gatt.cmd_write,
                                        pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %d: SetDateTime write failed (%s)",
                 slot, esp_err_to_name(err));
    }
    return err;
}

/* -------------------------------------------------------------------------
 * Set Camera Control Status — sent once after GetHardwareInfo succeeds
 * ------------------------------------------------------------------------- */

/*
 * RequestSetCameraControlStatus packet — written to GP-0072 (Command characteristic).
 *
 * Per the OpenGoPro specification, sending this command with
 * CAMERA_CONTROL_STATUS_EXTERNAL (2) tells the camera that an external
 * controller (us) is claiming control.  The camera immediately exits any
 * contextual menu and returns to the idle screen.  This is required before
 * the camera will reliably accept recording commands from a BLE client.
 *
 * Note: any physical button press on the camera will cause it to reclaim
 * control.  If persistent external control is needed it must be re-asserted,
 * but for record-trigger use this one-time handshake at connect time is
 * sufficient.
 *
 * GPBS Protobuf framing for a short Command:
 *   Byte 0: 0x04  GPBS single-packet length (4 bytes follow)
 *   Byte 1: 0xF1  Feature ID  (Camera Control)
 *   Byte 2: 0x69  Action ID   (RequestSetCameraControlStatus)
 *
 * Protobuf payload for
 *   RequestSetCameraControlStatus { camera_control_status = EXTERNAL (2) }:
 *   Byte 3: 0x08  protobuf field 1, wiretype 0 (varint)
 *   Byte 4: 0x02  EnumCameraControlStatus.CAMERA_CONTROL_STATUS_EXTERNAL
 *
 * Response arrives on GP-0073 (cmd_resp_notify) as ResponseGeneric
 * (Feature ID 0xF1, Action ID 0xE9) and is dispatched by query.c to
 * gopro_readiness_handle_camera_control_acked() in readiness.c.
 */
static const uint8_t k_set_camera_control_pkt[] = { 0x04, 0xF1, 0x69, 0x08, 0x02 };

esp_err_t control_send_camera_control(uint16_t conn_handle)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "camera_control: no slot for handle %d", conn_handle);
        return ESP_ERR_INVALID_ARG;
    }

    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!gctx || gctx->gatt.cmd_write == 0) {
        ESP_LOGW(TAG, "slot %d: camera_control: cmd_write not available", slot);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "slot %d: Sending SetCameraControlStatus → EXTERNAL (claiming BLE control)",
             slot);
    esp_err_t err = ble_core_gatt_write(conn_handle, gctx->gatt.cmd_write,
                                        k_set_camera_control_pkt,
                                        sizeof(k_set_camera_control_pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %d: SetCameraControlStatus write failed (%s)",
                 slot, esp_err_to_name(err));
    }
    return err;
}

void open_gopro_ble_sync_time_all(void)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (!camera_manager_is_camera_ready(i)) {
            continue;
        }

        uint16_t conn_h = camera_manager_get_handle(i);
        if (conn_h == BLE_HS_CONN_HANDLE_NONE) {
            continue;
        }

        control_send_set_date_time(conn_h);
    }
}

/* -------------------------------------------------------------------------
 * Status poll timer — queries recording state of every connected camera
 * ------------------------------------------------------------------------- */

static esp_timer_handle_t s_status_poll_timer = NULL;

/* Query packet written to GP-0076 to request encoding_active (status ID 8).
 * OpenGoPro TLV: [length=2][query_id=0x13][status_id=0x08] */
static const uint8_t k_status_query_pkt[] = { 0x02, 0x13, 0x08 };

static void status_poll_timer_cb(void *arg)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (!camera_manager_is_camera_ready(i)) continue;

        uint16_t conn_h = camera_manager_get_handle(i);
        if (conn_h == BLE_HS_CONN_HANDLE_NONE) continue;

        gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(i);
        if (!ctx || ctx->gatt.query_write == 0) continue;

        esp_err_t err = ble_core_gatt_write(conn_h, ctx->gatt.query_write,
                                            k_status_query_pkt,
                                            sizeof(k_status_query_pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "slot %d: status query write failed (%s)",
                     i, esp_err_to_name(err));
        }
    }
}

/* -------------------------------------------------------------------------
 * Keep-alive timer — resets the camera's keep-alive counter every 3 s
 * ------------------------------------------------------------------------- */

static esp_timer_handle_t s_keep_alive_timer = NULL;

/*
 * Keep Alive packet — written to GP-0074 (settings_write handle).
 *
 * OpenGoPro TLV (Settings format):
 *   Byte 0: 0x03  GPBS single-packet length (3 payload bytes follow)
 *   Byte 1: 0x5B  Keep Alive setting ID
 *   Byte 2: 0x01  parameter length
 *   Byte 3: 0x42  hard-coded keep-alive value (per OpenGoPro spec)
 */
static const uint8_t k_keep_alive_pkt[] = { 0x03, 0x5B, 0x01, 0x42 };

static void keep_alive_timer_cb(void *arg)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (!camera_manager_is_camera_ready(i)) continue;

        uint16_t conn_h = camera_manager_get_handle(i);
        if (conn_h == BLE_HS_CONN_HANDLE_NONE) continue;

        gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(i);
        if (!ctx || ctx->gatt.settings_write == 0) continue;

        esp_err_t err = ble_core_gatt_write(conn_h, ctx->gatt.settings_write,
                                            k_keep_alive_pkt,
                                            sizeof(k_keep_alive_pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "slot %d: keep-alive write failed (%s)",
                     i, esp_err_to_name(err));
        } else {
            ESP_LOGD(TAG, "slot %d: keep-alive sent", i);
        }
    }
}

/* -------------------------------------------------------------------------
 * Public — called once from open_gopro_ble_init() in driver.c
 * ------------------------------------------------------------------------- */

void open_gopro_control_start_timers(void)
{
    esp_err_t err;

    /* Status poll timer */
    if (s_status_poll_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = status_poll_timer_cb,
            .arg      = NULL,
            .name     = "gopro_status_poll",
        };
        err = esp_timer_create(&args, &s_status_poll_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create status poll timer: %s",
                     esp_err_to_name(err));
        } else {
            err = esp_timer_start_periodic(s_status_poll_timer,
                                           (uint64_t)STATUS_POLL_INTERVAL_MS * 1000ULL);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start status poll timer: %s",
                         esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Status poll timer started — interval %d ms",
                         STATUS_POLL_INTERVAL_MS);
            }
        }
    }

    /* Keep-alive timer */
    if (s_keep_alive_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = keep_alive_timer_cb,
            .arg      = NULL,
            .name     = "gopro_keepalive",
        };
        err = esp_timer_create(&args, &s_keep_alive_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create keep-alive timer: %s",
                     esp_err_to_name(err));
        } else {
            err = esp_timer_start_periodic(s_keep_alive_timer,
                                           (uint64_t)KEEP_ALIVE_INTERVAL_MS * 1000ULL);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start keep-alive timer: %s",
                         esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Keep-alive timer started — interval %d ms",
                         KEEP_ALIVE_INTERVAL_MS);
            }
        }
    }
}
