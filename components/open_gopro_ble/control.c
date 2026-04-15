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
 *     We start after gatt_ready — i.e. after CCCD subscriptions complete —
 *     because:
 *       1. We have confirmed that settings_write is a valid GATT handle.
 *       2. Using a single global timer that checks gatt_ready is simpler and
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

#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include "ble_core.h"
#include "camera_manager.h"

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

    /* OpenGoPro TLV: [length=3][cmd_id=0x01][param_len=1][param=1] */
    uint8_t pkt[4] = { 0x03, 0x01, 0x01, 0x01 };
    ESP_LOGI(TAG, "conn=%d cmd_write=0x%04x: sending Start Recording",
             gctx->conn_handle, gctx->gatt.cmd_write);
    esp_err_t err = ble_core_gatt_write(gctx->conn_handle, gctx->gatt.cmd_write,
                                        pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "conn=%d: Start Recording write failed (%s)",
                 gctx->conn_handle, esp_err_to_name(err));
    }
    return err;
}

esp_err_t control_stop_recording(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx || gctx->gatt.cmd_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }

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
 * Pairing complete — sent once after initial pairing to dismiss the camera UI
 * ------------------------------------------------------------------------- */

/*
 * RequestPairingFinish packet — written to GP-0091 (net_mgmt_cmd_write).
 *
 * GPBS framing for a short Network Management command:
 *   Byte 0: 0x04  — GPBS single-packet length (4 bytes follow)
 *   Byte 1: 0x03  — Feature ID (Network Management)
 *   Byte 2: 0x01  — Action ID  (Set Pairing State / RequestPairingFinish)
 *
 * Protobuf payload for RequestPairingFinish { pairing_state = COMPLETED(2) }:
 *   Byte 3: 0x08  — field 1, wiretype 0 (varint)
 *   Byte 4: 0x02  — EnumPairingState.PAIRING_STATE_COMPLETED
 *
 * The response arrives on GP-0092 (net_mgmt_resp_notify) as ResponseGeneric.
 * Because this is sent before CCCD subscriptions are in place, the response
 * notification is not received — this is intentional (fire-and-forget).
 */
static const uint8_t k_pairing_complete_pkt[] = { 0x04, 0x03, 0x01, 0x08, 0x02 };

void control_send_pairing_complete(uint16_t conn_handle)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "pairing_complete: no slot for handle %d", conn_handle);
        return;
    }

    gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!ctx || ctx->gatt.net_mgmt_cmd_write == 0) {
        ESP_LOGW(TAG, "pairing_complete: net_mgmt_cmd_write not available for slot %d", slot);
        return;
    }

    esp_err_t err = ble_core_gatt_write(conn_handle, ctx->gatt.net_mgmt_cmd_write,
                                        k_pairing_complete_pkt,
                                        sizeof(k_pairing_complete_pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %d: RequestPairingFinish write failed (%s)",
                 slot, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "slot %d: RequestPairingFinish sent (pairing screen dismissed)", slot);
    }

    ctx->is_first_pairing = false;
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
        if (!camera_manager_is_gatt_ready(i)) continue;

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
        if (!camera_manager_is_gatt_ready(i)) continue;

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
