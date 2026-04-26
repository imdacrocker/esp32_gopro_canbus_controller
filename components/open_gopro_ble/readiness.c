/**
 * @file readiness.c
 * @brief OpenGoPro BLE camera readiness poll.
 *
 * After CCCD subscriptions complete (gatt.c), the camera's BLE communication
 * stack may not be fully initialised.  Per the OpenGoPro specification the
 * client must continuously poll GetHardwareInfo until the camera returns
 * status 0 (ready) before sending any further commands.
 *
 * Poll state machine
 * ------------------
 *  1. gopro_start_readiness_poll() — called by gatt.c once GATT handles are
 *     committed.  Resets the retry counter, sets readiness_polling = true,
 *     sends the first GetHardwareInfo, and arms a one-shot timeout timer.
 *
 *  2. If a response arrives before the timer fires:
 *       - Status 0x00 (success): camera is ready.  Timer is cancelled, the
 *         camera_ready flag is set, and gopro_on_camera_ready() runs the
 *         post-ready sequence (SetDateTime, RequestGetPresetStatus).
 *       - Any non-zero status (camera not yet ready): timer is cancelled,
 *         retry counter is incremented.  If under the limit another
 *         GetHardwareInfo is sent and the timer is re-armed; otherwise
 *         ble_gap_terminate() is called and an error is logged.
 *
 *  3. If the timer fires before a response arrives:
 *       Same retry / give-up logic as a non-zero status response.
 *
 * Retry parameters
 * ----------------
 *   READINESS_MAX_RETRIES  10   (up to ~30 s of polling before giving up)
 *   READINESS_TIMEOUT_US   3 000 000  (3 s, matching the keep-alive interval)
 *
 * On disconnect gopro_readiness_cancel() must be called to stop and delete
 * the timer before the driver context is touched, preventing a stale callback.
 */

#include "open_gopro_ble_internal.h"

#include "esp_log.h"
#include "host/ble_gap.h"
#include "camera_manager.h"

static const char *TAG = "open_gopro_ble";

#define READINESS_MAX_RETRIES        10
#define READINESS_TIMEOUT_US         (3000ULL * 1000ULL)   /* 3 s in microseconds */
#define CAMERA_CONTROL_TIMEOUT_US    (3000ULL * 1000ULL)   /* 3 s in microseconds */

#define HW_INFO_STATUS_SUCCESS 0x00

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

/**
 * Retrieve the driver context for a conn_handle.
 * Returns NULL if the slot cannot be found or has no context.
 */
static gopro_ble_ctx_t *ctx_for_handle(uint16_t conn_handle, int *slot_out)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot_out) *slot_out = slot;
    if (slot < 0) return NULL;
    return (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
}

/**
 * Send another GetHardwareInfo and re-arm the timeout timer.
 * The retry counter must already have been incremented before calling this.
 */
static void send_and_rearm(uint16_t conn_handle, gopro_ble_ctx_t *gctx, int slot)
{
    ESP_LOGW(TAG, "slot %d: readiness retry %d/%d — resending GetHardwareInfo",
             slot, gctx->readiness_retry_count, READINESS_MAX_RETRIES);

    gopro_query_send_hw_info(conn_handle);
    esp_timer_start_once(gctx->readiness_timer, READINESS_TIMEOUT_US);
}

/**
 * Final connection sequence — runs after SetCameraControlStatus is
 * acknowledged (or its timeout fires).  Marks the camera ready and sends
 * the remaining setup commands in order.
 *
 *  1. Mark camera_ready so keep-alive and status-poll timers start firing.
 *  2. Set the camera's clock (best-effort; skipped if no GPS lock yet).
 *  3. Switch the camera to Video preset mode.
 */
static void complete_connection_sequence(uint16_t conn_handle, int slot,
                                          gopro_ble_ctx_t *gctx)
{
    ESP_LOGI(TAG, "slot %d: camera ready — completing connection sequence", slot);

    camera_manager_set_camera_ready(slot, true);

    /* Set clock on every connection.  Returns ESP_ERR_INVALID_STATE if UTC
     * is not yet available (no GPS lock); that is acceptable here. */
    control_send_set_date_time(conn_handle);

    /* Phase 1 of Video preset selection (Phase 2 fires from notify.c). */
    gopro_presets_request_video(conn_handle);
}

/**
 * Fired when no SetCameraControlStatus response arrives within
 * CAMERA_CONTROL_TIMEOUT_US.  Logs a warning and advances the connection
 * sequence anyway — a silent camera should not permanently stall setup.
 */
static void camera_control_timeout_cb(void *arg)
{
    gopro_ble_ctx_t *gctx       = (gopro_ble_ctx_t *)arg;
    uint16_t         conn_handle = gctx->conn_handle;

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        return;
    }

    ESP_LOGW(TAG, "slot %d: SetCameraControlStatus timed out — "
             "proceeding with connection sequence anyway", slot);

    gctx->camera_control_pending = false;
    complete_connection_sequence(conn_handle, slot, gctx);
}

/**
 * Called after GetHardwareInfo returns status 0.  Sends
 * SetCameraControlStatus (EXTERNAL) and gates the rest of the connection
 * sequence on the response via camera_control_pending.  If the send fails
 * or the timer cannot be created the sequence continues immediately.
 */
static void gopro_on_camera_ready(uint16_t conn_handle, int slot,
                                   gopro_ble_ctx_t *gctx)
{
    ESP_LOGI(TAG, "slot %d: hardware ready — sending SetCameraControlStatus",
             slot);

    /* Create the one-shot timeout timer if not yet allocated for this slot. */
    if (gctx->camera_control_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = camera_control_timeout_cb,
            .arg      = gctx,
            .name     = "gopro_cam_ctrl",
        };
        esp_err_t err = esp_timer_create(&args, &gctx->camera_control_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "slot %d: failed to create camera_control timer: %s — "
                     "skipping camera control handshake", slot, esp_err_to_name(err));
            complete_connection_sequence(conn_handle, slot, gctx);
            return;
        }
    }

    gctx->camera_control_pending = true;

    esp_err_t err = control_send_camera_control(conn_handle);
    if (err != ESP_OK) {
        /* Write failed — do not wait for a response that will never come. */
        ESP_LOGW(TAG, "slot %d: SetCameraControlStatus send failed — "
                 "skipping handshake", slot);
        gctx->camera_control_pending = false;
        complete_connection_sequence(conn_handle, slot, gctx);
        return;
    }

    esp_timer_start_once(gctx->camera_control_timer, CAMERA_CONTROL_TIMEOUT_US);
}

/* -------------------------------------------------------------------------
 * Timeout timer callback
 * ------------------------------------------------------------------------- */

/**
 * Fired when no GetHardwareInfo response arrives within READINESS_TIMEOUT_US.
 * Treats the silence as equivalent to a "not ready" status and retries.
 *
 * @param arg  Pointer to the gopro_ble_ctx_t for the connection.
 */
static void readiness_timeout_cb(void *arg)
{
    gopro_ble_ctx_t *gctx       = (gopro_ble_ctx_t *)arg;
    uint16_t         conn_handle = gctx->conn_handle;

    /* Bail out if the connection is already gone. */
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        return;
    }

    gctx->readiness_retry_count++;

    if (gctx->readiness_retry_count >= READINESS_MAX_RETRIES) {
        ESP_LOGE(TAG, "slot %d: camera readiness timed out after %d retries "
                 "with no response — disconnecting",
                 slot, gctx->readiness_retry_count);
        gctx->readiness_polling = false;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    send_and_rearm(conn_handle, gctx, slot);
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void gopro_start_readiness_poll(uint16_t conn_handle)
{
    int              slot;
    gopro_ble_ctx_t *gctx = ctx_for_handle(conn_handle, &slot);
    if (!gctx) {
        ESP_LOGW(TAG, "readiness: no ctx for handle %d", conn_handle);
        return;
    }

    /* Reset state for this connection. */
    gctx->readiness_polling     = true;
    gctx->readiness_retry_count = 0;

    /* Create the one-shot timer if it doesn't exist yet.
     * The timer passes gctx as its argument so the callback can find the
     * conn_handle without a global lookup. */
    if (gctx->readiness_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = readiness_timeout_cb,
            .arg      = gctx,
            .name     = "gopro_ready",
        };
        esp_err_t err = esp_timer_create(&args, &gctx->readiness_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "slot %d: failed to create readiness timer: %s",
                     slot, esp_err_to_name(err));
            gctx->readiness_polling = false;
            return;
        }
    }

    ESP_LOGI(TAG, "slot %d: starting readiness poll (max %d retries, %llu ms timeout)",
             slot, READINESS_MAX_RETRIES, READINESS_TIMEOUT_US / 1000ULL);

    gopro_query_send_hw_info(conn_handle);
    esp_timer_start_once(gctx->readiness_timer, READINESS_TIMEOUT_US);
}

void gopro_readiness_handle_hw_info_status(uint16_t conn_handle, uint8_t status)
{
    int              slot;
    gopro_ble_ctx_t *gctx = ctx_for_handle(conn_handle, &slot);
    if (!gctx) {
        return;
    }

    /* Cancel the in-flight timeout — we received a response. */
    if (gctx->readiness_timer != NULL) {
        esp_timer_stop(gctx->readiness_timer);   /* no-op if not running */
    }

    if (status == HW_INFO_STATUS_SUCCESS) {
        gctx->readiness_polling = false;
        gopro_on_camera_ready(conn_handle, slot, gctx);
        return;
    }

    /* Non-zero status — camera not yet ready (status 2) or other error. */
    gctx->readiness_retry_count++;

    if (gctx->readiness_retry_count >= READINESS_MAX_RETRIES) {
        ESP_LOGE(TAG, "slot %d: camera returned not-ready status 0x%02x "
                 "on %d consecutive attempts — disconnecting",
                 slot, status, gctx->readiness_retry_count);
        gctx->readiness_polling = false;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    send_and_rearm(conn_handle, gctx, slot);
}

void gopro_readiness_handle_camera_control_acked(uint16_t conn_handle, uint8_t result)
{
    int              slot;
    gopro_ble_ctx_t *gctx = ctx_for_handle(conn_handle, &slot);
    if (!gctx) {
        return;
    }

    if (!gctx->camera_control_pending) {
        /* Not waiting — stale or duplicate notification; ignore. */
        return;
    }

    /* Cancel the in-flight timeout — we received a response. */
    if (gctx->camera_control_timer != NULL) {
        esp_timer_stop(gctx->camera_control_timer);   /* no-op if not running */
    }

    gctx->camera_control_pending = false;

    if (result == 0x00) {
        ESP_LOGI(TAG, "slot %d: SetCameraControlStatus → SUCCESS "
                 "(camera is under external control)", slot);
    } else if (result == 0xFF) {
        ESP_LOGW(TAG, "slot %d: SetCameraControlStatus response malformed "
                 "(result unparseable) — proceeding anyway", slot);
    } else {
        ESP_LOGW(TAG, "slot %d: SetCameraControlStatus rejected "
                 "(result=0x%02x) — proceeding anyway", slot, result);
    }

    complete_connection_sequence(conn_handle, slot, gctx);
}

void gopro_readiness_cancel(uint16_t conn_handle)
{
    int              slot;
    gopro_ble_ctx_t *gctx = ctx_for_handle(conn_handle, &slot);
    if (!gctx) {
        return;
    }

    if (gctx->readiness_timer != NULL) {
        esp_timer_stop(gctx->readiness_timer);   /* no-op if not running */
        esp_timer_delete(gctx->readiness_timer);
        gctx->readiness_timer = NULL;
    }

    if (gctx->camera_control_timer != NULL) {
        esp_timer_stop(gctx->camera_control_timer);   /* no-op if not running */
        esp_timer_delete(gctx->camera_control_timer);
        gctx->camera_control_timer = NULL;
    }

    gctx->readiness_polling      = false;
    gctx->readiness_retry_count  = 0;
    gctx->camera_control_pending = false;

    if (slot >= 0) {
        ESP_LOGD(TAG, "slot %d: readiness poll cancelled (disconnect)", slot);
    }
}
