/**
 * @file can_manager.c
 * @brief CAN bus (TWAI) driver — RaceCapture ↔ ESP32 protocol implementation.
 *
 * Frame handling
 * --------------
 *  0x600 (RaceCapture → ESP32): isLogging byte in payload[0].
 *    - Compared against the previous value; fires on_logging_state_changed
 *      only on transitions.  State times out to LOGGING_STATE_UNKNOWN after
 *      CAN_MANAGER_LOGGING_TIMEOUT_MS with no new frame.
 *
 *  0x601 (ESP32 → RaceCapture): camera state broadcast at 5 Hz.
 *    - Bytes 0–3 hold CAMERA_STATE_* values for slots 0–3.
 *    - Sent by a periodic esp_timer callback regardless of bus activity.
 *
 *  0x602 (RaceCapture → ESP32): GPS UTC timestamp (uint64_t ms epoch, LE).
 *    - First valid frame (year > 2020) fires the can_utc_acquired_cb_t
 *      callback exactly once and logs the human-readable timestamp at INFO.
 *    - Subsequent frames update the stored epoch + monotonic reference pair
 *      used by can_manager_get_utc_ms() for extrapolation.
 *
 * Threading model
 * ---------------
 * A single FreeRTOS task (s_rx_task) dequeues received frames from s_rx_queue
 * and processes them.  The TWAI ISR enqueues raw frames.  State accessible to
 * callers (camera states, logging state, UTC) is protected by s_state_mutex.
 */

#include "can_manager.h"

#include <string.h>
#include <inttypes.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "hal/twai_types.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "can_manager";

/* ============================================================
 * Private state
 * ============================================================ */

static twai_node_handle_t   s_node      = NULL;
static QueueHandle_t        s_rx_queue  = NULL;
static TaskHandle_t         s_rx_task   = NULL;

/* Raw frame callback (optional, used for debugging/sniffing). */
static can_rx_frame_cb_t    s_rx_cb     = NULL;
static void                *s_rx_cb_ctx = NULL;

/* Logging-state callback, fired only when the state transitions. */
static can_logging_state_cb_t s_logging_cb      = NULL;
static void                  *s_logging_cb_ctx  = NULL;
static volatile logging_state_t s_logging_state  = LOGGING_STATE_UNKNOWN; /* current state */
static int64_t                s_last_cmd_rx_us  = 0;   /* esp_timer_get_time() of last 0x600 */

/* UTC-acquired callback, fired exactly once on first valid 0x602 frame. */
static can_utc_acquired_cb_t  s_utc_acquired_cb     = NULL;
static void                  *s_utc_acquired_cb_ctx = NULL;

/* Camera states broadcast in 0x601.  Written from any task, read from the
 * processing task.  Single-byte writes are atomic on ESP32-S3 Xtensa LX7,
 * so volatile is sufficient here — no mutex needed. */
static volatile uint8_t s_camera_states[CAN_MANAGER_MAX_CAMERAS];  /* all 0 on start */

/* TX frame and data buffer for the 0x601 broadcast.
 *
 * IMPORTANT: These MUST be static (module lifetime), not stack-allocated.
 * The v6.0 TWAI driver queues a *pointer* to twai_frame_t, not a copy.
 * The ISR dereferences that pointer later when the hardware is ready to
 * transmit.  A stack-allocated frame becomes invalid the moment the
 * function that created it returns, causing a LoadProhibited crash when
 * the ISR fires.  Static storage guarantees the pointer is always valid. */
static uint8_t      s_tx_buf[8];
static twai_frame_t s_tx_frame;

/* Flags set from ISR, consumed from the processing task. */
static volatile bool     s_bus_off     = false;
static volatile uint32_t s_error_count = 0;

/* UTC state — updated from every valid 0x602 frame.
 * Protected by s_utc_mutex so can_manager_get_utc_ms() is safe to call
 * from any task (e.g. the BLE/camera task on camera connect). */
static SemaphoreHandle_t s_utc_mutex   = NULL;
static volatile bool     s_utc_valid   = false;  /* true once first good frame received */
static uint64_t          s_utc_ms      = 0;       /* epoch ms from last 0x602 frame      */
static int64_t           s_utc_rx_tick = 0;       /* esp_timer_get_time() at last RX (µs)*/

/* ============================================================
 * Timezone offset — stored in NVS, applied when setting camera clocks
 *
 * Kept here (rather than wifi_manager) so that open_gopro_ble and
 * legacy_gopro can read the value without creating a circular
 * component dependency on wifi_manager.
 *
 * NVS namespace: "settings"   key: "tz_offset"   type: int8_t
 * Default: -8 (UTC-8 / US Pacific Standard Time).
 * Reset: nvs_flash_erase() (factory reset) restores the default.
 * ============================================================ */

#define TZ_NVS_NS   "settings"
#define TZ_NVS_KEY  "tz_offset"

static int8_t s_tz_offset_hours = -8;

static void tz_load_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(TZ_NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "TZ settings namespace absent — using default (%d h)", (int)s_tz_offset_hours);
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "tz_load_from_nvs: nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    int8_t offset;
    err = nvs_get_i8(h, TZ_NVS_KEY, &offset);
    if (err == ESP_OK) { s_tz_offset_hours = offset; }
    nvs_close(h);
    ESP_LOGI(TAG, "TZ offset loaded from NVS: %d h", (int)s_tz_offset_hours);
}

static void tz_save_to_nvs(int8_t offset)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(TZ_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tz_save_to_nvs: nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_i8(h, TZ_NVS_KEY, offset);
    if (err == ESP_OK) { err = nvs_commit(h); }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tz_save_to_nvs: write failed: %s", esp_err_to_name(err));
    }
}

/* ============================================================
 * ISR Callbacks
 * Running in ISR context — no blocking calls, no ESP_LOG.
 * ============================================================ */

/**
 * Called by the TWAI driver ISR each time a frame is ready to read.
 * We copy it into a fully self-contained struct and push it to the
 * task queue via xQueueSendFromISR.
 */
static bool on_rx_done(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *edata,
                       void *user_ctx)
{
    uint8_t raw_buf[TWAI_FRAME_MAX_LEN] = {0};
    twai_frame_t rx_frame = {
        .buffer     = raw_buf,
        .buffer_len = sizeof(raw_buf),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
        return false;
    }

    uint16_t raw_len = twaifd_dlc2len(rx_frame.header.dlc);
    uint8_t  data_len = (raw_len > TWAI_FRAME_MAX_LEN)
                        ? (uint8_t)TWAI_FRAME_MAX_LEN
                        : (uint8_t)raw_len;

    can_frame_t frame = {
        .id          = rx_frame.header.id,
        .dlc         = (uint8_t)rx_frame.header.dlc,
        .data_len    = data_len,
        .is_extended = (bool)rx_frame.header.ide,
        .is_rtr      = (bool)rx_frame.header.rtr,
    };
    memcpy(frame.data, raw_buf, data_len);

    BaseType_t higher_prio_task_woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &frame, &higher_prio_task_woken);
    return (higher_prio_task_woken == pdTRUE);
}

/** Called on CAN error state transitions.  We flag bus-off for the task. */
static bool on_state_change(twai_node_handle_t handle,
                            const twai_state_change_event_data_t *edata,
                            void *user_ctx)
{
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        s_bus_off = true;
    }
    return false;
}

/** Called on bus errors.  We tally for periodic task-side reporting. */
static bool on_error(twai_node_handle_t handle,
                     const twai_error_event_data_t *edata,
                     void *user_ctx)
{
    s_error_count++;
    return false;
}

/* ============================================================
 * Internal — Protocol Handlers (task context)
 * ============================================================ */

/** Human-readable label for a logging_state_t value (for log messages). */
static const char *logging_state_name(logging_state_t state)
{
    switch (state) {
        case LOGGING_STATE_LOGGING:     return "LOGGING";
        case LOGGING_STATE_NOT_LOGGING: return "NOT_LOGGING";
        default:                        return "UNKNOWN";
    }
}

/**
 * Process a received 0x600 frame (RaceCapture → ESP32 commands).
 * Byte 0: isLogging (0 or 1).
 *
 * Always stamps s_last_cmd_rx_us regardless of whether the state changes,
 * so the 5 s timeout resets on every valid frame.  The callback fires only
 * when the state actually changes.
 */
static void handle_rc_command(const can_frame_t *frame)
{
    if (frame->data_len < 1) {
        ESP_LOGW(TAG, "0x600 frame too short (len=%u), ignoring", frame->data_len);
        return;
    }

    /* Reset the timeout clock on every valid frame. */
    s_last_cmd_rx_us = esp_timer_get_time();

    logging_state_t new_state = (frame->data[0] != 0)
                                ? LOGGING_STATE_LOGGING
                                : LOGGING_STATE_NOT_LOGGING;

    if (new_state == s_logging_state) {
        return;     /* no change — nothing to do */
    }

    ESP_LOGI(TAG, "0x600 RX: byte0=0x%02X  logging state: %s → %s",
             frame->data[0],
             logging_state_name(s_logging_state),
             logging_state_name(new_state));

    s_logging_state = new_state;

    if (s_logging_cb) {
        s_logging_cb(s_logging_state, s_logging_cb_ctx);
    }
}

/**
 * Process a received 0x602 frame (RaceCapture → ESP32 UTC timestamp).
 *
 * Payload: 64-bit millisecond Unix epoch, little-endian (LSB in byte 0).
 * The RaceCapture Lua script suppresses transmission before GPS lock, but
 * we validate anyway: any timestamp before Jan 1 2020 is treated as pre-lock
 * noise and silently dropped.
 *
 * Logs exactly once — on the first valid frame — then goes silent.
 */
static void handle_rc_utc(const can_frame_t *frame)
{
    if (frame->data_len < 8) {
        ESP_LOGW(TAG, "0x602 UTC frame too short (len=%u), ignoring", frame->data_len);
        return;
    }

    /* Unpack 64-bit little-endian millisecond epoch. */
    uint64_t epoch_ms = 0;
    for (int i = 0; i < 8; i++) {
        epoch_ms |= (uint64_t)frame->data[i] << (i * 8);
    }

    /* Sanity check: Jan 1, 2020 00:00:00 UTC = 1577836800000 ms.
     * Anything older is almost certainly a pre-lock 1970 epoch value. */
    if (epoch_ms < 1577836800000ULL) {
        return;
    }

    int64_t rx_tick    = esp_timer_get_time();
    bool    first_valid = false;

    xSemaphoreTake(s_utc_mutex, portMAX_DELAY);
    if (!s_utc_valid) {
        s_utc_valid = true;
        first_valid = true;
    }
    s_utc_ms      = epoch_ms;
    s_utc_rx_tick = rx_tick;
    xSemaphoreGive(s_utc_mutex);

    if (first_valid) {
        /* Convert to a human-readable UTC string for the one-time log. */
        time_t t = (time_t)(epoch_ms / 1000);
        struct tm ti;
        gmtime_r(&t, &ti);
        ESP_LOGI(TAG, "UTC acquired — %04d-%02d-%02d %02d:%02d:%02d.%03" PRIu64 " UTC  "
                 "(%" PRIu64 " ms epoch)",
                 ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                 ti.tm_hour, ti.tm_min, ti.tm_sec,
                 epoch_ms % 1000, epoch_ms);

        /* Notify any registered listener (e.g. to set date/time on cameras
         * that are already connected at the moment UTC first becomes valid). */
        if (s_utc_acquired_cb) {
            s_utc_acquired_cb(s_utc_acquired_cb_ctx);
        }
    }
}

/**
 * Build and transmit the 0x601 camera status frame.
 * Called at 5 Hz from the processing task.
 *
 * Frame layout:
 *   Byte 0: Camera 0 state (camera_state_t)
 *   Byte 1: Camera 1 state
 *   Byte 2: Camera 2 state
 *   Byte 3: Camera 3 state
 *   Bytes 4–7: reserved (0x00)
 */
static void broadcast_camera_status(void)
{
    /* Update the persistent buffer with current camera states.
     * s_tx_frame.buffer already points to s_tx_buf (set in can_manager_init). */
    s_tx_buf[0] = s_camera_states[0];
    s_tx_buf[1] = s_camera_states[1];
    s_tx_buf[2] = s_camera_states[2];
    s_tx_buf[3] = s_camera_states[3];
    /* bytes 4-7 stay 0x00 (set once during init, never written) */

    esp_err_t ret = twai_node_transmit(s_node, &s_tx_frame, 0 /* non-blocking */);
    if (ret == ESP_ERR_TIMEOUT) {
        /* TX queue full (bus is down).  Skip this cycle — the queue already
         * holds our frame pointer and will drain when the bus recovers. */
        ESP_LOGD(TAG, "0x601 TX skipped (queue full)");
    } else if (ret != ESP_OK) {
        ESP_LOGW(TAG, "0x601 TX failed: %s", esp_err_to_name(ret));
    }
}

/* ============================================================
 * Processing Task
 * ============================================================ */

static void can_rx_task(void *arg)
{
    can_frame_t frame;
    // uint32_t    last_logged_errors = 0;
    // int         diag_tick = 0;  /* counts 100 ms ticks for 5 s diagnostics */
    int         tx_tick   = 0;  /* counts 100 ms ticks for 5 Hz TX */

    const int    tx_ticks      = (int)(CAN_MANAGER_TX_INTERVAL_MS / 100U);   /* = 2  */
    const int64_t logging_timeout_us =
        (int64_t)CAN_MANAGER_LOGGING_TIMEOUT_MS * 1000LL;                    /* = 5 s in µs */

    /* Stamp the clock now so the timeout is measured from task-start, not from
     * the epoch origin (which would cause an instant UNKNOWN→UNKNOWN no-op on
     * first check — harmless, but misleading). */
    s_last_cmd_rx_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Processing task started");

    while (1) {
        /* Block up to 100 ms waiting for an RX frame.  The short timeout lets
         * the loop also handle TX, bus-off recovery, and diagnostics reliably
         * even during periods of silence on the bus. */
        if (xQueueReceive(s_rx_queue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {

            /* ---- Dispatch known protocol messages --------------------- */
            if (frame.id == CAN_ID_RC_COMMAND && !frame.is_extended) {
                handle_rc_command(&frame);
            } else if (frame.id == CAN_ID_RC_UTC && !frame.is_extended) {
                handle_rc_utc(&frame);
            } //else {
            //     /* DBG: log any frame we don't recognise.  Helps confirm
            //      * the bus is active and shows what else RaceCapture sends.
            //      * Remove once communication is confirmed working. */
            //     ESP_LOGI(TAG, "CAN RX unknown: ID=0x%03"PRIX32"%s len=%u "
            //              "[%02X %02X %02X %02X %02X %02X %02X %02X]",
            //              frame.id,
            //              frame.is_extended ? "(ext)" : "",
            //              frame.data_len,
            //              frame.data[0], frame.data[1],
            //              frame.data[2], frame.data[3],
            //              frame.data[4], frame.data[5],
            //              frame.data[6], frame.data[7]);
            // }

            /* ---- Forward all frames to raw callback (debug/sniff) ----- */
            if (s_rx_cb) {
                s_rx_cb(&frame, s_rx_cb_ctx);
            }
        }

        /* ---- Periodic TX: broadcast camera status at 5 Hz ------------ */
        if (++tx_tick >= tx_ticks) {
            tx_tick = 0;
            broadcast_camera_status();
        }

        /* ---- Bus-off recovery ---------------------------------------- */
        if (s_bus_off) {
            s_bus_off = false;
            ESP_LOGW(TAG, "Bus-off detected — initiating recovery");
            esp_err_t ret = twai_node_recover(s_node);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Recovery failed: %s", esp_err_to_name(ret));
            }
        }

        /* ---- Logging timeout → UNKNOWN ------------------------------- */
        /* Only meaningful when we're in a known state; if we're already
         * UNKNOWN there's nothing to transition into. */
        if (s_logging_state != LOGGING_STATE_UNKNOWN) {
            int64_t elapsed = esp_timer_get_time() - s_last_cmd_rx_us;
            if (elapsed > logging_timeout_us) {
                ESP_LOGW(TAG, "No 0x600 for %" PRId64 " ms — logging state: %s → UNKNOWN",
                         elapsed / 1000LL, logging_state_name(s_logging_state));
                s_logging_state = LOGGING_STATE_UNKNOWN;
                if (s_logging_cb) {
                    s_logging_cb(LOGGING_STATE_UNKNOWN, s_logging_cb_ctx);
                }
            }
        }

        /* ---- Periodic diagnostics (every 5 s) ------------------------ */
        // if (++diag_tick >= diag_ticks) {
        //     diag_tick = 0;
        //     uint32_t errs = s_error_count;
        //     if (errs != last_logged_errors) {
        //         ESP_LOGW(TAG, "CAN bus errors since boot: %" PRIu32, errs);
        //         last_logged_errors = errs;
        //     }
        // }
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

esp_err_t can_manager_register_rx_callback(can_rx_frame_cb_t cb, void *user_ctx)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_rx_cb     = cb;
    s_rx_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t can_manager_register_logging_callback(can_logging_state_cb_t cb, void *user_ctx)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_logging_cb     = cb;
    s_logging_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t can_manager_register_utc_acquired_callback(can_utc_acquired_cb_t cb, void *user_ctx)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_utc_acquired_cb     = cb;
    s_utc_acquired_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t can_manager_set_camera_state(uint8_t camera_idx, camera_state_t state)
{
    if (camera_idx >= CAN_MANAGER_MAX_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((uint8_t)state > (uint8_t)CAMERA_STATE_RECORDING) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Single-byte write — atomic on Xtensa LX7 without a mutex. */
    s_camera_states[camera_idx] = (uint8_t)state;
    return ESP_OK;
}

logging_state_t can_manager_get_logging_state(void)
{
    /* 32-bit aligned read — single instruction on Xtensa LX7, no mutex needed. */
    return s_logging_state;
}

bool can_manager_get_utc_ms(uint64_t *epoch_ms_out)
{
    if (epoch_ms_out == NULL) {
        return false;
    }

    xSemaphoreTake(s_utc_mutex, portMAX_DELAY);
    bool     valid    = s_utc_valid;
    uint64_t base_ms  = s_utc_ms;
    int64_t  rx_tick  = s_utc_rx_tick;
    xSemaphoreGive(s_utc_mutex);

    if (!valid) {
        return false;
    }

    /* Extrapolate from the last received timestamp using the monotonic timer.
     * esp_timer_get_time() returns microseconds — divide by 1000 for ms. */
    int64_t elapsed_ms = (esp_timer_get_time() - rx_tick) / 1000;
    *epoch_ms_out = base_ms + (uint64_t)elapsed_ms;
    return true;
}

void can_manager_set_tz_offset(int8_t hours)
{
    /* Clamp to the valid IANA UTC offset range. */
    if (hours < -12) { hours = -12; }
    if (hours >  14) { hours =  14; }
    s_tz_offset_hours = hours;
    tz_save_to_nvs(hours);
    ESP_LOGI(TAG, "TZ offset set to %d h", (int)hours);
}

int8_t can_manager_get_tz_offset_hours(void)
{
    return s_tz_offset_hours;
}

esp_err_t can_manager_init(void)
{
    esp_err_t ret;

    /* Load persisted timezone offset before any camera-clock operations. */
    tz_load_from_nvs();

    /* Camera states default to 0 (UNDEFINED) via static initialisation.
     * Explicitly set here for clarity in case of a future re-init path. */
    memset((void *)s_camera_states, CAMERA_STATE_UNDEFINED,
           sizeof(s_camera_states));

    /* Initialise the persistent TX frame once.  The driver stores a pointer
     * to this struct, so it must never move — static storage guarantees that. */
    memset(s_tx_buf,   0x00, sizeof(s_tx_buf));
    memset(&s_tx_frame, 0x00, sizeof(s_tx_frame));
    s_tx_frame.header.id  = CAN_ID_CAM_STATUS;
    s_tx_frame.header.dlc = 8;
    /* ide/rtr/fdf all remain 0: standard 11-bit data frame, classic CAN. */
    s_tx_frame.buffer     = s_tx_buf;
    s_tx_frame.buffer_len = sizeof(s_tx_buf);

    /* --- Create the UTC mutex ------------------------------------------ */
    s_utc_mutex = xSemaphoreCreateMutex();
    if (s_utc_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UTC mutex");
        return ESP_ERR_NO_MEM;
    }

    /* --- Create the software receive queue ----------------------------- */
    s_rx_queue = xQueueCreate(CAN_MANAGER_RX_QUEUE_DEPTH, sizeof(can_frame_t));
    if (s_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        ret = ESP_ERR_NO_MEM;
        goto cleanup_mutex;
    }

    /* --- Configure and create the on-chip TWAI node -------------------- */
    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx               = CAN_MANAGER_TX_GPIO,
            .rx               = CAN_MANAGER_RX_GPIO,
            .quanta_clk_out   = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CAN_MANAGER_BITRATE_BPS,
            /* sp_permill = 0: driver selects an optimal sample point. */
        },
        .tx_queue_depth = CAN_MANAGER_TX_QUEUE_DEPTH,
        /* fail_retry_cnt: how many times the hardware retries a frame on TX
         * error before giving up and calling on_tx_done(is_tx_success=false).
         *
         * -1 (retry forever) caused an error storm when the bus was
         * disconnected: the hardware retried the first queued frame
         * indefinitely, fired on_error ~7000 times/second, and prevented
         * the queue from draining.  5 retries is enough for normal CAN
         * arbitration and transient errors while avoiding infinite loops. */
        .fail_retry_cnt = 5,
        /* flags all 0: normal mode — participates on bus, sends ACKs, can TX. */
    };

    ret = twai_new_node_onchip(&node_cfg, &s_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(ret));
        goto cleanup_queue;
    }

    /* --- Register ISR event callbacks ---------------------------------- */
    twai_event_callbacks_t cbs = {
        .on_rx_done      = on_rx_done,
        .on_tx_done      = NULL,
        .on_state_change = on_state_change,
        .on_error        = on_error,
    };

    ret = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
        goto cleanup_node;
    }

    /* --- Start the processing task ------------------------------------- */
    BaseType_t task_ret = xTaskCreate(can_rx_task,
                                       "can_rx",
                                       4096,
                                       NULL,
                                       CAN_MANAGER_TASK_PRIORITY,
                                       &s_rx_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processing task");
        ret = ESP_ERR_NO_MEM;
        goto cleanup_node;
    }

    /* --- Enable the node (begin active participation on the bus) ------- */
    ret = twai_node_enable(s_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TWAI node: %s", esp_err_to_name(ret));
        goto cleanup_task;
    }

    ESP_LOGI(TAG, "CAN manager ready — TX GPIO %d, RX GPIO %d, %" PRIu32 " bps",
             CAN_MANAGER_TX_GPIO, CAN_MANAGER_RX_GPIO,
             (uint32_t)CAN_MANAGER_BITRATE_BPS);
    return ESP_OK;

cleanup_task:
    vTaskDelete(s_rx_task);
    s_rx_task = NULL;
cleanup_node:
    twai_node_delete(s_node);
    s_node = NULL;
cleanup_queue:
    vQueueDelete(s_rx_queue);
    s_rx_queue = NULL;
cleanup_mutex:
    vSemaphoreDelete(s_utc_mutex);
    s_utc_mutex = NULL;
    return ret;
}

esp_err_t can_manager_deinit(void)
{
    esp_err_t ret = ESP_OK;

    if (s_node != NULL) {
        ret = twai_node_disable(s_node);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "twai_node_disable: %s", esp_err_to_name(ret));
        }
        twai_node_delete(s_node);
        s_node = NULL;
    }

    if (s_rx_task != NULL) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }

    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }

    if (s_utc_mutex != NULL) {
        vSemaphoreDelete(s_utc_mutex);
        s_utc_mutex = NULL;
    }

    ESP_LOGI(TAG, "CAN manager stopped");
    return ret;
}