#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * CAN Bus Configuration
 * Change these values to adapt the CAN interface for your setup.
 * ============================================================ */

/** Baud rate in bits per second.  1 Mbps is the rate used with RaceCapture.
 *  Supported values: 100000, 125000, 250000, 500000, 1000000. */
#define CAN_MANAGER_BITRATE_BPS     1000000U

/** GPIO connected to the CAN transceiver TX line (board-fixed for CAN 1). */
#define CAN_MANAGER_TX_GPIO         GPIO_NUM_7

/** GPIO connected to the CAN transceiver RX line (board-fixed for CAN 1). */
#define CAN_MANAGER_RX_GPIO         GPIO_NUM_6

/** Depth of the software RX queue (frames buffered between ISR and task). */
#define CAN_MANAGER_RX_QUEUE_DEPTH  32

/** Depth of the hardware TX queue. */
#define CAN_MANAGER_TX_QUEUE_DEPTH  8

/** FreeRTOS priority of the CAN processing task. */
#define CAN_MANAGER_TASK_PRIORITY   5

/** How often the ESP32 broadcasts camera status to RaceCapture (milliseconds).
 *  200 ms = 5 Hz.  Must be a multiple of the task tick period (100 ms). */
#define CAN_MANAGER_TX_INTERVAL_MS  200U

/* ============================================================
 * CAN Protocol — Message IDs
 * Both sides use standard 11-bit (non-extended) frames.
 * ============================================================ */

/** RaceCapture → ESP32: logging control commands. */
#define CAN_ID_RC_COMMAND   0x600U

/** ESP32 → RaceCapture: camera status broadcast. */
#define CAN_ID_CAM_STATUS   0x601U

/** RaceCapture → ESP32: UTC timestamp (millisecond Unix epoch, little-endian 64-bit).
 *  Broadcast by the RaceCapture Lua script at 25 Hz once GPS lock is acquired.
 *  Temporary ID — will migrate to the developer's standard ID when native
 *  UTC broadcast is available in firmware. */
#define CAN_ID_RC_UTC       0x602U

/* ============================================================
 * CAN Termination Note (HARDWARE — not software controlled)
 * ============================================================
 * The ESP32-CAN-X2 board has 120-ohm termination resistors that are
 * ENABLED by default via solder-jumper pads (TERM1/TERM2) on the back
 * of the board.
 *
 *  - End node (most installations): leave the jumpers intact.
 *  - Middle node: scratch the copper trace between TERM1 and TERM2 pads
 *    to disable termination.  See board documentation for details.
 *
 * Incorrect termination causes bus errors and silent communication
 * failures.  Always verify before debugging CAN issues in software.
 * ============================================================ */

/* ============================================================
 * Public Types
 * ============================================================ */

/**
 * @brief State of a single camera slot, transmitted in 0x601 frames.
 *
 * Values are defined by the CAN protocol and must match the RaceCapture
 * direct-CAN channel mapping (Unsigned, offset 0–3, multiplier 1, adder 0).
 * Do not reorder without updating the RaceCapture configuration.
 */
typedef enum {
    CAMERA_STATE_UNDEFINED    = 0,  /**< Slot not configured / no information yet */
    CAMERA_STATE_DISCONNECTED = 1,  /**< Camera not found or connection lost */
    CAMERA_STATE_IDLE         = 2,  /**< Connected, not recording */
    CAMERA_STATE_RECORDING    = 3,  /**< Connected and actively recording */
} camera_state_t;

/** Maximum number of camera slots supported by the CAN protocol. */
#define CAN_MANAGER_MAX_CAMERAS  4U

/**
 * @brief Self-contained CAN frame delivered to the raw RX callback.
 *
 * All data is copied by value — no pointers into ISR memory.
 */
typedef struct {
    uint32_t id;            /**< CAN identifier (11-bit standard or 29-bit extended) */
    uint8_t  data[8];       /**< Frame payload bytes */
    uint8_t  data_len;      /**< Actual payload length in bytes (derived from DLC) */
    uint8_t  dlc;           /**< Raw DLC value as received */
    bool     is_extended;   /**< true = 29-bit extended ID */
    bool     is_rtr;        /**< true = remote transmission request (no data) */
} can_frame_t;

/**
 * @brief Raw frame callback — invoked for every received frame, from task context.
 *
 * Primarily useful for development and bus sniffing.  Known protocol messages
 * (0x600) are also dispatched to their specific callbacks before this fires.
 *
 * @param frame     Pointer to the received frame.  Valid only during the callback.
 * @param user_ctx  Opaque context registered with can_manager_register_rx_callback().
 */
typedef void (*can_rx_frame_cb_t)(const can_frame_t *frame, void *user_ctx);

/**
 * @brief Callback invoked when the RaceCapture logging state changes (0x600).
 *
 * Only fired when the state actually changes, not on every received frame.
 *
 * @param is_logging  true if RaceCapture is actively logging, false if stopped.
 * @param user_ctx    Opaque context registered with can_manager_register_logging_callback().
 */
typedef void (*can_logging_state_cb_t)(bool is_logging, void *user_ctx);

/* ============================================================
 * Public API
 * ============================================================ */

/**
 * @brief Initialise the CAN manager, start the TWAI node, and begin
 *        the 5 Hz camera status broadcast.
 *
 * Must be called once before any other can_manager function.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t can_manager_init(void);

/**
 * @brief Stop the TWAI node and release all resources.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t can_manager_deinit(void);

/**
 * @brief Register a callback for raw received CAN frames (all IDs).
 *
 * Optional — primarily for development and debugging.  Only one callback
 * is supported at a time.
 *
 * @param cb        Callback function (must not be NULL).
 * @param user_ctx  Opaque context pointer passed to the callback unchanged.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if cb is NULL.
 */
esp_err_t can_manager_register_rx_callback(can_rx_frame_cb_t cb, void *user_ctx);

/**
 * @brief Register a callback for RaceCapture logging state changes (0x600).
 *
 * The callback fires only when the isLogging value changes, not on every frame.
 *
 * @param cb        Callback function (must not be NULL).
 * @param user_ctx  Opaque context pointer passed to the callback unchanged.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if cb is NULL.
 */
esp_err_t can_manager_register_logging_callback(can_logging_state_cb_t cb, void *user_ctx);

/**
 * @brief Get the current best-estimate UTC time in milliseconds.
 *
 * Uses the last received 0x602 timestamp plus elapsed time from the ESP32's
 * monotonic clock (esp_timer_get_time) to extrapolate the current UTC without
 * waiting for the next CAN frame.
 *
 * Thread-safe: may be called from any task.
 *
 * @param[out] epoch_ms_out  Receives the estimated current Unix epoch in milliseconds.
 *                           Undefined if the function returns false.
 * @return true  if a valid UTC has been received from the RaceCapture.
 * @return false if GPS lock has not yet been established (no valid frame received).
 */
bool can_manager_get_utc_ms(uint64_t *epoch_ms_out);

/**
 * @brief Update the recorded state of one camera slot.
 *
 * The new state will be included in the next 0x601 broadcast (within 200 ms).
 * Thread-safe: may be called from any task.
 *
 * @param camera_idx  Camera index, 0–(CAN_MANAGER_MAX_CAMERAS-1).
 * @param state       New state for this camera.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if camera_idx or state is out of range.
 */
esp_err_t can_manager_set_camera_state(uint8_t camera_idx, camera_state_t state);

#ifdef __cplusplus
}
#endif
