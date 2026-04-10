/**
 * @file camera_driver.h
 * @brief Camera driver interface (vtable) used by camera_manager.
 *
 * camera_manager is camera-protocol-agnostic.  Each supported camera type
 * (e.g. GoPro via BLE) provides a camera_driver_t struct containing function
 * pointers for the three operations camera_manager needs to perform:
 * start/stop recording and query recording status.
 *
 * Adding a new camera type:
 *  1. Add a new CAMERA_TYPE_* enumerator.
 *  2. Implement the three vtable functions.
 *  3. Call camera_manager_register_driver() during init with the vtable and a
 *     factory function that allocates the per-camera context.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Enumerations
 * ============================================================ */

/**
 * @brief Recording state as reported by the camera driver.
 *
 * Returned by camera_driver_t::get_recording_status().  camera_manager uses
 * this value to derive the CAMERA_STATUS_* constants it exposes to callers and
 * the CAN camera_state_t values it sends to can_manager.
 */
typedef enum {
    CAMERA_RECORDING_UNKNOWN = 0, /**< Status not yet known (e.g. GATT not ready). */
    CAMERA_RECORDING_IDLE,        /**< Camera is connected but not recording. */
    CAMERA_RECORDING_ACTIVE,      /**< Camera is actively recording. */
} camera_recording_status_t;

/**
 * @brief Identifies which driver implementation manages a camera slot.
 *
 * Persisted to NVS so the correct driver can be reloaded on boot.
 */
typedef enum {
    CAMERA_TYPE_NONE      = 0, /**< Slot is unconfigured — no driver assigned. */
    CAMERA_TYPE_GOPRO_BLE,     /**< GoPro camera controlled via BLE (gopro_ble component). */
} camera_type_t;

/* ============================================================
 * Driver vtable
 * ============================================================ */

/** Forward declaration to allow self-referential typedef. */
typedef struct camera_driver camera_driver_t;

/**
 * @brief Camera driver vtable.
 *
 * All function pointers receive an opaque @p ctx that was allocated by the
 * driver's factory function (the create_ctx argument to
 * camera_manager_register_driver()).  The context holds all per-camera state
 * the driver needs (e.g. BLE connection handle, GATT attribute handles).
 *
 * All functions are called from camera_manager's task context (FreeRTOS timer
 * callback or BLE event handler).  Implementations must not block indefinitely.
 */
struct camera_driver {
    /**
     * @brief Command the camera to start recording.
     *
     * @param ctx  Per-camera driver context.
     * @return ESP_OK if the command was dispatched successfully.
     *         ESP_ERR_INVALID_STATE if the camera is not ready (e.g. no GATT handles).
     */
    esp_err_t (*start_recording)(void *ctx);

    /**
     * @brief Command the camera to stop recording.
     *
     * @param ctx  Per-camera driver context.
     * @return ESP_OK if the command was dispatched successfully.
     *         ESP_ERR_INVALID_STATE if the camera is not ready.
     */
    esp_err_t (*stop_recording)(void *ctx);

    /**
     * @brief Query the current recording state of the camera.
     *
     * This function must be non-blocking and return the last known state
     * cached in @p ctx.  The GoPro driver updates this cache in response to
     * asynchronous GATT status notifications polled at STATUS_POLL_INTERVAL_MS.
     *
     * @param ctx  Per-camera driver context.
     * @return Current recording state.
     */
    camera_recording_status_t (*get_recording_status)(void *ctx);
};

#ifdef __cplusplus
}
#endif
