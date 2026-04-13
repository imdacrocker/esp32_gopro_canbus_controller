#include "nvs_flash.h"
#include "esp_log.h"
#include "ble_core.h"
#include "open_gopro_ble.h"
#include "camera_manager.h"
#include "wifi_manager.h"
#include "can_manager.h"

static const char *TAG = "main";

/* ============================================================
 * Camera-state → CAN bridge
 * ============================================================ */

/**
 * Called by camera_manager whenever a slot's derived status changes
 * (connected, disconnected, GATT ready, recording started/stopped).
 *
 * Maps the internal CAMERA_STATUS_* value to the CAN camera_state_t
 * enum and forwards it to can_manager so the 0x601 broadcast stays current.
 *
 *  CAMERA_STATUS_NOT_CONFIGURED (-1) → CAMERA_STATE_UNDEFINED    (0)
 *  CAMERA_STATUS_DISCONNECTED   ( 0) → CAMERA_STATE_DISCONNECTED (1)
 *  CAMERA_STATUS_CONNECTED      ( 1) → CAMERA_STATE_IDLE         (2)
 *  CAMERA_STATUS_RECORDING      ( 2) → CAMERA_STATE_RECORDING    (3)
 */
static void on_camera_state_changed(int slot, int status, void *user_ctx)
{
    (void)user_ctx;

    camera_state_t can_state;
    switch (status) {
        case CAMERA_STATUS_DISCONNECTED: can_state = CAMERA_STATE_DISCONNECTED; break;
        case CAMERA_STATUS_CONNECTED:    can_state = CAMERA_STATE_IDLE;         break;
        case CAMERA_STATUS_RECORDING:    can_state = CAMERA_STATE_RECORDING;    break;
        default:                         can_state = CAMERA_STATE_UNDEFINED;    break;
    }

    can_manager_set_camera_state((uint8_t)slot, can_state);
}

/* ============================================================
 * CAN Callbacks
 * ============================================================ */

/**
 * Called by can_manager when the RaceCapture logging state changes (0x600).
 *
 * Sets the desired recording state on all camera slots so the tick timer
 * keeps retrying if a camera is mid-reconnect, then immediately dispatches
 * the start/stop command to every camera that is already GATT-ready.
 */
static void on_logging_state_changed(bool is_logging, void *user_ctx)
{
    if (is_logging) {
        ESP_LOGI(TAG, "RaceCapture logging STARTED — commanding cameras to record");

        /* Set the desired state first so the camera_manager tick timer will
         * keep trying to record even if a camera is mid-reconnect. */
        camera_manager_set_desired_recording(true);

        /* Immediately dispatch the start command to all cameras that are
         * already connected and GATT-ready, without waiting for the next
         * 2-second tick.  Returns the number of cameras commanded. */
        int n = camera_manager_start_recording_all();
        ESP_LOGI(TAG, "Start recording dispatched to %d camera(s)", n);
    } else {
        ESP_LOGI(TAG, "RaceCapture logging STOPPED — commanding cameras to stop");

        /* Clear desired state so the tick timer stops trying to restart. */
        camera_manager_set_desired_recording(false);

        int n = camera_manager_stop_recording_all();
        ESP_LOGI(TAG, "Stop recording dispatched to %d camera(s)", n);
    }
}

/* ============================================================
 * Entry Point
 * ============================================================ */

void app_main(void)
{
    /* NVS is required by the BLE stack for storing bonding info. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Register camera drivers BEFORE camera_manager_init() loads NVS so that
     * the driver factory is available when stored camera records are rehydrated
     * into driver contexts. */
    open_gopro_ble_init();

    /* Register the camera-state → CAN bridge BEFORE camera_manager_init() so
     * that the initial slot-load notifications (if any) are not missed. */
    camera_manager_register_state_change_callback(on_camera_state_changed, NULL);

    /* Load remembered cameras from NVS into RAM and start the tick timer. */
    camera_manager_init();

    wifi_manager_init();

    /* Start the NimBLE stack.  on_sync fires once the stack is ready and
     * kicks off the boot reconnect chain. */
    ble_core_init();

    /* Register the logging-state callback before init so no transitions
     * are missed during startup. */
    can_manager_register_logging_callback(on_logging_state_changed, NULL);

    ret = can_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN manager init failed: %s — check wiring and baud rate",
                 esp_err_to_name(ret));
        /* Non-fatal: the rest of the system continues without CAN. */
    }
}
