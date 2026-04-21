#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "camera_driver.h"
#include "host/ble_gap.h"
#include "sdkconfig.h"

/* Max cameras equals the NimBLE bond limit — one bond per camera. */
#define CAMERA_MAX_SLOTS       CONFIG_BT_NIMBLE_MAX_BONDS
#define CAMERA_NAME_LEN        32
#define CAMERA_MODEL_NAME_LEN  32

#define CAMERA_STATUS_NOT_CONFIGURED  (-1)
#define CAMERA_STATUS_DISCONNECTED      0
#define CAMERA_STATUS_CONNECTED         1
#define CAMERA_STATUS_RECORDING         2

typedef struct {
    int        index;
    char       name[CAMERA_NAME_LEN];
    char       model_name[CAMERA_MODEL_NAME_LEN]; /**< Camera model string from GetHardwareInfo, e.g. "HERO12 Black". Empty until hw_info is received. */
    ble_addr_t mac_address;
    bool       is_configured;
    int        status;
} camera_slot_info_t;

void camera_manager_init(void);
int  camera_manager_find_by_addr(const ble_addr_t *addr);
int  camera_manager_find_free_slot(void);
int  camera_manager_remembered_count(void);
int  camera_manager_connected_count(void);
void camera_manager_on_connected(int slot, uint16_t conn_handle);
void camera_manager_on_disconnected(uint16_t conn_handle);
void camera_manager_set_gatt_ready(int slot, bool ready);
int  camera_manager_register_new(const ble_addr_t *addr, const char *name,
                                  const camera_driver_t *driver, void *driver_ctx,
                                  camera_type_t type);
int  camera_manager_find_by_handle(uint16_t conn_handle);
uint16_t camera_manager_get_handle(int slot);
bool camera_manager_is_gatt_ready(int slot);
void *camera_manager_get_driver_ctx(int slot);
esp_err_t camera_manager_save_slot(int slot);
esp_err_t camera_manager_remove_slot(int slot);
camera_slot_info_t camera_manager_get_slot_info(int slot);
void camera_manager_set_model_name(int slot, const char *model_name);
void camera_manager_set_desired_recording(bool recording);
int  camera_manager_start_recording_all(void);
int  camera_manager_stop_recording_all(void);

/**
 * @brief Start or stop recording on a single camera slot.
 *
 * Sets desired_recording for the slot (so the 2-second tick timer will retry
 * if the command cannot be dispatched immediately) and immediately dispatches
 * the command if the slot is connected and GATT-ready.
 *
 * Note: calling camera_manager_set_desired_recording() or the _all variants
 * afterwards will overwrite the per-slot desired state for all slots.
 *
 * @param slot  0-based slot index.
 * @param on    true = start recording, false = stop recording.
 * @return      1 if the command was dispatched, 0 if the slot was not ready.
 */
int  camera_manager_set_recording_slot(int slot, bool on);
bool camera_manager_is_known_addr(const ble_addr_t *addr);
bool camera_manager_has_disconnected_cameras(void);

/* -----------------------------------------------------------------------
 * Legacy Wi-Fi camera API (CAMERA_TYPE_LEGACY_WIFI)
 *
 * Wi-Fi cameras (e.g. GoPro Hero4) connect to the ESP32 SoftAP and are
 * controlled via the gpControl HTTP API.  Unlike BLE cameras they are NOT
 * persisted to NVS — they re-probe on every reconnect.
 * ----------------------------------------------------------------------- */

/**
 * @brief Register a newly-identified Wi-Fi camera.
 *
 * Looks up an existing slot by MAC address first; if found, updates the
 * driver and context pointer (handles reconnects).  If not found, allocates
 * a new slot.  Does NOT save to NVS.
 *
 * @param mac        Station MAC address (6 bytes).
 * @param name       Human-readable camera name (e.g. from gpControl status).
 * @param driver     Pointer to the driver vtable.
 * @param driver_ctx Per-camera driver context (must remain valid for the slot lifetime).
 * @return           Slot index on success, -1 if no slots are available.
 */
int camera_manager_register_wifi_camera(const uint8_t mac[6], const char *name,
                                         const camera_driver_t *driver, void *driver_ctx);

/**
 * @brief Mark a Wi-Fi camera slot as connected and store its current IP.
 *
 * @param slot  0-based slot index returned by camera_manager_register_wifi_camera().
 * @param ip    Station IPv4 address (network-byte-order uint32_t).
 */
void camera_manager_on_wifi_connected(int slot, uint32_t ip);

/**
 * @brief Mark the Wi-Fi camera with the given MAC as disconnected.
 *
 * Looks up the slot by MAC and clears the wifi_connected flag.  The slot
 * remains allocated so the camera shows as "disconnected" in the UI until
 * it reconnects and re-probes.
 *
 * @param mac  MAC address of the disconnected station (6 bytes).
 */
void camera_manager_on_wifi_disconnected_by_mac(const uint8_t mac[6]);
void camera_manager_register_driver(camera_type_t type,
                                     const camera_driver_t *driver,
                                     void *(*create_ctx)(void));

/**
 * @brief Enable or disable automatic camera control via the CAN logging state.
 *
 * When enabled (the default on every boot), the on_logging_state_changed
 * callback in main.c will start/stop cameras in response to RaceCapture
 * isLogging transitions.  When disabled, those transitions are ignored and
 * cameras must be controlled manually via the web UI shutter buttons.
 *
 * The flag always resets to true on restart — it is never stored in NVS.
 *
 * @param enabled  true = automatic control active, false = manual only.
 */
void camera_manager_set_auto_control(bool enabled);

/**
 * @brief Return the current automatic camera control state.
 *
 * @return true if automatic control is active, false if overridden to manual.
 */
bool camera_manager_get_auto_control(void);

/**
 * @brief Callback fired when a camera slot's derived status changes.
 *
 * Invoked from camera_manager_tick() (every 2 s) and immediately from
 * on_connected / on_disconnected / set_gatt_ready so that the CAN status
 * broadcast stays current without polling.
 *
 * @param slot    Camera slot index (0-based).
 * @param status  One of the CAMERA_STATUS_* constants defined above.
 * @param ctx     Opaque context registered with the callback.
 */
typedef void (*camera_state_change_fn_t)(int slot, int status, void *ctx);

/**
 * @brief Register a callback to receive camera slot state changes.
 *
 * Call before camera_manager_init() so that no transitions are missed
 * during the initial slot load.  Only one callback is supported at a time.
 *
 * @param cb   Callback function.
 * @param ctx  Opaque context pointer passed to the callback unchanged.
 */
void camera_manager_register_state_change_callback(camera_state_change_fn_t cb, void *ctx);
