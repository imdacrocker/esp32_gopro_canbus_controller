/**
 * @file open_gopro_ble.h
 * @brief OpenGoPro BLE driver — discovery, pairing, and recording control.
 *
 * This component sits between ble_core (raw BLE) and camera_manager (slot
 * management).  It implements the OpenGoPro BLE command protocol and exposes
 * a camera_driver_t vtable so camera_manager can control GoPro cameras without
 * knowing anything about BLE.
 *
 * Initialisation order (see main.c):
 *  1. Call open_gopro_ble_init() to register the GoPro driver with
 *     camera_manager and register BLE event callbacks with ble_core.
 *  2. Call ble_core_init() to start the NimBLE stack.
 *
 * Discovery / pairing flow:
 *  1. open_gopro_ble_start_discovery() triggers a 30-second BLE scan.
 *  2. Poll open_gopro_ble_get_discovered() to enumerate detected cameras.
 *  3. open_gopro_ble_connect_by_addr() initiates pairing with a specific camera.
 *     On success the camera is registered with camera_manager and bonded for
 *     automatic reconnection on future boots.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "host/ble_gap.h"
#include "camera_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Constants
 * ============================================================ */

/** Maximum number of GoPro cameras that can be held in the discovery list. */
#define GOPRO_MAX_DISCOVERED  10

/**
 * @brief Interval at which the GoPro recording status is polled (milliseconds).
 *
 * A query packet is written to the GP-0076 (Query) characteristic every
 * STATUS_POLL_INTERVAL_MS milliseconds, requesting the encoding_active status
 * field (ID 8) from each connected, GATT-ready camera.
 */
#define STATUS_POLL_INTERVAL_MS  5000

/**
 * @brief Interval at which Keep Alive packets are sent to each connected camera.
 *
 * Per the OpenGoPro specification, a Keep Alive command (ID 0x5B) must be
 * sent to GP-0074 (Settings characteristic) every 3 seconds after connection
 * is established to prevent the camera from auto-sleeping.
 */
#define KEEP_ALIVE_INTERVAL_MS  3000

/* ============================================================
 * Types
 * ============================================================ */

/**
 * @brief A GoPro camera detected during a discovery scan.
 */
typedef struct {
    char       name[32];   /**< Advertised device name (null-terminated). */
    ble_addr_t addr;       /**< BLE address (6 bytes + type). */
    int8_t     rssi;       /**< Signal strength in dBm at time of discovery. */
} gopro_device_t;

/**
 * @brief GATT attribute handles discovered during service enumeration.
 *
 * Populated by gatt.c after the encrypted link is established.
 * All handle fields are zero until GATT discovery completes.
 *
 * GoPro BLE service UUIDs follow the OpenGoPro specification:
 * https://gopro.github.io/OpenGoPro/ble_2_0
 */
typedef struct {
    uint16_t cmd_write;             /**< GP-0072: Command write (start/stop recording, etc.). */
    uint16_t cmd_resp_notify;       /**< GP-0073: Command response notifications. */
    uint16_t settings_write;        /**< GP-0074: Settings write (also used for Keep Alive). */
    uint16_t settings_resp_notify;  /**< GP-0075: Settings response notifications. */
    uint16_t query_write;           /**< GP-0076: Query write (request status values). */
    uint16_t query_resp_notify;     /**< GP-0077: Query response notifications. */
    uint16_t net_mgmt_cmd_write;    /**< GP-0091: Network management command write. */
    uint16_t net_mgmt_resp_notify;  /**< GP-0092: Network management response. */
    uint16_t wifi_ssid_read;        /**< GP-0002: Wi-Fi AP SSID (readable). */
    uint16_t wifi_pass_read;        /**< GP-0003: Wi-Fi AP password (readable). */
    uint16_t wifi_power_write;      /**< GP-0001: Wi-Fi AP power on/off. */
    uint16_t wifi_state_indicate;   /**< GP-0004: Wi-Fi AP state indications. */
} gopro_gatt_handles_t;

/* ============================================================
 * Public API
 * ============================================================ */

/**
 * @brief Initialise the OpenGoPro BLE component.
 *
 * Performs the following in order:
 *  - Registers the GoPro camera_driver_t with camera_manager.
 *  - Registers BLE event callbacks with ble_core.
 *  - Starts the recording-status poll timer and the keep-alive timer.
 *
 * Must be called before camera_manager_init() so that the driver factory is
 * available when stored camera records are rehydrated from NVS.
 * Must be called before ble_core_init() so that no BLE events are missed.
 */
void open_gopro_ble_init(void);

/**
 * @brief Start a 30-second BLE discovery scan for GoPro cameras.
 *
 * Clears the previous discovery list and triggers a scan via ble_core.
 * Only devices advertising the GoPro service UUID (0xFEA6) are added to the
 * list.  Results are available immediately via open_gopro_ble_get_discovered().
 *
 * Safe to call from any task.
 */
void open_gopro_ble_start_discovery(void);

/**
 * @brief Retrieve the list of GoPro cameras found during the last scan.
 *
 * Copies up to @p max_count entries from the internal discovery list into
 * the caller-supplied buffer.  The list is stable between calls unless a
 * new scan is started with open_gopro_ble_start_discovery().
 *
 * @param out        Output buffer; must hold at least @p max_count entries.
 * @param max_count  Maximum number of entries to copy.
 * @return Number of entries written to @p out (0 if nothing was discovered).
 */
int open_gopro_ble_get_discovered(gopro_device_t *out, int max_count);

/**
 * @brief Initiate a BLE connection and pairing with a specific GoPro camera.
 *
 * The connection is deferred until the target device is seen advertising.
 * Once connected and encrypted, GATT handles are discovered and the camera is
 * registered with camera_manager (persisted to NVS) so it reconnects
 * automatically on future boots.
 *
 * Safe to call from any task.
 *
 * @param addr  BLE address of the camera to connect to.
 */
void open_gopro_ble_connect_by_addr(const ble_addr_t *addr);

/**
 * @brief Return a pointer to the GoPro camera_driver_t vtable.
 *
 * Used by open_gopro_ble_init() to register the driver with camera_manager.
 * The returned pointer is to static storage and is always valid.
 *
 * @return Pointer to the GoPro BLE camera driver.
 */
const camera_driver_t *open_gopro_ble_get_driver(void);

/**
 * @brief Send a SetDateTime command to every currently GATT-ready camera.
 *
 * Intended to be called from the can_manager UTC-acquired callback so that
 * cameras already connected at the moment GPS lock is first established also
 * have their clocks set.  For cameras that connect after UTC is already
 * available, control_send_set_date_time() is called directly from gatt.c.
 *
 * Fetches the current UTC from can_manager_get_utc_ms() once and uses it for
 * all slots in the same call.  Slots that are not GATT-ready are skipped
 * silently.
 *
 * Safe to call from any task.
 */
void open_gopro_ble_sync_time_all(void);

/**
 * @brief Allocate and initialise a per-camera driver context.
 *
 * Called by camera_manager each time a GoPro camera slot is loaded from NVS
 * or a new camera is registered.  The returned pointer is stored in the camera
 * slot and passed to all camera_driver_t vtable calls for that slot.
 *
 * @return Pointer to a heap-allocated gopro_ble_ctx_t, or NULL on OOM.
 */
void *open_gopro_ble_create_driver_ctx(void);

#ifdef __cplusplus
}
#endif
