/**
 * @file legacy_gopro.h
 * @brief Legacy GoPro (Hero4) Wi-Fi HTTP control component.
 *
 * When a station connects to the ESP32 SoftAP and is assigned a DHCP address,
 * legacy_gopro probes it via HTTP GET /gp/gpControl/status.  If the camera
 * responds with the expected JSON payload it is identified as a Hero4 and
 * placed in the "discovered" state.
 *
 * Discovered cameras are NOT sent keepalives or registered with camera_manager
 * until a user explicitly adds them via legacy_gopro_add_camera().  Added
 * cameras are "managed": they are registered with camera_manager (assigned a
 * slot), receive keepalives, are polled for status, and respond to shutter
 * commands.  Their MACs are persisted to NVS so they are automatically
 * promoted to managed on subsequent reconnects without user intervention.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define LEGACY_CAMERA_NAME_LEN  32

/**
 * @brief Describes an AP station that is connected but not yet added to
 *        camera_manager.  The device is not probed until the user clicks Add,
 *        so only the MAC and IP are available at this stage.
 */
typedef struct {
    uint8_t  mac[6];
    uint32_t ip_addr;   /**< Network byte order */
} legacy_discovered_camera_t;

/**
 * @brief Initialize the legacy GoPro component.
 *
 * Loads managed MACs from NVS, opens the UDP socket, and starts the internal
 * FreeRTOS task and UDP RX task.  Must be called after camera_manager_init()
 * and wifi_manager_init().
 */
void legacy_gopro_init(void);

/**
 * @brief Notify the component that a Wi-Fi station has been assigned a DHCP
 *        IP address on the SoftAP.
 *
 * Posts an asynchronous CMD_STATION_CONNECT.  The station is recorded in the
 * internal table and will appear in the discovered list returned by
 * legacy_gopro_get_discovered().  No HTTP probe is performed at this point;
 * probing happens only when the user explicitly adds the camera.
 *
 * If the station's MAC matches a previously saved (managed) MAC it is
 * automatically promoted to managed without requiring user action.
 *
 * Safe to call from any context.
 *
 * @param ip   Station IPv4 address (network-byte-order uint32_t).
 * @param mac  Station MAC address (6 bytes).
 */
void legacy_gopro_on_station_connected(uint32_t ip, const uint8_t mac[6]);

/**
 * @brief Notify the component that a Wi-Fi station has disconnected.
 *
 * Posts an asynchronous disconnect command.  Managed cameras are kept in the
 * table (slot stays visible as "disconnected") so they auto-promote on
 * reconnect.  Unmanaged/discovered cameras are discarded.
 *
 * Safe to call from any context.
 *
 * @param mac  MAC address of the disconnected station (6 bytes).
 */
void legacy_gopro_on_station_disconnected(const uint8_t mac[6]);

/**
 * @brief Return a snapshot of Hero4 cameras that are connected but not yet
 *        managed (i.e. available to be added by the user).
 *
 * Safe to call from any context (reads internal state without a mutex; minor
 * races are acceptable for display purposes).
 *
 * @param out        Caller-provided array to fill.
 * @param max_count  Size of the array.
 * @return           Number of discovered (unmanaged, connected) cameras.
 */
int  legacy_gopro_get_discovered(legacy_discovered_camera_t *out, int max_count);

/**
 * @brief Promote a discovered camera to managed status.
 *
 * Posts an asynchronous CMD_ADD_CAMERA to the internal task queue.  The task
 * registers the camera with camera_manager, performs the keepalive settle
 * loop, marks it as connected, and saves the MAC to NVS.
 *
 * @param mac  6-byte MAC of the discovered camera to add.
 * @return     true if the command was queued, false if the queue is full.
 */
bool legacy_gopro_add_camera(const uint8_t mac[6]);

/**
 * @brief Remove a managed camera from camera_manager.
 *
 * Posts an asynchronous CMD_REMOVE_CAMERA.  The task unregisters the camera,
 * frees the camera_manager slot, and removes the MAC from NVS.  If the camera
 * is still physically connected it will reappear in the discovered list.
 *
 * @param mac  6-byte MAC of the managed camera to remove.
 * @return     true if the command was queued, false if the queue is full.
 */
bool legacy_gopro_remove_camera(const uint8_t mac[6]);

/**
 * @brief Check whether a camera_manager slot belongs to a managed legacy camera.
 *
 * Used by wifi_manager to determine whether /api/remove-camera should invoke
 * ble_core_remove_bond() (BLE camera) or legacy_gopro_remove_camera() (Wi-Fi).
 *
 * @param slot  0-based camera_manager slot index.
 * @return      true if the slot is occupied by a managed legacy Wi-Fi camera.
 */
bool legacy_gopro_is_managed_slot(int slot);
