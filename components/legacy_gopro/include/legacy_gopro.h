/**
 * @file legacy_gopro.h
 * @brief Legacy GoPro (Hero4) Wi-Fi HTTP control component.
 *
 * Connected stations are tracked by wifi_manager's station table and exposed
 * to the web UI via /api/legacy/discovered.  No probing occurs automatically.
 *
 * When the user selects a device and clicks Add, /api/legacy/add calls
 * legacy_gopro_add_camera() with the station's MAC and known IP.  The
 * component probes the device via HTTP; if it responds as a Hero4 it is
 * registered with camera_manager and saved to NVS.
 *
 * Managed cameras are "remembered": their MAC and last-known IP are stored in
 * NVS so that on subsequent reconnects they are auto-promoted to managed
 * status without requiring user action.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define LEGACY_CAMERA_NAME_LEN  32

/**
 * @brief Initialize the legacy GoPro component.
 *
 * Loads managed MACs from NVS, opens the UDP socket, and starts the internal
 * FreeRTOS task and UDP RX task.  Must be called after camera_manager_init()
 * and wifi_manager_init().
 */
void legacy_gopro_init(void);

/**
 * @brief Notify the component that a station has completed Wi-Fi L2 association.
 *
 * If the MAC is in NVS (previously managed camera), CMD_STATION_CONNECT is
 * posted with the saved IP.  An HTTP probe runs before the camera is promoted
 * to managed.  If DHCP later fires with a different IP, legacy_gopro_on_station_connected()
 * will correct it via CMD_IP_UPDATE.
 *
 * Unknown MACs are ignored — they are tracked by wifi_manager's station table
 * and appear in /api/legacy/discovered once they have a DHCP IP.
 *
 * Safe to call from any context (ISR-safe queue post).
 *
 * @param mac  Station MAC address (6 bytes).
 */
void legacy_gopro_on_station_wifi_associated(const uint8_t mac[6]);

/**
 * @brief Notify the component that a station has been assigned a DHCP IP.
 *
 * Only acts on MACs that are already in NVS (managed cameras).  Posts
 * CMD_IP_UPDATE to persist the new IP and redirect live traffic if the IP
 * has changed (e.g. after the camera re-pairs).  Unknown MACs are ignored.
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
 * reconnect.  Unmanaged cameras are not tracked here and require no action.
 *
 * Safe to call from any context.
 *
 * @param mac  MAC address of the disconnected station (6 bytes).
 */
void legacy_gopro_on_station_disconnected(const uint8_t mac[6]);

/**
 * @brief Probe a connected station and promote it to managed if it is a Hero4.
 *
 * Posts an asynchronous CMD_ADD_CAMERA to the internal task queue.  The task
 * probes @p ip via HTTP GET /gp/gpControl/status; if the camera responds it
 * is registered with camera_manager, runs the keepalive settle loop, and has
 * its MAC and IP saved to NVS.
 *
 * After ~15 s the caller can poll /api/legacy/discovered: if the MAC has
 * disappeared the camera was successfully promoted; if it is still present
 * the probe failed and the device is not a Hero4.
 *
 * @p ip must be non-zero — /api/legacy/discovered only returns stations that
 * already have a DHCP address, so this should always be satisfied.
 *
 * @param mac  6-byte MAC of the station to probe.
 * @param ip   Station IP in network-byte-order (must not be 0).
 * @return     true if the command was queued, false if the queue is full.
 */
bool legacy_gopro_add_camera(const uint8_t mac[6], uint32_t ip);

/**
 * @brief Remove a managed camera from camera_manager.
 *
 * Posts an asynchronous CMD_REMOVE_CAMERA.  The task unregisters the camera,
 * frees the camera_manager slot, and removes the MAC from NVS.
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

/**
 * @brief Check whether a MAC address belongs to a currently managed legacy camera.
 *
 * Used by wifi_manager to filter managed cameras out of /api/legacy/discovered.
 * Safe to call from any context (lock-free read of internal state).
 *
 * @param mac  6-byte MAC address to check.
 * @return     true if the MAC is managed by this component.
 */
bool legacy_gopro_is_managed_mac(const uint8_t mac[6]);

/**
 * @brief Push the current UTC date/time to all managed+connected Wi-Fi cameras.
 *
 * Posts an asynchronous CMD_SYNC_TIME to the internal task queue.  The task
 * then calls the Hero4 HTTP date/time endpoint for each connected camera.
 * The call returns immediately — the actual HTTP work is done on the task.
 *
 * If UTC is not yet available (RaceCapture GPS lock not acquired) the HTTP
 * call is silently skipped for each camera.  Call this function again once
 * UTC becomes available, or rely on the can_utc_acquired callback to do so.
 *
 * Safe to call from any context (ISR-safe queue post).
 */
void legacy_gopro_sync_time_all(void);
