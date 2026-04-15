/**
 * @file ble_core.h
 * @brief BLE Central stack abstraction built on NimBLE.
 *
 * ble_core owns the NimBLE host task, GAP scanning, and connection lifecycle.
 * It is camera-agnostic: higher layers (gopro_ble) register a callback struct
 * and are notified of every relevant event.
 *
 * Typical initialisation order (see main.c):
 *  1. Register callbacks via ble_core_register_callbacks().
 *  2. Call ble_core_init() to start the NimBLE stack.
 *     The stack fires on_sync once it is ready, which kicks off the
 *     boot reconnect chain automatically.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/ble_gap.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Callback type definitions
 * ============================================================ */

/**
 * @brief Called for every advertisement packet seen during a discovery scan.
 *
 * Invoked on the NimBLE host task.  Must not block.
 *
 * @param disc    Raw discovery descriptor (address, RSSI, event type, etc.).
 * @param fields  Pre-parsed AD fields (name, UUIDs, manufacturer data, etc.).
 */
typedef void (*ble_core_on_disc_cb_t)(const struct ble_gap_disc_desc *disc,
                                       const struct ble_hs_adv_fields *fields);

/**
 * @brief Called when a BLE connection is established (before encryption).
 *
 * At this point the link is up but the GoPro will not accept GATT commands
 * until the encrypted callback fires.
 *
 * @param conn_handle  NimBLE connection handle.
 * @param addr         Peer BLE address.
 */
typedef void (*ble_core_on_connected_cb_t)(uint16_t conn_handle, const ble_addr_t *addr);

/**
 * @brief Called when the link is encrypted (pairing / bond resumption complete).
 *
 * This is the correct point to begin GATT service discovery and subscribe to
 * notifications.
 *
 * @param conn_handle  NimBLE connection handle.
 * @param addr         Peer BLE address.
 */
typedef void (*ble_core_on_encrypted_cb_t)(uint16_t conn_handle, const ble_addr_t *addr);

/**
 * @brief Called when a BLE connection is dropped for any reason.
 *
 * The higher layer should clear all state associated with conn_handle and
 * trigger a reconnect attempt if appropriate.
 *
 * @param conn_handle  NimBLE connection handle of the dropped link.
 * @param addr         Peer BLE address (may be zeroed if unknown).
 * @param reason       BLE host error code (BLE_ERR_*) describing the cause.
 */
typedef void (*ble_core_on_disconnected_cb_t)(uint16_t conn_handle, const ble_addr_t *addr, int reason);

/**
 * @brief Called when an ATT notification or indication is received.
 *
 * Invoked on the NimBLE host task.  The data buffer is only valid for the
 * duration of the callback — copy any bytes you need.
 *
 * @param conn_handle  Connection the notification arrived on.
 * @param attr_handle  GATT attribute handle of the notifying characteristic.
 * @param data         Notification payload.
 * @param len          Payload length in bytes.
 */
typedef void (*ble_core_on_notify_rx_cb_t)(uint16_t conn_handle, uint16_t attr_handle,
                                            const uint8_t *data, uint16_t len);

/**
 * @brief Predicate that tells ble_core whether an address belongs to a
 *        registered camera.
 *
 * ble_core calls this during background scanning to decide whether to attempt
 * an automatic reconnect to an advertising device.  Returning true causes
 * ble_core to initiate a connection without waiting for an explicit
 * ble_core_connect_by_addr() call.
 *
 * Implemented by camera_manager_is_known_addr() in the reference build.
 *
 * @param addr  Advertising device address.
 * @return true if the address is a registered camera; false otherwise.
 */
typedef bool (*ble_core_is_known_addr_cb_t)(const ble_addr_t *addr);

/**
 * @brief Predicate that tells ble_core whether any configured camera is
 *        currently disconnected.
 *
 * ble_core calls this before starting a background scan to decide whether
 * scanning serves any purpose.  If all known cameras are already connected,
 * scanning wastes radio resources and is suppressed.
 *
 * Implemented by camera_manager_has_disconnected_cameras() in the reference
 * build.
 *
 * @return true if at least one configured camera is not connected;
 *         false if all cameras are connected (or none are configured).
 */
typedef bool (*ble_core_has_disconnected_cameras_cb_t)(void);

/* ============================================================
 * Callback struct
 * ============================================================ */

/**
 * @brief Aggregated callback table registered with ble_core_register_callbacks().
 *
 * All fields are optional — set unused callbacks to NULL.
 */
typedef struct {
    ble_core_on_disc_cb_t          on_disc;        /**< Advertisement seen during discovery. */
    ble_core_on_connected_cb_t     on_connected;   /**< Connection established. */
    ble_core_on_encrypted_cb_t     on_encrypted;   /**< Link encrypted — safe to use GATT. */
    ble_core_on_disconnected_cb_t  on_disconnected;/**< Connection dropped. */
    ble_core_on_notify_rx_cb_t     on_notify_rx;   /**< ATT notification / indication received. */
    ble_core_is_known_addr_cb_t             is_known_addr;           /**< Returns true for registered cameras. */
    ble_core_has_disconnected_cameras_cb_t  has_disconnected_cameras;/**< Returns true if any camera is not connected. */
} ble_core_callbacks_t;

/* ============================================================
 * Public API
 * ============================================================ */

/**
 * @brief Register the callback table used for all BLE events.
 *
 * Must be called before ble_core_init() so that no events are missed
 * during stack start-up.  Overwrites any previously registered callbacks.
 *
 * @param cbs  Pointer to a fully initialised callback struct.  The struct is
 *             copied by value; the caller does not need to keep it alive.
 */
void ble_core_register_callbacks(const ble_core_callbacks_t *cbs);

/**
 * @brief Initialise the NimBLE stack and start the host task.
 *
 * Configures the security manager for bonding with no I/O capability
 * (Just Works pairing).  Once the stack is ready, on_sync fires and
 * ble_core automatically attempts to reconnect any stored bonds before
 * falling back to a passive background scan.
 *
 * Call once after ble_core_register_callbacks().
 */
void ble_core_init(void);

/**
 * @brief Start a 30-second active discovery scan.
 *
 * All advertisement packets received during the scan are forwarded to the
 * on_disc callback.  Deduplication is disabled so every packet is surfaced.
 * At the end of 30 seconds the scan automatically returns to the passive
 * background scan.
 *
 * Safe to call from any task — the request is posted to the NimBLE event queue.
 */
void ble_core_start_discovery(void);

/**
 * @brief Cancel a running discovery scan and resume the background scan.
 *
 * No-op if no discovery scan is currently active.
 */
void ble_core_stop_discovery(void);

/**
 * @brief Initiate a connection to a specific BLE address.
 *
 * The connection is deferred until the target device is seen advertising.
 * Safe to call from any task.
 *
 * @param addr  Target device address.  Must remain valid until the call returns.
 */
void ble_core_connect_by_addr(const ble_addr_t *addr);

/**
 * @brief Delete all BLE bonds except those in the keep list.
 *
 * Used to clear stale bonds after cameras are removed via the web UI.
 * Bonds not present in @p keep are erased from NVS.
 *
 * @param keep        Array of addresses to retain.  Pass NULL to delete all bonds.
 * @param keep_count  Number of entries in @p keep.
 */
void ble_core_purge_unknown_bonds(const ble_addr_t *keep, int keep_count);

/**
 * @brief Write data to a remote GATT characteristic.
 *
 * Performs a Write Without Response (ATT write command) to the specified
 * attribute handle.  Non-blocking on success.
 *
 * @param conn_handle  Connection to write on.
 * @param attr_handle  Target GATT attribute handle.
 * @param data         Payload to send.
 * @param len          Payload length in bytes.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ble_core_gatt_write(uint16_t conn_handle, uint16_t attr_handle,
                               const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif
