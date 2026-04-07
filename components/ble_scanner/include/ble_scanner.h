#pragma once

#include <stdint.h>
#include "host/ble_gap.h"

#define GOPRO_MAX_DISCOVERED 10

/**
 * A GoPro camera found during a discovery scan.
 */
typedef struct {
    char       name[32];
    ble_addr_t addr;
    int8_t     rssi;
} gopro_device_t;

/**
 * Initialize the NimBLE stack and register callbacks.
 * Call this once from app_main before anything else.
 */
void ble_init(void);

/**
 * Start a 30-second discovery scan. Found GoPros are collected
 * into an internal list without connecting. Results are retrieved
 * with ble_scanner_get_discovered().
 */
void ble_scanner_start_discovery(void);

/**
 * Copy discovered cameras into the provided array.
 * Returns the number of cameras found (up to max_count).
 */
int ble_scanner_get_discovered(gopro_device_t *out, int max_count);

/**
 * Stop discovery mode, connect to the camera at the given address,
 * and resume normal auto-reconnect scanning afterwards.
 */
void ble_scanner_connect_by_addr(const ble_addr_t *addr);

/**
 * Delete any stored NimBLE bonds whose address is not in the keep list.
 * Pass the MAC addresses of all known paired cameras to preserve them.
 * Pass keep=NULL / keep_count=0 to delete all bonds.
 * Safe to call from any task — executes on the NimBLE host task.
 */
void ble_scanner_purge_unknown_bonds(const ble_addr_t *keep, int keep_count);
