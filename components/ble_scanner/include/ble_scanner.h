#pragma once

/**
 * Initialize the NimBLE stack and register callbacks.
 * Call this once from app_main before anything else.
 */
void ble_scanner_init(void);

/**
 * Start a passive BLE scan for all advertising devices.
 * Logs each device address and RSSI to the console.
 * Called automatically after the BLE stack syncs.
 */
void ble_scanner_start(void);
