/**
 * @file wifi_manager.h
 * @brief Wi-Fi Access Point and HTTP management server.
 *
 * wifi_manager starts the ESP32 in soft-AP mode and hosts a single-page web
 * application that allows a connected phone or laptop to:
 *  - Scan for nearby GoPro cameras.
 *  - Pair (bond) a camera to the controller.
 *  - View live camera status.
 *  - Manually start / stop recording on all cameras.
 *  - Reset all stored BLE bonds.
 *
 * Network details:
 *  - SSID: HERO-RC-XXXXXX (last 3 bytes of AP MAC address)
 *  - Security: Open (no password)
 *  - IP address: 10.71.79.1
 *  - Max simultaneous clients: 4
 *
 * The embedded web UI (components/wifi_manager/www/index.html) is compiled
 * into the firmware image via the ESP-IDF EMBED_FILES mechanism and served
 * directly from flash — no file system required.
 *
 * HTTP API endpoints are documented in API.md.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the Wi-Fi AP and start the HTTP server.
 *
 * Performs the following in order:
 *  1. Initialises esp_netif and creates the default Wi-Fi AP interface.
 *  2. Assigns static IP 10.71.79.1/24 and starts the DHCP server.
 *  3. Configures and starts the Wi-Fi AP (SSID derived from MAC, open auth).
 *  4. Starts the ESP-IDF HTTP server and registers all /api/* URI handlers.
 *
 * Call once from app_main() after camera_manager_init() and ble_core_init(),
 * as the HTTP handlers call into both of those components.
 */
void wifi_manager_init(void);

#ifdef __cplusplus
}
#endif
