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
 *
 * Typical call sequence in app_main():
 *
 *   wifi_manager_init();
 *   wifi_manager_wait_for_ap_ready();   // blocks until beacon is on air
 *   ble_core_init();                    // safe to start radio work now
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the Wi-Fi AP and start the HTTP server.
 *
 * Performs the following in order:
 *  1. Creates the AP-ready event group.
 *  2. Initialises esp_netif and creates the default Wi-Fi AP interface.
 *  3. Assigns static IP 10.71.79.1/24 and starts the DHCP server.
 *  4. Registers the internal WiFi event handler (AP_START, AP_STOP, etc.).
 *  5. Configures and starts the Wi-Fi AP (SSID derived from MAC, open auth,
 *     HT20 bandwidth applied inside the AP_START event handler).
 *  6. Starts the ESP-IDF HTTP server and registers all /api/ URI handlers.
 *
 * This function returns before the AP beacon is necessarily on air.
 * Call wifi_manager_wait_for_ap_ready() afterwards if you need to gate
 * other initialisation steps (e.g. BLE stack startup) on the AP being up.
 */
void wifi_manager_init(void);

/**
 * @brief Block until the Wi-Fi AP is on air (WIFI_EVENT_AP_START received).
 *
 * Waits up to 5 seconds for the AP to start broadcasting.  If the timeout
 * expires, a warning is logged and the function returns so startup can
 * continue — callers should not treat a timeout as fatal.
 *
 * Intended use: call this between wifi_manager_init() and ble_core_init()
 * so that BLE radio activity does not interfere with the AP bring-up
 * sequence on the shared antenna.
 */
void wifi_manager_wait_for_ap_ready(void);

#ifdef __cplusplus
}
#endif
