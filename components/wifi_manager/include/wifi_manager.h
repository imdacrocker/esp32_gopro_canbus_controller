/**
 * @file wifi_manager.h
 * @brief Wi-Fi Access Point and HTTP management server.
 *
 * wifi_manager starts the ESP32 in soft-AP mode and hosts a single-page web
 * application that allows a connected phone or laptop to:
 *  - Scan for nearby GoPro cameras (BLE) and pair them.
 *  - Discover legacy Wi-Fi cameras (Hero4) connected to the AP and add them.
 *  - View live camera status for all connected cameras.
 *  - Manually start / stop recording on all cameras or individual slots.
 *  - Enable or disable automatic CAN-driven recording control.
 *  - Perform a factory reset (erase NVS, reboot).
 *
 * Legacy camera (Hero4) management:
 *  - /api/legacy/discovered — lists unmanaged stations with DHCP addresses.
 *  - /api/legacy/add — probes and promotes a station to managed.
 *  - /api/legacy/remove — unregisters a managed camera.
 *
 * Station event routing to legacy_gopro:
 *  - WIFI_EVENT_AP_STACONNECTED → legacy_gopro_on_station_wifi_associated()
 *  - IP_EVENT_ASSIGNED_IP_TO_CLIENT → legacy_gopro_on_station_connected()
 *  - WIFI_EVENT_AP_STADISCONNECTED → legacy_gopro_on_station_disconnected()
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

#include <stdint.h>

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

/**
 * @brief Per-station record returned by wifi_manager_get_connected_stations().
 */
typedef struct {
    uint8_t  mac[6];
    uint32_t ip_addr;   /**< DHCP-assigned IP (network-byte-order); 0 if not yet assigned */
} wifi_mgr_sta_info_t;

/**
 * @brief Return a snapshot of all stations currently associated with the SoftAP.
 *
 * Includes stations that have completed L2 association but have not yet
 * received a DHCP address (ip_addr == 0).  The Hero4 in RC-remote mode
 * never performs a DHCP request and will always appear with ip_addr == 0.
 *
 * The snapshot is read lock-free from the internal station table; minor
 * races are acceptable for display purposes.
 *
 * @param out        Caller-provided array (must hold at least @p max_count entries).
 * @param max_count  Maximum entries to write.
 * @return           Number of associated stations written to @p out.
 */
int wifi_manager_get_connected_stations(wifi_mgr_sta_info_t *out, int max_count);


/**
 * @brief Return the DHCP-assigned IP for a single station by MAC address.
 *
 * @param mac  6-byte MAC address of the station to look up.
 * @return     IP address in network byte order, or 0 if the station is not
 *             associated or has not yet been assigned an address by DHCP.
 */
uint32_t wifi_manager_get_station_ip(const uint8_t mac[6]);

#ifdef __cplusplus
}
#endif
