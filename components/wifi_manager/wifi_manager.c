#include "wifi_manager.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "open_gopro_ble.h"
#include "camera_manager.h"
#include "ble_core.h"
#include "can_manager.h"

#define AP_CHANNEL          6   /* ch.6 is the 2.4 GHz center channel; avoids HT40+
                                 * regulatory issues that ch.1 has with iOS clients */
#define AP_MAX_CONN         4
#define AP_READY_TIMEOUT_MS 5000  /* Max ms to wait for WIFI_EVENT_AP_START */
#define AP_STARTED_BIT      BIT0

static const char *TAG = "WIFI_MGR";

/* Event group used to signal that the AP has successfully started.
 * wifi_manager_wait_for_ap_ready() blocks on this bit so that callers
 * (e.g. ble_core_init) do not start radio-intensive work until the AP
 * beacon is on air. */
static EventGroupHandle_t s_ap_event_group = NULL;

/* Symbols injected by the build system from www/index.html */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

/* ============================================================
 * WiFi Event Handler
 * ============================================================ */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_START) {
        /* AP is on air — safe to apply bandwidth config now.
         * Calling esp_wifi_set_bandwidth() before this event can disrupt
         * the AP bring-up sequence and leave it in a non-broadcasting state. */
        esp_err_t err = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_set_bandwidth failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "AP started — HT20 bandwidth applied");
        }
        /* Unblock wifi_manager_wait_for_ap_ready() */
        xEventGroupSetBits(s_ap_event_group, AP_STARTED_BIT);

    } else if (id == WIFI_EVENT_AP_STOP) {
        /* AP went down unexpectedly (coexistence scheduling, memory pressure, etc.).
         * Clear the ready bit and restart.  Without this, the AP simply stays
         * dark and the iPhone shows "network unavailable" indefinitely. */
        ESP_LOGW(TAG, "AP stopped unexpectedly — restarting");
        xEventGroupClearBits(s_ap_event_group, AP_STARTED_BIT);
        esp_wifi_start();

    } else if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "Station connected — AID=%d", ev->aid);

    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "Station disconnected — AID=%d reason=%d", ev->aid, ev->reason);
    }
}

/* ============================================================
 * HTTP Handlers
 * ============================================================ */

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri     = "/",
    .method  = HTTP_GET,
    .handler = root_handler,
};

/* POST /api/scan — start a 30-second discovery scan */
static esp_err_t api_scan_handler(httpd_req_t *req)
{
    open_gopro_ble_start_discovery();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"scanning\"}");
    return ESP_OK;
}

static const httpd_uri_t api_scan_uri = {
    .uri     = "/api/scan",
    .method  = HTTP_POST,
    .handler = api_scan_handler,
};

/* GET /api/cameras — return discovered cameras as a JSON array */
static esp_err_t api_cameras_handler(httpd_req_t *req)
{
    gopro_device_t devices[GOPRO_MAX_DISCOVERED];
    int count = open_gopro_ble_get_discovered(devices, GOPRO_MAX_DISCOVERED);

    /* Build JSON: [{"name":"...","addr":"XX:XX:XX:XX:XX:XX","addr_type":N,"rssi":N}, ...] */
    char buf[1024];
    int pos = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "[");
    for (int i = 0; i < count; i++) {
        const uint8_t *v = devices[i].addr.val;
        if (i > 0) pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "{\"name\":\"%s\","
            "\"addr\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"addr_type\":%d,"
            "\"rssi\":%d}",
            devices[i].name,
            v[5], v[4], v[3], v[2], v[1], v[0],
            devices[i].addr.type,
            devices[i].rssi);
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "]");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_cameras_uri = {
    .uri     = "/api/cameras",
    .method  = HTTP_GET,
    .handler = api_cameras_handler,
};

/* POST /api/pair — body: {"addr":"XX:XX:XX:XX:XX:XX","addr_type":N} */
static esp_err_t api_pair_handler(httpd_req_t *req)
{
    char body[128] = {0};
    int  received  = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    /* Parse MAC — find the "addr":"XX:XX:XX:XX:XX:XX" field */
    char *p = strstr(body, "\"addr\":\"");
    if (!p) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing addr");
        return ESP_FAIL;
    }
    p += 8; /* skip past "addr":" */

    unsigned int v[6];
    if (sscanf(p, "%02X:%02X:%02X:%02X:%02X:%02X",
               &v[5], &v[4], &v[3], &v[2], &v[1], &v[0]) != 6) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad addr format");
        return ESP_FAIL;
    }

    ble_addr_t addr;
    for (int i = 0; i < 6; i++) addr.val[i] = (uint8_t)v[i];

    char *t = strstr(body, "\"addr_type\":");
    addr.type = t ? (uint8_t)atoi(t + 12) : BLE_ADDR_PUBLIC;

    open_gopro_ble_connect_by_addr(&addr);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"pairing\"}");
    return ESP_OK;
}

static const httpd_uri_t api_pair_uri = {
    .uri     = "/api/pair",
    .method  = HTTP_POST,
    .handler = api_pair_handler,
};

/* GET /api/status — remembered and connected camera counts */
static esp_err_t api_status_handler(httpd_req_t *req)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"remembered\":%d,\"connected\":%d}",
             camera_manager_remembered_count(),
             camera_manager_connected_count());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_status_uri = {
    .uri     = "/api/status",
    .method  = HTTP_GET,
    .handler = api_status_handler,
};

/* GET /api/paired-cameras — all remembered cameras with live status
 *
 * Returns a JSON array of every configured slot:
 *   [{"index":N,"addr":"XX:XX:XX:XX:XX:XX","status":"<label>"},...]
 *
 * status values:
 *   "disconnected"  — configured but no active BLE connection
 *   "not_recording" — connected, idle or recording state not yet known
 *   "recording"     — connected and actively recording
 */
static esp_err_t api_paired_cameras_handler(httpd_req_t *req)
{
    char buf[512];
    int  pos   = 0;
    bool first = true;

    pos += snprintf(buf + pos, sizeof(buf) - pos, "[");

    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        camera_slot_info_t info = camera_manager_get_slot_info(i);
        if (!info.is_configured) continue;

        const uint8_t *v = info.mac_address.val;
        const char    *status_str;

        switch (info.status) {
            case CAMERA_STATUS_RECORDING:    status_str = "recording";     break;
            case CAMERA_STATUS_CONNECTED:    status_str = "not_recording"; break;
            case CAMERA_STATUS_DISCONNECTED: status_str = "disconnected";  break;
            default:                         status_str = "unknown";        break;
        }

        if (!first) pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
        first = false;

        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "{\"index\":%d,"
            "\"addr\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"status\":\"%s\"}",
            i,
            v[5], v[4], v[3], v[2], v[1], v[0],
            status_str);
    }

    pos += snprintf(buf + pos, sizeof(buf) - pos, "]");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_paired_cameras_uri = {
    .uri     = "/api/paired-cameras",
    .method  = HTTP_GET,
    .handler = api_paired_cameras_handler,
};

/* POST /api/reset-bonds — delete all stored BLE bonds and camera slots */
static esp_err_t api_reset_bonds_handler(httpd_req_t *req)
{
    /* Remove every camera slot from camera_manager (RAM + NVS).
     * Without this, the Camera Status panel still shows cameras as paired
     * after the reset because /api/paired-cameras reads from camera_manager,
     * not from NimBLE's bond store. */
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        camera_manager_remove_slot(i);
    }

    /* Purge BLE bonds from NimBLE's peer-security store (NVS).
     * This is posted asynchronously to the NimBLE event queue. */
    ble_core_purge_unknown_bonds(NULL, 0);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"bonds cleared\"}");
    return ESP_OK;
}

static const httpd_uri_t api_reset_bonds_uri = {
    .uri     = "/api/reset-bonds",
    .method  = HTTP_POST,
    .handler = api_reset_bonds_handler,
};

/* GET /api/logging-state — current RaceCapture logging state
 *
 * Returns: {"state":"logging"} | {"state":"not_logging"} | {"state":"unknown"}
 *
 * "unknown" means no 0x600 frame has been received from the RaceCapture
 * within the last 5 seconds (see CAN_MANAGER_LOGGING_TIMEOUT_MS).
 */
static esp_err_t api_logging_state_handler(httpd_req_t *req)
{
    const char *state_str;
    switch (can_manager_get_logging_state()) {
        case LOGGING_STATE_LOGGING:     state_str = "logging";     break;
        case LOGGING_STATE_NOT_LOGGING: state_str = "not_logging"; break;
        default:                        state_str = "unknown";     break;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "{\"state\":\"%s\"}", state_str);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_logging_state_uri = {
    .uri     = "/api/logging-state",
    .method  = HTTP_GET,
    .handler = api_logging_state_handler,
};

/* GET /api/utc — current UTC time derived from the RaceCapture GPS clock
 *
 * Returns: {"valid":true,"epoch_ms":NNNN} once GPS lock has been acquired,
 *          {"valid":false}                if no valid UTC frame received yet.
 *
 * The ESP32 extrapolates the current time between 0x602 frames using its
 * monotonic timer (see can_manager_get_utc_ms), so sub-second accuracy is
 * maintained without waiting for the next CAN frame.
 */
static esp_err_t api_utc_handler(httpd_req_t *req)
{
    uint64_t epoch_ms = 0;
    bool valid = can_manager_get_utc_ms(&epoch_ms);

    char buf[64];
    if (valid) {
        snprintf(buf, sizeof(buf), "{\"valid\":true,\"epoch_ms\":%llu}", epoch_ms);
    } else {
        snprintf(buf, sizeof(buf), "{\"valid\":false}");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_utc_uri = {
    .uri     = "/api/utc",
    .method  = HTTP_GET,
    .handler = api_utc_handler,
};

/* POST /api/shutter — body: {"on":true} or {"on":false}
 *
 * Sends a start/stop recording command to every connected, GATT-ready camera.
 * Responds with {"dispatched": N} where N is the number of cameras reached.
 */
static esp_err_t api_shutter_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    /* Simple JSON parse — look for "on":true or "on":false */
    char *p = strstr(body, "\"on\":");
    if (!p) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'on' field");
        return ESP_FAIL;
    }
    p += 5; /* skip past "on": */
    while (*p == ' ') p++;

    bool shutter_on;
    if (strncmp(p, "true", 4) == 0) {
        shutter_on = true;
    } else if (strncmp(p, "false", 5) == 0) {
        shutter_on = false;
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid 'on' value");
        return ESP_FAIL;
    }

    camera_manager_set_desired_recording(shutter_on);

    int dispatched = shutter_on
                   ? camera_manager_start_recording_all()
                   : camera_manager_stop_recording_all();

    char resp[48];
    snprintf(resp, sizeof(resp), "{\"dispatched\":%d}", dispatched);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static const httpd_uri_t api_shutter_uri = {
    .uri     = "/api/shutter",
    .method  = HTTP_POST,
    .handler = api_shutter_handler,
};

/* ============================================================
 * HTTP Server
 * ============================================================ */

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;  /* default is 8; bump to fit current 9 + headroom */
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &api_status_uri);
        httpd_register_uri_handler(server, &api_scan_uri);
        httpd_register_uri_handler(server, &api_cameras_uri);
        httpd_register_uri_handler(server, &api_pair_uri);
        httpd_register_uri_handler(server, &api_reset_bonds_uri);
        httpd_register_uri_handler(server, &api_shutter_uri);
        httpd_register_uri_handler(server, &api_paired_cameras_uri);
        httpd_register_uri_handler(server, &api_logging_state_uri);
        httpd_register_uri_handler(server, &api_utc_uri);
        ESP_LOGI(TAG, "HTTP server started");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

void wifi_manager_init(void)
{
    s_ap_event_group = xEventGroupCreate();
    configASSERT(s_ap_event_group);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    /* Set a custom IP — must be done before esp_wifi_start() */
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    IP4_ADDR((ip4_addr_t *)&ip_info.ip,      10,  71,  79,   1);
    IP4_ADDR((ip4_addr_t *)&ip_info.gw,      10,  71,  79,   1);
    IP4_ADDR((ip4_addr_t *)&ip_info.netmask, 255, 255, 255,  0);
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handler BEFORE esp_wifi_start() so no events are missed.
     * esp_wifi_set_bandwidth() is intentionally NOT called here — it is called
     * from the WIFI_EVENT_AP_START handler.  Calling it before AP_START can
     * silently disrupt the bring-up sequence and leave the AP dark. */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    /* Build SSID from last 3 bytes of AP MAC address.
     * esp_wifi_get_mac() is safe to call after esp_wifi_init() + set_mode().
     * Validate the result — an all-zero suffix means the read failed and we
     * fall back to a static name rather than broadcasting a garbage SSID. */
    uint8_t mac[6] = {0};
    esp_err_t mac_err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    char ap_ssid[16];
    if (mac_err != ESP_OK || (mac[3] == 0 && mac[4] == 0 && mac[5] == 0)) {
        ESP_LOGW(TAG, "MAC read failed (%s) — using fallback SSID",
                 esp_err_to_name(mac_err));
        snprintf(ap_ssid, sizeof(ap_ssid), "HERO-RC-000000");
    } else {
        snprintf(ap_ssid, sizeof(ap_ssid), "HERO-RC-%02X%02X%02X",
                 mac[3], mac[4], mac[5]);
    }

    wifi_config_t wifi_config = {
        .ap = {
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_OPEN,
            /* Explicitly disable PMF on the open AP.  With WPA3/SAE compiled
             * into the firmware, capability bits can bleed into beacon frames
             * and cause iOS to attempt (and fail) a WPA3/OWE handshake. */
            .pmf_cfg        = { .required = false },
        },
    };
    memcpy(wifi_config.ap.ssid, ap_ssid, strlen(ap_ssid));
    wifi_config.ap.ssid_len = strlen(ap_ssid);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    /* Start the AP.  The event handler will fire WIFI_EVENT_AP_START once the
     * beacon is on air, apply HT20 bandwidth, and set AP_STARTED_BIT. */
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP starting — SSID: %s  IP: 10.71.79.1", ap_ssid);

    /* Start the HTTP server immediately — it will be reachable once the iPhone
     * connects after AP_STARTED_BIT is set. */
    start_http_server();
}

void wifi_manager_wait_for_ap_ready(void)
{
    if (s_ap_event_group == NULL) {
        ESP_LOGE(TAG, "wifi_manager_wait_for_ap_ready called before wifi_manager_init");
        return;
    }

    ESP_LOGI(TAG, "Waiting for AP to be ready...");
    EventBits_t bits = xEventGroupWaitBits(s_ap_event_group,
                                           AP_STARTED_BIT,
                                           pdFALSE,                      /* don't clear on exit */
                                           pdTRUE,                       /* wait for all bits */
                                           pdMS_TO_TICKS(AP_READY_TIMEOUT_MS));
    if (bits & AP_STARTED_BIT) {
        ESP_LOGI(TAG, "AP ready — proceeding with BLE init");
    } else {
        ESP_LOGE(TAG, "Timed out waiting for AP_START after %d ms — proceeding anyway",
                 AP_READY_TIMEOUT_MS);
    }
}
