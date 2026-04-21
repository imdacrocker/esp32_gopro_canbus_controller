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
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "open_gopro_ble.h"
#include "camera_manager.h"
#include "ble_core.h"
#include "can_manager.h"
#include "legacy_gopro.h"

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
        ESP_LOGI(TAG, "Station connected — AID=%d  MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                 ev->aid,
                 ev->mac[0], ev->mac[1], ev->mac[2],
                 ev->mac[3], ev->mac[4], ev->mac[5]);
        /* IP is not available yet — wait for IP_EVENT_AP_STAIPASSIGNED before
         * probing.  The probe is triggered from ip_event_handler() below. */

    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "Station disconnected — AID=%d reason=%d  MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                 ev->aid, ev->reason,
                 ev->mac[0], ev->mac[1], ev->mac[2],
                 ev->mac[3], ev->mac[4], ev->mac[5]);
        legacy_gopro_on_station_disconnected(ev->mac);
    }
}

/* ============================================================
 * IP Event Handler
 * ============================================================ */

/**
 * Fired by lwIP / esp_netif when the DHCP server assigns an IP to a client.
 * This is the right moment to probe — the station now has a routable address.
 *
 * ip_event_ap_staipassigned_t fields:
 *   .ip.addr  — assigned IPv4 in network-byte-order uint32_t
 *   .mac[6]   — station MAC address
 */
static void ip_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    if (id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        ip_event_assigned_ip_to_client_t *ev = (ip_event_assigned_ip_to_client_t *)data;
        uint32_t ip = ev->ip.addr;

        ESP_LOGI(TAG, "DHCP assigned %d.%d.%d.%d to %02X:%02X:%02X:%02X:%02X:%02X",
                 (int)(ip & 0xFF), (int)((ip >> 8) & 0xFF),
                 (int)((ip >> 16) & 0xFF), (int)((ip >> 24) & 0xFF),
                 ev->mac[0], ev->mac[1], ev->mac[2],
                 ev->mac[3], ev->mac[4], ev->mac[5]);

        legacy_gopro_on_station_connected(ip, ev->mac);
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

/* POST /api/scan-cancel — stop a running discovery scan */
static esp_err_t api_scan_cancel_handler(httpd_req_t *req)
{
    open_gopro_ble_stop_discovery();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"cancelled\"}");
    return ESP_OK;
}

static const httpd_uri_t api_scan_cancel_uri = {
    .uri     = "/api/scan-cancel",
    .method  = HTTP_POST,
    .handler = api_scan_cancel_handler,
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
 *   [{"slot":N,"index":N,"name":"...","addr":"XX:XX:XX:XX:XX:XX","status":"<label>"},...]
 *
 * slot   — 0-based slot index used by API calls (e.g. /api/remove-camera)
 * index  — 1-based display number shown in the UI
 * status values:
 *   "disconnected"  — configured but no active BLE connection
 *   "not_recording" — connected, idle or recording state not yet known
 *   "recording"     — connected and actively recording
 */
static esp_err_t api_paired_cameras_handler(httpd_req_t *req)
{
    char buf[1536];
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

        const char *type_str = legacy_gopro_is_managed_slot(i) ? "legacy_wifi" : "ble";

        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "{\"slot\":%d,"
            "\"index\":%d,"
            "\"name\":\"%s\","
            "\"model_name\":\"%s\","
            "\"addr\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"status\":\"%s\","
            "\"type\":\"%s\"}",
            i,
            i + 1,
            info.name,
            info.model_name,
            v[5], v[4], v[3], v[2], v[1], v[0],
            status_str,
            type_str);
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

/* POST /api/remove-camera — remove a single camera slot
 *
 * Body: {"slot":N}  where N is the 0-based slot index from /api/paired-cameras.
 *
 * Sequence:
 *  1. Validate the slot and retrieve the camera's MAC address.
 *  2. Clear the slot from camera_manager (RAM + NVS) so that
 *     is_known_addr returns false before the BLE disconnect fires.
 *  3. Post an async request to the NimBLE task to terminate the active
 *     connection (if any) and remove just this camera's BLE bond.
 */
static esp_err_t api_remove_camera_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    char *p = strstr(body, "\"slot\":");
    if (!p) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing slot");
        return ESP_FAIL;
    }
    int slot = atoi(p + 7);
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid slot");
        return ESP_FAIL;
    }

    camera_slot_info_t info = camera_manager_get_slot_info(slot);
    if (!info.is_configured) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Slot not configured");
        return ESP_FAIL;
    }

    if (legacy_gopro_is_managed_slot(slot)) {
        /* Legacy Wi-Fi camera: delegate removal to legacy_gopro (async).
         * It will call camera_manager_remove_slot() internally. */
        legacy_gopro_remove_camera(info.mac_address.val);
    } else {
        /* BLE camera: clear camera_manager slot then remove NimBLE bond. */
        ble_addr_t mac = info.mac_address;
        camera_manager_remove_slot(slot);
        ble_core_remove_bond(&mac);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"removed\"}");
    return ESP_OK;
}

static const httpd_uri_t api_remove_camera_uri = {
    .uri     = "/api/remove-camera",
    .method  = HTTP_POST,
    .handler = api_remove_camera_handler,
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

/* GET /api/auto-control — return the current automatic_camera_control flag
 *
 * Returns: {"enabled":true} or {"enabled":false}
 */
static esp_err_t api_auto_control_get_handler(httpd_req_t *req)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "{\"enabled\":%s}",
             camera_manager_get_auto_control() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_auto_control_get_uri = {
    .uri     = "/api/auto-control",
    .method  = HTTP_GET,
    .handler = api_auto_control_get_handler,
};

/* POST /api/auto-control — body: {"enabled":true} or {"enabled":false}
 *
 * Sets the automatic_camera_control flag.  Does not change the current camera
 * recording state — cameras continue whatever they were doing.
 *
 * Returns: {"enabled":true} or {"enabled":false} reflecting the new state.
 */
static esp_err_t api_auto_control_post_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    char *p = strstr(body, "\"enabled\":");
    if (!p) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'enabled' field");
        return ESP_FAIL;
    }
    p += 10; /* skip past "enabled": */
    while (*p == ' ') p++;

    bool enabled;
    if (strncmp(p, "true", 4) == 0) {
        enabled = true;
    } else if (strncmp(p, "false", 5) == 0) {
        enabled = false;
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid 'enabled' value");
        return ESP_FAIL;
    }

    camera_manager_set_auto_control(enabled);

    char buf[24];
    snprintf(buf, sizeof(buf), "{\"enabled\":%s}", enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_auto_control_post_uri = {
    .uri     = "/api/auto-control",
    .method  = HTTP_POST,
    .handler = api_auto_control_post_handler,
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

    /* Optional "slot" field: if present, target only that camera.
     * If absent, apply to all cameras (original behaviour). */
    int dispatched;
    char *slot_p = strstr(body, "\"slot\":");
    if (slot_p) {
        int slot = atoi(slot_p + 7);
        dispatched = camera_manager_set_recording_slot(slot, shutter_on);
    } else {
        camera_manager_set_desired_recording(shutter_on);
        dispatched = shutter_on
                   ? camera_manager_start_recording_all()
                   : camera_manager_stop_recording_all();
    }

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

/* GET /api/legacy/discovered — AP stations that are connected but not yet
 * added to camera_manager by the user.
 *
 * Returns: [{"addr":"XX:XX:XX:XX:XX:XX"}, ...]
 *
 * The device making this request is automatically excluded from the list so
 * the phone/browser that opens the settings page never shows itself as a
 * candidate camera.  Devices that are already managed (paired) are also
 * excluded since they appear in /api/paired-cameras.
 */
static esp_err_t api_legacy_discovered_handler(httpd_req_t *req)
{
    /* Determine the requester's IP so we can exclude it from the list. */
    uint32_t requester_ip = 0;
    int sock = httpd_req_to_sockfd(req);
    if (sock >= 0) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        if (getpeername(sock, (struct sockaddr *)&peer, &peer_len) == 0) {
            requester_ip = peer.sin_addr.s_addr;
        }
    }

    legacy_discovered_camera_t all[4];
    int total = legacy_gopro_get_discovered(all, 4);

    char buf[512];
    int  pos   = 0;
    bool first = true;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "[");
    for (int i = 0; i < total; i++) {
        /* Skip the device that is loading the web page. */
        if (requester_ip && all[i].ip_addr == requester_ip) continue;

        const uint8_t *m = all[i].mac;
        if (!first) pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
        first = false;
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "{\"addr\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}",
            m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "]");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static const httpd_uri_t api_legacy_discovered_uri = {
    .uri     = "/api/legacy/discovered",
    .method  = HTTP_GET,
    .handler = api_legacy_discovered_handler,
};

/* POST /api/legacy/add — promote a discovered Hero4 to managed status.
 *
 * Body: {"addr":"XX:XX:XX:XX:XX:XX"}
 *
 * The operation is asynchronous — legacy_gopro posts a CMD_ADD_CAMERA to its
 * internal task queue.  The camera will appear in /api/paired-cameras within
 * ~3 seconds (settle loop).
 */
static esp_err_t api_legacy_add_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    char *p = strstr(body, "\"addr\":\"");
    if (!p) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing addr");
        return ESP_FAIL;
    }
    p += 8;

    unsigned int v[6];
    if (sscanf(p, "%02X:%02X:%02X:%02X:%02X:%02X",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad addr format");
        return ESP_FAIL;
    }

    uint8_t mac[6];
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];

    if (!legacy_gopro_add_camera(mac)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Queue full");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"adding\"}");
    return ESP_OK;
}

static const httpd_uri_t api_legacy_add_uri = {
    .uri     = "/api/legacy/add",
    .method  = HTTP_POST,
    .handler = api_legacy_add_handler,
};

/* POST /api/legacy/remove — un-manage a legacy camera.
 *
 * Body: {"addr":"XX:XX:XX:XX:XX:XX"}
 *
 * Delegates to legacy_gopro_remove_camera() which posts CMD_REMOVE_CAMERA.
 * The camera slot will be freed and removed from NVS.  If the camera is still
 * physically connected it will reappear in /api/legacy/discovered.
 */
static esp_err_t api_legacy_remove_handler(httpd_req_t *req)
{
    char body[64] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    char *p = strstr(body, "\"addr\":\"");
    if (!p) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing addr");
        return ESP_FAIL;
    }
    p += 8;

    unsigned int v[6];
    if (sscanf(p, "%02X:%02X:%02X:%02X:%02X:%02X",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad addr format");
        return ESP_FAIL;
    }

    uint8_t mac[6];
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];

    if (!legacy_gopro_remove_camera(mac)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Queue full");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"removed\"}");
    return ESP_OK;
}

static const httpd_uri_t api_legacy_remove_uri = {
    .uri     = "/api/legacy/remove",
    .method  = HTTP_POST,
    .handler = api_legacy_remove_handler,
};

/* ============================================================
 * HTTP Server
 * ============================================================ */

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 18;  /* 13 original + 3 legacy + 2 headroom */
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &api_status_uri);
        httpd_register_uri_handler(server, &api_scan_uri);
        httpd_register_uri_handler(server, &api_scan_cancel_uri);
        httpd_register_uri_handler(server, &api_cameras_uri);
        httpd_register_uri_handler(server, &api_pair_uri);
        httpd_register_uri_handler(server, &api_remove_camera_uri);
        httpd_register_uri_handler(server, &api_shutter_uri);
        httpd_register_uri_handler(server, &api_paired_cameras_uri);
        httpd_register_uri_handler(server, &api_logging_state_uri);
        httpd_register_uri_handler(server, &api_utc_uri);
        httpd_register_uri_handler(server, &api_auto_control_get_uri);
        httpd_register_uri_handler(server, &api_auto_control_post_uri);
        httpd_register_uri_handler(server, &api_legacy_discovered_uri);
        httpd_register_uri_handler(server, &api_legacy_add_uri);
        httpd_register_uri_handler(server, &api_legacy_remove_uri);
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

    /* Register for DHCP IP-assigned event so we know when a station has a
     * routable address and can be probed by legacy_gopro. */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_ASSIGNED_IP_TO_CLIENT,
                                                        &ip_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    /* Read the chip's factory MAC so we can preserve the last three bytes
     * (the device-unique portion) in our spoofed address.
     * esp_wifi_get_mac() is safe to call after esp_wifi_init() + set_mode(). */
    uint8_t mac[6] = {0};
    esp_err_t mac_err = esp_wifi_get_mac(WIFI_IF_AP, mac);

    /* Override the OUI to d8:96:85 so the ESP32 presents itself as a GoPro
     * RC Wi-Fi remote.  Hero4 (and other legacy GoPro) cameras auto-connect
     * to an AP whose SSID is "HERO-RC-XXXXXX" and whose source MAC starts
     * with this OUI.  The last three bytes are kept from the factory MAC so
     * the address stays unique across devices.
     *
     * NOTE: ESP-IDF v5.x validates that bit 1 of the first MAC octet (the
     * "locally administered" bit) is set before accepting a custom MAC.
     * 0xD8 (1101 1000) does NOT have that bit set, so esp_wifi_set_mac()
     * may return ESP_ERR_WIFI_MAC on newer IDF builds.  If that happens the
     * AP will start with the factory OUI and the Hero4 will not auto-connect;
     * the IDF validation will need to be patched or bypassed. */
    uint8_t gopro_mac[6] = {0xd8, 0x96, 0x85, mac[3], mac[4], mac[5]};
    esp_err_t set_mac_err = esp_wifi_set_mac(WIFI_IF_AP, gopro_mac);
    if (set_mac_err != ESP_OK) {
        ESP_LOGW(TAG, "Custom MAC (d8:96:85:%02X:%02X:%02X) rejected: %s — "
                      "Hero4 auto-connect will NOT work with factory OUI",
                 mac[3], mac[4], mac[5], esp_err_to_name(set_mac_err));
    } else {
        ESP_LOGI(TAG, "AP MAC overridden to d8:96:85:%02X:%02X:%02X",
                 mac[3], mac[4], mac[5]);
    }

    /* Build SSID from the same last-3-byte suffix used in the MAC above.
     * Validate — an all-zero suffix means the factory MAC read failed and we
     * fall back to a static name rather than broadcasting a garbage SSID. */
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
