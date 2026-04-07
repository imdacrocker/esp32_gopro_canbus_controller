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
#include "ble_scanner.h"

#define AP_CHANNEL   1
#define AP_MAX_CONN  4

static const char *TAG = "WIFI_MGR";

/* Symbols injected by the build system from www/index.html */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

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

/* POST /api/scan — start a 10-second discovery scan */
static esp_err_t api_scan_handler(httpd_req_t *req)
{
    ble_scanner_start_discovery();
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
    int count = ble_scanner_get_discovered(devices, GOPRO_MAX_DISCOVERED);

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

    ble_scanner_connect_by_addr(&addr);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"pairing\"}");
    return ESP_OK;
}

static const httpd_uri_t api_pair_uri = {
    .uri     = "/api/pair",
    .method  = HTTP_POST,
    .handler = api_pair_handler,
};

/* POST /api/reset-bonds — delete all stored BLE bonds */
static esp_err_t api_reset_bonds_handler(httpd_req_t *req)
{
    ble_scanner_purge_unknown_bonds(NULL, 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"bonds cleared\"}");
    return ESP_OK;
}

static const httpd_uri_t api_reset_bonds_uri = {
    .uri     = "/api/reset-bonds",
    .method  = HTTP_POST,
    .handler = api_reset_bonds_handler,
};

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &api_scan_uri);
        httpd_register_uri_handler(server, &api_cameras_uri);
        httpd_register_uri_handler(server, &api_pair_uri);
        httpd_register_uri_handler(server, &api_reset_bonds_uri);
        ESP_LOGI(TAG, "HTTP server started");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

void wifi_manager_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    /* Set a custom IP — must be done before esp_wifi_start() */
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    IP4_ADDR((ip4_addr_t *)&ip_info.ip,      10,  71,  79,   1);
    IP4_ADDR((ip4_addr_t *)&ip_info.gw,      10,  71,  79,   1);
    IP4_ADDR((ip4_addr_t *)&ip_info.netmask, 255, 255, 255,  0);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_wifi_set_mode(WIFI_MODE_AP);

    /* Build SSID from last 3 bytes of AP MAC address */
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    char ap_ssid[16];
    snprintf(ap_ssid, sizeof(ap_ssid), "HERO-RC-%02X%02X%02X",
             mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_OPEN,
        },
    };
    memcpy(wifi_config.ap.ssid, ap_ssid, strlen(ap_ssid));
    wifi_config.ap.ssid_len = strlen(ap_ssid);

    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi AP started — SSID: %s  IP: 10.71.79.1", ap_ssid);

    start_http_server();
}
