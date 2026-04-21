/**
 * @file legacy_gopro.c
 * @brief Legacy GoPro (Hero4) Wi-Fi control component.
 *
 * Architecture
 * ------------
 * A single FreeRTOS task owns all blocking operations (HTTP probe, command
 * handling, keepalive settle loops).  Callers post lightweight commands to a
 * queue; the task services them in order and runs periodic keepalive + status
 * polls on a 2-second timeout.
 *
 * Managed vs Discovered cameras
 * --------------------------------
 * On DHCP assignment, legacy_gopro probes the station via HTTP to determine
 * if it is a Hero4.  Probed cameras are "discovered" but NOT registered with
 * camera_manager until the user explicitly adds them via /api/legacy/add.
 *
 * Once added, a camera is "managed": it is registered with camera_manager,
 * receives keepalives every 2 s, is polled for recording status, and responds
 * to shutter commands.  Its MAC is saved to NVS so that subsequent reconnects
 * auto-promote it to managed without user action.
 *
 * Shutter commands bypass the queue.  A single UDP broadcast to
 * 255.255.255.255:8484 reaches all managed cameras simultaneously, providing
 * the best possible recording-start synchronisation.
 *
 * Hero4 UDP RC-remote protocol
 * ----------------------------
 *   Keepalive:   "_GPHD_:0:0:2:0.000000\n"  (ASCII, 22 bytes, unicast per camera)
 *   Shutter ON:  00 00 00 00 00 00 00 00 00 01 00 53 48 02  (14 bytes, broadcast)
 *   Shutter OFF: 00 00 00 00 00 00 00 00 00 01 00 53 48 00  (14 bytes, broadcast)
 *   Status req:  00 00 00 00 00 00 00 00 00 00 00 73 74      (13 bytes, unicast)
 */

#include "legacy_gopro.h"
#include "camera_manager.h"
#include "camera_driver.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "LEGACY_GOPRO";

/* ============================================================
 * Constants
 * ============================================================ */

#define LEGACY_MAX_CAMERAS      4       /* max simultaneous stations tracked */
#define PROBE_ON_ADD_TIMEOUT_MS 5000    /* HTTP timeout for on-demand probe (add flow) */
#define PROBE_ON_ADD_ATTEMPTS   2       /* max attempts when user clicks Add */
#define POLL_INTERVAL_MS        2000    /* keepalive + status poll cadence */
#define HTTP_BUF_SIZE           2560
#define TASK_STACK_SIZE         8192
#define QUEUE_DEPTH             8
#define SETTLE_DURATION_MS      2000    /* keepalive settle before on_wifi_connected */
#define SETTLE_STEP_MS          500

/* Hero4 UDP RC-remote protocol */
#define GOPRO_UDP_CMD_PORT      8484
#define GOPRO_UDP_LOCAL_PORT    8383
#define GOPRO_UDP_CMD_LEN       14
#define GOPRO_KEEPALIVE_STR     "_GPHD_:0:0:2:0.000000\n"
#define GOPRO_KEEPALIVE_LEN     22

/* NVS storage for managed camera MACs */
#define NVS_NAMESPACE           "lgcy_gopro"
#define NVS_KEY_MANAGED_MACS    "managed_macs"

/* ============================================================
 * Hero4 UDP command payloads
 * ============================================================ */

/* Byte[9] must be 0x01 (confirmed from working RaceCapture Lua implementation) */
static const uint8_t UDP_SHUTTER_ON[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x53, 0x48, 0x02    /* 'S','H', param=2 (start) */
};
static const uint8_t UDP_SHUTTER_OFF[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x53, 0x48, 0x00    /* 'S','H', param=0 (stop) */
};
static const uint8_t UDP_STATUS_REQ[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x73, 0x74           /* 's','t' */
};

/* ============================================================
 * Internal types
 * ============================================================ */

typedef enum {
    CMD_STATION_CONNECT, /* new station got a DHCP IP: record MAC+IP, no probe */
    CMD_DISCONNECT,      /* station left the AP */
    CMD_ADD_CAMERA,      /* user clicked Add: probe then promote if Hero4 */
    CMD_REMOVE_CAMERA,   /* un-manage a camera */
} legacy_cmd_type_t;

typedef struct {
    legacy_cmd_type_t type;
    uint32_t          ip;
    uint8_t           mac[6];
} legacy_cmd_t;

/**
 * Per-camera runtime state.  The driver_ctx pointer stored in camera_manager
 * points directly into this array — entries must not be moved after registration.
 */
typedef struct {
    bool     active;          /**< true while station is connected to the SoftAP */
    bool     managed;         /**< true after user explicitly adds this camera */
    bool     wifi_connected;  /**< true while camera is on the AP and registered */
    uint32_t ip_addr;         /**< current IP, network byte order */
    uint8_t  mac[6];
    int      slot;            /**< camera_manager slot; -1 until managed */
    char     name[LEGACY_CAMERA_NAME_LEN]; /**< name from probe response */
    camera_recording_status_t recording_status;
} legacy_camera_t;

/* ============================================================
 * Module state
 * ============================================================ */

static QueueHandle_t   s_queue   = NULL;
static TaskHandle_t    s_task    = NULL;
static TaskHandle_t    s_rx_task = NULL;
static legacy_camera_t s_cameras[LEGACY_MAX_CAMERAS];

/* Last shutter state broadcast.  Suppresses duplicate broadcasts when
 * camera_manager calls drv_start/stop_recording on multiple legacy slots. */
static bool            s_shutter_active = false;

/* UDP socket bound to GOPRO_UDP_LOCAL_PORT (8383) — opened once in init. */
static int             s_udp_sock = -1;

/* NVS-persisted managed MACs — loaded at init, updated on add/remove. */
static uint8_t         s_saved_macs[LEGACY_MAX_CAMERAS][6];
static int             s_saved_mac_count = 0;

/* HTTP response buffer — only accessed from the task context. */
static char s_http_buf[HTTP_BUF_SIZE];
static int  s_http_buf_len;

/* ============================================================
 * Driver vtable — forward declarations
 * ============================================================ */

static esp_err_t                 drv_start_recording(void *ctx);
static esp_err_t                 drv_stop_recording(void *ctx);
static camera_recording_status_t drv_get_recording_status(void *ctx);

static const camera_driver_t s_legacy_driver = {
    .start_recording      = drv_start_recording,
    .stop_recording       = drv_stop_recording,
    .get_recording_status = drv_get_recording_status,
};

/* ============================================================
 * Internal helpers
 * ============================================================ */

static void ip_to_str(uint32_t ip, char *buf, size_t buf_len)
{
    snprintf(buf, buf_len, "%d.%d.%d.%d",
             (int)(ip         & 0xFF),
             (int)((ip >>  8) & 0xFF),
             (int)((ip >> 16) & 0xFF),
             (int)((ip >> 24) & 0xFF));
}

static int find_camera_by_mac(const uint8_t mac[6])
{
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        /* Check active OR managed: managed entries pre-registered at init have
         * active=false until the camera physically connects. */
        if ((s_cameras[i].active || s_cameras[i].managed) &&
            memcmp(s_cameras[i].mac, mac, 6) == 0) {
            return i;
        }
    }
    return -1;
}

static int find_free_camera_entry(void)
{
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        /* Skip managed entries: they are pre-registered at init and must not
         * be overwritten by a new station that happens to land in the same slot. */
        if (!s_cameras[i].active && !s_cameras[i].managed) return i;
    }
    return -1;
}

static int find_camera_by_ip(uint32_t ip)
{
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (s_cameras[i].active && s_cameras[i].ip_addr == ip) {
            return i;
        }
    }
    return -1;
}

/* ============================================================
 * NVS helpers
 * ============================================================ */

/** Return true if mac is in the saved managed-MAC list. */
static bool nvs_mac_is_saved(const uint8_t mac[6])
{
    for (int i = 0; i < s_saved_mac_count; i++) {
        if (memcmp(s_saved_macs[i], mac, 6) == 0) return true;
    }
    return false;
}

/** Write the current s_saved_macs[] array to NVS. */
static void nvs_persist(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }
    if (s_saved_mac_count == 0) {
        nvs_erase_key(h, NVS_KEY_MANAGED_MACS);
    } else {
        err = nvs_set_blob(h, NVS_KEY_MANAGED_MACS,
                           s_saved_macs, (size_t)(s_saved_mac_count * 6));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS set_blob failed: %s", esp_err_to_name(err));
        }
    }
    nvs_commit(h);
    nvs_close(h);
}

/** Load managed MACs from NVS into s_saved_macs[]. */
static void nvs_load_managed_macs(void)
{
    s_saved_mac_count = 0;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open (read) failed: %s", esp_err_to_name(err));
        return;
    }

    size_t blob_len = sizeof(s_saved_macs);
    err = nvs_get_blob(h, NVS_KEY_MANAGED_MACS, s_saved_macs, &blob_len);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) return;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS get_blob failed: %s", esp_err_to_name(err));
        return;
    }

    s_saved_mac_count = (int)(blob_len / 6);
    ESP_LOGI(TAG, "Loaded %d managed MAC(s) from NVS", s_saved_mac_count);
    for (int i = 0; i < s_saved_mac_count; i++) {
        const uint8_t *m = s_saved_macs[i];
        ESP_LOGI(TAG, "  [%d] %02X:%02X:%02X:%02X:%02X:%02X",
                 i, m[0], m[1], m[2], m[3], m[4], m[5]);
    }
}

/** Add a MAC to the saved list (no-op if already present) and persist. */
static void nvs_add_managed_mac(const uint8_t mac[6])
{
    if (nvs_mac_is_saved(mac)) return;
    if (s_saved_mac_count >= LEGACY_MAX_CAMERAS) {
        ESP_LOGW(TAG, "NVS managed MAC list full — cannot save");
        return;
    }
    memcpy(s_saved_macs[s_saved_mac_count++], mac, 6);
    nvs_persist();
}

/** Remove a MAC from the saved list and persist. */
static void nvs_remove_managed_mac(const uint8_t mac[6])
{
    for (int i = 0; i < s_saved_mac_count; i++) {
        if (memcmp(s_saved_macs[i], mac, 6) == 0) {
            memmove(s_saved_macs[i], s_saved_macs[i + 1],
                    (size_t)((s_saved_mac_count - i - 1) * 6));
            s_saved_mac_count--;
            nvs_persist();
            return;
        }
    }
}

/* ============================================================
 * UDP socket
 * ============================================================ */

static void udp_socket_init(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "UDP socket create failed (errno %d) — shutter disabled", errno);
        return;
    }

    int broadcast_en = 1;
    if (setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST,
                   &broadcast_en, sizeof(broadcast_en)) < 0) {
        ESP_LOGW(TAG, "SO_BROADCAST failed (errno %d)", errno);
    }

    struct sockaddr_in local = {
        .sin_family      = AF_INET,
        .sin_port        = htons(GOPRO_UDP_LOCAL_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_udp_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGW(TAG, "UDP bind to port %d failed (errno %d) — using ephemeral port",
                 GOPRO_UDP_LOCAL_PORT, errno);
    } else {
        ESP_LOGI(TAG, "UDP socket bound to port %d", GOPRO_UDP_LOCAL_PORT);
    }

    struct timeval rx_timeout = { .tv_sec = 0, .tv_usec = 500 * 1000 };
    setsockopt(s_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));
}

static void do_udp_send(const void *payload, size_t len,
                        uint32_t dest_ip, uint16_t dest_port,
                        const char *label)
{
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "UDP socket not initialised — cannot send %s", label);
        return;
    }

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(dest_port),
        .sin_addr.s_addr = dest_ip,
    };

    int sent = sendto(s_udp_sock, payload, len, 0,
                      (struct sockaddr *)&dest, sizeof(dest));
    if (sent < 0) {
        ESP_LOGE(TAG, "UDP %s → port %d failed (errno %d)", label, dest_port, errno);
    } else {
        char ip_str[16];
        ip_to_str(dest_ip, ip_str, sizeof(ip_str));
        ESP_LOGD(TAG, "UDP %s → %s:%d (%d bytes)", label, ip_str, dest_port, sent);
    }
}

/**
 * Broadcast a shutter command to all cameras on the AP subnet.
 * A single packet reaches every Hero4 simultaneously.
 */
static void do_udp_shutter(bool on)
{
    const uint8_t *payload = on ? UDP_SHUTTER_ON : UDP_SHUTTER_OFF;
    do_udp_send(payload, GOPRO_UDP_CMD_LEN,
                htonl(INADDR_BROADCAST), GOPRO_UDP_CMD_PORT,
                on ? "shutter-ON(bcast)" : "shutter-OFF(bcast)");
}

/**
 * Send keepalive to every managed+connected camera, plus an optional extra IP.
 *
 * extra_ip is used during probe and settle to send to a camera that is not
 * yet wifi_connected=true in the table.  Pass 0 to skip.
 *
 * Must only be called from the legacy_gopro task (reads s_cameras[]).
 */
static void do_udp_keepalive(uint32_t extra_ip)
{
    if (s_udp_sock < 0) return;
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (!s_cameras[i].active)         continue;
        if (!s_cameras[i].managed)        continue;  /* only managed cameras */
        if (!s_cameras[i].wifi_connected) continue;
        do_udp_send(GOPRO_KEEPALIVE_STR, GOPRO_KEEPALIVE_LEN,
                    s_cameras[i].ip_addr, GOPRO_UDP_CMD_PORT, "keepalive");
    }
    if (extra_ip) {
        do_udp_send(GOPRO_KEEPALIVE_STR, GOPRO_KEEPALIVE_LEN,
                    extra_ip, GOPRO_UDP_CMD_PORT, "keepalive(extra)");
    }
}

/* ============================================================
 * HTTP client
 * ============================================================ */

static int do_http_get(const char *url, int timeout_ms)
{
    s_http_buf_len = 0;
    memset(s_http_buf, 0, HTTP_BUF_SIZE);

    esp_http_client_config_t cfg = {
        .url        = url,
        .timeout_ms = timeout_ms,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "HTTP client init failed");
        return -1;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HTTP open %s → %s (0x%x)",
                 url, esp_err_to_name(err), (unsigned)err);
        esp_http_client_cleanup(client);
        return -1;
    }

    (void)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);

    if (status != 200) {
        ESP_LOGW(TAG, "Non-200 response (%d) — not a Hero4 gpControl endpoint", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return -1;
    }

    int n;
    while (s_http_buf_len < HTTP_BUF_SIZE - 1) {
        n = esp_http_client_read(client,
                                 s_http_buf + s_http_buf_len,
                                 HTTP_BUF_SIZE - 1 - s_http_buf_len);
        if (n <= 0) break;
        s_http_buf_len += n;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    s_http_buf[s_http_buf_len] = '\0';

    if (s_http_buf_len == 0) {
        ESP_LOGW(TAG, "HTTP 200 but empty body");
        return 0;
    }

    ESP_LOGD(TAG, "Body (%d bytes): %.120s", s_http_buf_len, s_http_buf);
    return s_http_buf_len;
}

/* ============================================================
 * JSON parsing
 * ============================================================ */

static bool json_get_int(const char *haystack, const char *end,
                         const char *key, int *out_val)
{
    char pat[16];
    snprintf(pat, sizeof(pat), "\"%s\":", key);

    const char *p = haystack;
    while ((p = strstr(p, pat)) != NULL) {
        if (end && p >= end) break;
        p += strlen(pat);
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '-' || (*p >= '0' && *p <= '9')) {
            *out_val = (int)strtol(p, NULL, 10);
            return true;
        }
    }
    return false;
}

static bool json_get_str(const char *haystack, const char *end,
                         const char *key, char *out, int out_len)
{
    char pat[16];
    snprintf(pat, sizeof(pat), "\"%s\":", key);

    const char *p = haystack;
    while ((p = strstr(p, pat)) != NULL) {
        if (end && p >= end) break;
        p += strlen(pat);
        while (*p == ' ' || *p == '\t') p++;
        if (*p != '"') continue;
        p++;
        int i = 0;
        while (*p && *p != '"' && i < out_len - 1) {
            out[i++] = *p++;
        }
        out[i] = '\0';
        return true;
    }
    return false;
}

static camera_recording_status_t parse_status_json(const char *json,
                                                    char *name_out, int name_len)
{
    camera_recording_status_t result = CAMERA_RECORDING_UNKNOWN;

    const char *status_key = strstr(json, "\"status\":");
    if (!status_key) {
        ESP_LOGW(TAG, "No 'status' key in response — not a Hero4 gpControl response");
        return result;
    }
    const char *status_open = strchr(status_key, '{');
    if (!status_open) return result;

    int depth = 0;
    const char *status_close = status_open;
    while (*status_close) {
        if (*status_close == '{') depth++;
        else if (*status_close == '}') { if (--depth == 0) break; }
        status_close++;
    }

    int rec_val = 0;
    if (json_get_int(status_open, status_close, "8", &rec_val)) {
        result = (rec_val == 1) ? CAMERA_RECORDING_ACTIVE : CAMERA_RECORDING_IDLE;
    }

    if (name_out && name_len > 0) {
        json_get_str(status_open, status_close, "30", name_out, name_len);
    }

    return result;
}

/* ============================================================
 * Managed camera promotion — runs inside the task
 * ============================================================ */

/**
 * Register cam_idx with camera_manager, run the keepalive settle loop, then
 * call camera_manager_on_wifi_connected().
 *
 * Sets cam->slot and cam->wifi_connected on success.
 * Resets cam->managed on registration failure.
 *
 * Must only be called from the legacy_gopro task.
 */
static void promote_to_managed_settled(int cam_idx)
{
    legacy_camera_t *cam = &s_cameras[cam_idx];
    char ip_str[16];
    ip_to_str(cam->ip_addr, ip_str, sizeof(ip_str));

    /* Register with camera_manager only if not already slotted.
     * Cameras pre-registered at init already have a valid slot. */
    if (cam->slot < 0) {
        int slot = camera_manager_register_wifi_camera(
                       cam->mac, cam->name,
                       &s_legacy_driver, cam);
        if (slot < 0) {
            ESP_LOGE(TAG, "camera_manager_register_wifi_camera failed for %s", ip_str);
            cam->managed = false;
            return;
        }
        cam->slot = slot;
    }

    ESP_LOGI(TAG, "Hero4 '%s' at %s → slot %d — settling pairing...",
             cam->name, ip_str, cam->slot);

    /* Send keepalives for SETTLE_DURATION_MS before marking the camera ready.
     * The Hero4 must ACK several keepalives before it accepts shutter commands.
     * extra_ip sends to this camera even though wifi_connected is still false. */
    for (int elapsed = 0; elapsed < SETTLE_DURATION_MS; elapsed += SETTLE_STEP_MS) {
        do_udp_keepalive(cam->ip_addr);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_STEP_MS));
    }

    cam->wifi_connected = true;
    camera_manager_on_wifi_connected(cam->slot, cam->ip_addr);
    ESP_LOGI(TAG, "Hero4 '%s' at %s slot %d ready", cam->name, ip_str, cam->slot);
}

/* ============================================================
 * Command handlers — all run inside the task
 * ============================================================ */

static void handle_station_connect(const legacy_cmd_t *cmd)
{
    char ip_str[16];
    ip_to_str(cmd->ip, ip_str, sizeof(ip_str));

    /* Find existing entry (managed camera reconnect) or allocate a new slot. */
    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx < 0) {
        cam_idx = find_free_camera_entry();
        if (cam_idx < 0) {
            ESP_LOGW(TAG, "No free camera entries for %s — ignoring", ip_str);
            return;
        }
        s_cameras[cam_idx].slot    = -1;
        s_cameras[cam_idx].managed = false;
        s_cameras[cam_idx].name[0] = '\0';
    }

    legacy_camera_t *cam = &s_cameras[cam_idx];
    cam->active           = true;
    cam->wifi_connected   = false;   /* set true inside promote_to_managed_settled */
    cam->ip_addr          = cmd->ip;
    cam->recording_status = CAMERA_RECORDING_UNKNOWN;
    memcpy(cam->mac, cmd->mac, 6);

    ESP_LOGI(TAG, "Station %s connected (managed=%d)", ip_str, cam->managed);

    /* Auto-promote if this MAC was previously saved to NVS or is already
     * managed (slot still allocated from a prior connection cycle). */
    bool auto_manage = cam->managed || nvs_mac_is_saved(cmd->mac);
    if (auto_manage) {
        cam->managed = true;
        /* Ensure a non-empty name — camera_manager requires one.
         * The real name (from gpControl/status) was set by handle_add_camera()
         * the first time the camera was added; on reconnect it is preserved in
         * the cam->name field.  At boot we use the NVS-restored default "Hero4"
         * because we don't re-probe on reconnect. */
        if (cam->name[0] == '\0') {
            strncpy(cam->name, "Hero4", LEGACY_CAMERA_NAME_LEN);
        }
        promote_to_managed_settled(cam_idx);
    }
    /* Otherwise the station appears in the discovered list until the user adds it. */
}

static void handle_disconnect(const legacy_cmd_t *cmd)
{
    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx < 0) return;  /* was never probed — nothing to do */

    legacy_camera_t *cam = &s_cameras[cam_idx];
    char ip_str[16];
    ip_to_str(cam->ip_addr, ip_str, sizeof(ip_str));

    ESP_LOGI(TAG, "Station %s disconnected (managed=%d slot=%d)",
             ip_str, cam->managed, cam->slot);

    if (cam->managed) {
        /* Keep the entry active so it auto-promotes on reconnect and the
         * camera_manager slot stays visible as "disconnected" in the UI. */
        cam->wifi_connected   = false;
        cam->recording_status = CAMERA_RECORDING_UNKNOWN;
        camera_manager_on_wifi_disconnected_by_mac(cmd->mac);
    } else {
        /* Unmanaged/discovered camera: discard the entry entirely. */
        memset(cam, 0, sizeof(*cam));
        cam->slot = -1;
    }

    /* If no legacy cameras remain connected, reset shutter state so the next
     * camera to connect receives start/stop commands fresh. */
    bool any_connected = false;
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (s_cameras[i].active && s_cameras[i].wifi_connected) {
            any_connected = true;
            break;
        }
    }
    if (!any_connected) s_shutter_active = false;
}

static void handle_add_camera(const legacy_cmd_t *cmd)
{
    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx < 0 || !s_cameras[cam_idx].active) {
        ESP_LOGE(TAG, "Add camera: %02X:%02X:%02X:%02X:%02X:%02X not connected",
                 cmd->mac[0], cmd->mac[1], cmd->mac[2],
                 cmd->mac[3], cmd->mac[4], cmd->mac[5]);
        return;
    }

    legacy_camera_t *cam = &s_cameras[cam_idx];

    if (cam->managed) {
        ESP_LOGW(TAG, "Add camera: already managed (slot %d)", cam->slot);
        return;
    }

    char ip_str[16];
    ip_to_str(cam->ip_addr, ip_str, sizeof(ip_str));

    char url[72];
    snprintf(url, sizeof(url), "http://%s/gp/gpControl/status", ip_str);

    /* Prime the RC pairing by sending a keepalive before the HTTP probe. */
    do_udp_keepalive(cam->ip_addr);

    int len = -1;
    for (int attempt = 1; attempt <= PROBE_ON_ADD_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Probing %s (attempt %d/%d)...", ip_str, attempt, PROBE_ON_ADD_ATTEMPTS);
        len = do_http_get(url, PROBE_ON_ADD_TIMEOUT_MS);
        if (len > 0) break;
        if (attempt < PROBE_ON_ADD_ATTEMPTS) {
            do_udp_keepalive(cam->ip_addr);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (len <= 0) {
        ESP_LOGW(TAG, "%s did not respond after %d attempts — not a Hero4",
                 ip_str, PROBE_ON_ADD_ATTEMPTS);
        /* Leave in discovered list; the UI will show a failure message. */
        return;
    }

    char name[LEGACY_CAMERA_NAME_LEN] = "Hero4";
    parse_status_json(s_http_buf, name, sizeof(name));
    if (name[0] == '\0') strncpy(name, "Hero4", sizeof(name));

    strncpy(cam->name, name, LEGACY_CAMERA_NAME_LEN);
    cam->name[LEGACY_CAMERA_NAME_LEN - 1] = '\0';
    cam->managed = true;
    nvs_add_managed_mac(cmd->mac);

    ESP_LOGI(TAG, "Hero4 '%s' at %s confirmed — promoting to managed", name, ip_str);
    promote_to_managed_settled(cam_idx);
}

static void handle_remove_camera(const legacy_cmd_t *cmd)
{
    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx < 0) {
        ESP_LOGW(TAG, "Remove camera: MAC not found in table");
        return;
    }

    legacy_camera_t *cam = &s_cameras[cam_idx];

    if (!cam->managed) {
        ESP_LOGW(TAG, "Remove camera: not managed");
        return;
    }

    int slot = cam->slot;
    ESP_LOGI(TAG, "Removing managed camera '%s' from slot %d", cam->name, slot);

    /* Unregister from camera_manager.  WiFi cameras are not in camera_manager's
     * NVS, so only the RAM slot is freed. */
    if (slot >= 0) {
        camera_manager_remove_slot(slot);
    }

    cam->managed        = false;
    cam->slot           = -1;
    cam->wifi_connected = false;

    nvs_remove_managed_mac(cmd->mac);

    if (!cam->active) {
        /* Camera is not physically connected (was a pre-registered placeholder).
         * Clear the entry entirely so it doesn't occupy a slot. */
        memset(cam, 0, sizeof(*cam));
        cam->slot = -1;
    }
    /* If camera IS active, keep the entry so it reappears in the discovered list. */
    ESP_LOGI(TAG, "Camera removed from managed list");
}

/* ============================================================
 * Periodic work (called on queue timeout)
 * ============================================================ */

static void poll_all_cameras(void)
{
    do_udp_keepalive(0);

    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (!s_cameras[i].active)         continue;
        if (!s_cameras[i].managed)        continue;  /* only managed cameras */
        if (!s_cameras[i].wifi_connected) continue;
        if (s_cameras[i].slot < 0)        continue;

        do_udp_send(UDP_STATUS_REQ, sizeof(UDP_STATUS_REQ),
                    s_cameras[i].ip_addr, GOPRO_UDP_CMD_PORT, "status-req");
    }
}

/* ============================================================
 * Driver vtable implementation
 * ============================================================ */

static esp_err_t drv_start_recording(void *ctx)
{
    legacy_camera_t *cam = (legacy_camera_t *)ctx;
    if (!cam || !cam->active || !cam->managed || !cam->wifi_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_shutter_active) {
        s_shutter_active = true;
        do_udp_shutter(true);
    }
    return ESP_OK;
}

static esp_err_t drv_stop_recording(void *ctx)
{
    legacy_camera_t *cam = (legacy_camera_t *)ctx;
    if (!cam || !cam->active || !cam->managed || !cam->wifi_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_shutter_active) {
        s_shutter_active = false;
        do_udp_shutter(false);
    }
    return ESP_OK;
}

static camera_recording_status_t drv_get_recording_status(void *ctx)
{
    legacy_camera_t *cam = (legacy_camera_t *)ctx;
    if (!cam || !cam->active || !cam->managed || !cam->wifi_connected) {
        return CAMERA_RECORDING_UNKNOWN;
    }
    return cam->recording_status;
}

/* ============================================================
 * UDP receive task
 * ============================================================ */

static void legacy_gopro_rx_task(void *arg)
{
    (void)arg;

    uint8_t            buf[128];
    struct sockaddr_in src_addr;
    socklen_t          src_len = sizeof(src_addr);
    char               ip_str[16];

    ESP_LOGI(TAG, "UDP RX task started — listening on port %d", GOPRO_UDP_LOCAL_PORT);

    while (1) {
        if (s_udp_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int n = recvfrom(s_udp_sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src_addr, &src_len);

        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "UDP recvfrom error (errno %d)", errno);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        ip_to_str(src_addr.sin_addr.s_addr, ip_str, sizeof(ip_str));

        if (n >= 14 && buf[0] == 0x5F) {
            /* Keepalive ACK: "_GPHD_:0:0:2:\x01" — camera confirms RC pairing */
            ESP_LOGD(TAG, "Keepalive ACK from %s", ip_str);

        } else if (n >= 20 && buf[11] == 0x73 && buf[12] == 0x74) {
            /* Status response */
            camera_recording_status_t status;
            if (buf[13] == 0x01) {
                status = CAMERA_RECORDING_IDLE;
            } else {
                status = (buf[15] == 0x01) ? CAMERA_RECORDING_ACTIVE
                                           : CAMERA_RECORDING_IDLE;
            }

            int cam_idx = find_camera_by_ip(src_addr.sin_addr.s_addr);
            if (cam_idx >= 0) {
                s_cameras[cam_idx].recording_status = status;
                ESP_LOGD(TAG, "Status from %s (slot %d): %s", ip_str,
                         s_cameras[cam_idx].slot,
                         status == CAMERA_RECORDING_ACTIVE ? "recording" : "idle");
            } else {
                ESP_LOGW(TAG, "Status response from unknown camera %s", ip_str);
            }

        } else if (n >= 15 && buf[11] == 0x53 && buf[12] == 0x48) {
            /* Shutter echo — informational */
            ESP_LOGD(TAG, "Shutter echo from %s (param=0x%02x)", ip_str, buf[13]);

        } else {
            char hex[320] = {0};
            int  pos = 0;
            for (int i = 0; i < n && pos < (int)sizeof(hex) - 4; i++) {
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x ", buf[i]);
            }
            ESP_LOGW(TAG, "UDP RX unknown %d bytes from %s:%d  [%s]",
                     n, ip_str, ntohs(src_addr.sin_port), hex);
        }
    }
}

/* ============================================================
 * FreeRTOS task
 * ============================================================ */

static void legacy_gopro_task(void *arg)
{
    (void)arg;
    legacy_cmd_t cmd;
    const TickType_t poll_ticks = pdMS_TO_TICKS(POLL_INTERVAL_MS);

    ESP_LOGI(TAG, "Task started — watching for Hero4 connections");

    while (1) {
        if (xQueueReceive(s_queue, &cmd, poll_ticks) == pdTRUE) {
            switch (cmd.type) {
                case CMD_STATION_CONNECT: handle_station_connect(&cmd); break;
                case CMD_DISCONNECT:      handle_disconnect(&cmd);      break;
                case CMD_ADD_CAMERA:      handle_add_camera(&cmd);      break;
                case CMD_REMOVE_CAMERA:   handle_remove_camera(&cmd);   break;
            }
        } else {
            poll_all_cameras();
        }
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

void legacy_gopro_init(void)
{
    memset(s_cameras, 0, sizeof(s_cameras));
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        s_cameras[i].slot = -1;
    }

    nvs_load_managed_macs();

    /* Pre-register previously-managed cameras with camera_manager so they
     * appear as "disconnected" in /api/paired-cameras immediately after boot,
     * before the camera physically reconnects.  This mirrors the BLE behaviour
     * where bonded cameras are always visible in the slot list. */
    for (int i = 0; i < s_saved_mac_count; i++) {
        int cam_idx = find_free_camera_entry();
        if (cam_idx < 0) {
            ESP_LOGW(TAG, "No free camera entries for pre-registration [%d]", i);
            break;
        }
        legacy_camera_t *cam = &s_cameras[cam_idx];
        cam->managed      = true;
        cam->active       = false;   /* not yet on AP */
        cam->wifi_connected = false;
        cam->slot         = -1;
        cam->ip_addr      = 0;
        memcpy(cam->mac, s_saved_macs[i], 6);
        strncpy(cam->name, "Hero4", LEGACY_CAMERA_NAME_LEN);

        int slot = camera_manager_register_wifi_camera(
                       cam->mac, cam->name, &s_legacy_driver, cam);
        if (slot >= 0) {
            cam->slot = slot;
            ESP_LOGI(TAG, "Pre-registered saved camera %02X:%02X:%02X:%02X:%02X:%02X → slot %d",
                     cam->mac[0], cam->mac[1], cam->mac[2],
                     cam->mac[3], cam->mac[4], cam->mac[5], slot);
        } else {
            ESP_LOGE(TAG, "Pre-registration failed for saved camera [%d]", i);
            memset(cam, 0, sizeof(*cam));
            cam->slot = -1;
        }
    }

    udp_socket_init();

    s_queue = xQueueCreate(QUEUE_DEPTH, sizeof(legacy_cmd_t));
    configASSERT(s_queue);

    BaseType_t ret = xTaskCreate(legacy_gopro_task, "legacy_gopro",
                                  TASK_STACK_SIZE, NULL, 5, &s_task);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(legacy_gopro_rx_task, "legacy_gopro_rx",
                      2048, NULL, 4, &s_rx_task);
    configASSERT(ret == pdPASS);

    ESP_LOGI(TAG, "Initialized (%d managed MAC(s) in NVS, UDP socket %s)",
             s_saved_mac_count, s_udp_sock >= 0 ? "ready" : "FAILED");
}

void legacy_gopro_on_station_connected(uint32_t ip, const uint8_t mac[6])
{
    legacy_cmd_t cmd = { .type = CMD_STATION_CONNECT, .ip = ip };
    memcpy(cmd.mac, mac, 6);
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — station connect notification dropped");
    }
}

void legacy_gopro_on_station_disconnected(const uint8_t mac[6])
{
    legacy_cmd_t cmd = { .type = CMD_DISCONNECT, .ip = 0 };
    memcpy(cmd.mac, mac, 6);
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — disconnect notification dropped");
    }
}

int legacy_gopro_get_discovered(legacy_discovered_camera_t *out, int max_count)
{
    int count = 0;
    for (int i = 0; i < LEGACY_MAX_CAMERAS && count < max_count; i++) {
        if (!s_cameras[i].active)  continue;   /* not on the AP */
        if (s_cameras[i].managed)  continue;   /* already managed — shown in paired list */
        memcpy(out[count].mac, s_cameras[i].mac, 6);
        out[count].ip_addr = s_cameras[i].ip_addr;
        count++;
    }
    return count;
}

bool legacy_gopro_add_camera(const uint8_t mac[6])
{
    legacy_cmd_t cmd = { .type = CMD_ADD_CAMERA, .ip = 0 };
    memcpy(cmd.mac, mac, 6);
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — add camera dropped");
        return false;
    }
    return true;
}

bool legacy_gopro_remove_camera(const uint8_t mac[6])
{
    legacy_cmd_t cmd = { .type = CMD_REMOVE_CAMERA, .ip = 0 };
    memcpy(cmd.mac, mac, 6);
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — remove camera dropped");
        return false;
    }
    return true;
}

bool legacy_gopro_is_managed_slot(int slot)
{
    if (slot < 0) return false;
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        /* Check managed only (not active): pre-registered cameras have managed=true
         * but active=false until the camera physically connects. */
        if (s_cameras[i].managed && s_cameras[i].slot == slot) {
            return true;
        }
    }
    return false;
}
