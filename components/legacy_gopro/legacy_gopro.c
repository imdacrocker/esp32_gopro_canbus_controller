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
 * Connection flow
 * ---------------
 * Connected AP stations that have a DHCP IP are exposed via
 * /api/legacy/discovered.  Stations still acquiring their address are hidden
 * until DHCP completes.
 *
 *  1. WIFI_EVENT_AP_STACONNECTED fires → wifi_manager adds the station to its
 *     table and calls legacy_gopro_on_station_wifi_associated(mac).
 *  2a. Known MAC (in NVS): saved IP is looked up and CMD_STATION_CONNECT is
 *      posted immediately.  An HTTP probe verifies the camera at the saved IP
 *      before it is auto-promoted to managed.
 *  2b. Unknown MAC: silently ignored; the station appears in
 *      /api/legacy/discovered once wifi_manager has its DHCP IP.
 *  3.  IP_EVENT_ASSIGNED_IP_TO_CLIENT fires → wifi_manager calls
 *      legacy_gopro_on_station_connected(ip, mac).
 *  3a. Known MAC (in NVS): CMD_IP_UPDATE is posted to update the IP in NVS
 *      and s_cameras.  If the camera is already connected the keepalives are
 *      redirected to the new IP immediately.  If it is associated but not yet
 *      connected (probe at old IP failed or in progress), a fresh
 *      CMD_STATION_CONNECT is queued at the new IP.
 *  3b. Unknown MAC: ignored here; wifi_manager's station table already has
 *      the IP and will show it in /api/legacy/discovered.
 *  4.  User clicks Add in the web UI → /api/legacy/add → CMD_ADD_CAMERA.
 *      The task probes the supplied IP and promotes the camera to managed on
 *      success, saving the MAC and IP to NVS.
 *
 * Once managed, a camera is registered with camera_manager, receives
 * keepalives every 2 s, is polled for recording status, and responds to
 * shutter commands.  Its MAC and IP are saved to NVS so that subsequent
 * reconnects auto-promote it without user action.
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
#include "legacy_gopro_internal.h"
#include "camera_manager.h"
#include "camera_driver.h"
#include "wifi_manager.h"

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

#define PROBE_ON_ADD_TIMEOUT_MS 5000    /* HTTP timeout for on-demand probe (add flow) */
#define PROBE_ON_ADD_ATTEMPTS   2       /* max attempts when user clicks Add */

/* Reconnect probe parameters.  A cold-boot Hero4 needs 3-5 seconds after L2
 * association before its HTTP server is ready, so we use a generous per-attempt
 * timeout and retry a few times before giving up.  Total worst-case wait:
 * PROBE_ON_RECONNECT_ATTEMPTS × (PROBE_ON_RECONNECT_TIMEOUT_MS + PROBE_ON_RECONNECT_RETRY_DELAY_MS)
 * = 3 × (5000 + 2000) = 21 s.  A warm camera responds in < 200 ms. */
#define PROBE_ON_RECONNECT_TIMEOUT_MS    5000   /* was 2000 — cold boot needs longer */
#define PROBE_ON_RECONNECT_ATTEMPTS      3
#define PROBE_ON_RECONNECT_RETRY_DELAY_MS 2000
#define POLL_INTERVAL_MS        2000    /* keepalive + status poll cadence */
#define HTTP_BUF_SIZE           2560
#define TASK_STACK_SIZE         8192
#define QUEUE_DEPTH             16  /* increased from 8: a 21 s probe loop can queue up
                                     * many CMD_DISCONNECTs while blocked; a depth of 8
                                     * left no room for the follow-on CMD_STATION_CONNECT,
                                     * causing it to be silently dropped ("not always
                                     * trying to pair"). */
#define SETTLE_DURATION_MS      2000    /* keepalive settle before on_wifi_connected */
#define SETTLE_STEP_MS          500

/* UDP local receive port — bound by udp_socket_init(). */
#define GOPRO_UDP_LOCAL_PORT    8383

/* Wake-on-LAN (WOL command implementation is in control.c) */
#define WOL_ENABLED             0       /* set to 1 to re-enable WOL broadcasts */
#define WOL_TIMEOUT_MS          5000    /* send WOL if camera silent for this long */

/* NVS storage for managed cameras (MAC + last-known IP) */
#define NVS_NAMESPACE           "lgcy_gopro"
#define NVS_KEY_MANAGED_MACS    "managed_cams"

/* ============================================================
 * Internal types
 * ============================================================ */

/* UDP payload constants and the legacy_camera_t struct are defined in
 * legacy_gopro_internal.h and control.c respectively. */

typedef enum {
    CMD_STATION_CONNECT,  /* known managed camera reconnected: probe at saved IP */
    CMD_DISCONNECT,       /* station left the AP */
    CMD_ADD_CAMERA,       /* user clicked Add: probe then promote if Hero4 */
    CMD_REMOVE_CAMERA,    /* un-manage a camera */
    CMD_SYNC_TIME,        /* send date/time to all currently connected cameras */
    CMD_IP_UPDATE,        /* managed camera acquired a new DHCP IP: update NVS + s_cameras */
} legacy_cmd_type_t;

/**
 * @brief Per-camera persistent record stored in NVS.
 *
 * Packed so the NVS blob size is exactly LEGACY_MAX_CAMERAS × 10 bytes
 * and the layout is unambiguous across builds.
 */
typedef struct {
    uint8_t  mac[6];     /**< Station MAC address */
    uint32_t ip_addr;    /**< Last known IP in network byte order; 0 if unknown */
} __attribute__((packed)) legacy_saved_camera_t;

typedef struct {
    legacy_cmd_type_t type;
    uint32_t          ip;
    uint8_t           mac[6];
} legacy_cmd_t;

/* legacy_camera_t is defined in legacy_gopro_internal.h */

/* ============================================================
 * Module state
 * ============================================================ */

static QueueHandle_t   s_queue   = NULL;
static TaskHandle_t    s_task    = NULL;
static TaskHandle_t    s_rx_task = NULL;

/* Shared with control.c via extern declarations in legacy_gopro_internal.h. */
legacy_camera_t s_cameras[LEGACY_MAX_CAMERAS];
int             s_udp_sock = -1;

/* Last shutter state broadcast.  Suppresses duplicate broadcasts when
 * camera_manager calls drv_start/stop_recording on multiple legacy slots. */
static bool            s_shutter_active = false;

/* NVS-persisted managed cameras (MAC + last-known IP) — loaded at init, updated on add/remove. */
static legacy_saved_camera_t s_saved_cameras[LEGACY_MAX_CAMERAS];
static int                   s_saved_camera_count = 0;

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

/** Return the index in s_saved_cameras[] for a MAC, or -1 if not found. */
static int nvs_find_saved_idx(const uint8_t mac[6])
{
    for (int i = 0; i < s_saved_camera_count; i++) {
        if (memcmp(s_saved_cameras[i].mac, mac, 6) == 0) return i;
    }
    return -1;
}

/** Return true if mac is in the saved managed-camera list. */
static bool nvs_mac_is_saved(const uint8_t mac[6])
{
    return nvs_find_saved_idx(mac) >= 0;
}

/**
 * Return the saved IP address for a MAC (network byte order), or 0 if not found.
 * Called from legacy_gopro_on_station_wifi_associated() in ISR-safe context.
 */
static uint32_t nvs_get_saved_ip(const uint8_t mac[6])
{
    int idx = nvs_find_saved_idx(mac);
    return (idx >= 0) ? s_saved_cameras[idx].ip_addr : 0;
}

/** Write the current s_saved_cameras[] array to NVS. */
static void nvs_persist(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }
    if (s_saved_camera_count == 0) {
        nvs_erase_key(h, NVS_KEY_MANAGED_MACS);
    } else {
        err = nvs_set_blob(h, NVS_KEY_MANAGED_MACS,
                           s_saved_cameras,
                           (size_t)(s_saved_camera_count * sizeof(legacy_saved_camera_t)));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS set_blob failed: %s", esp_err_to_name(err));
        }
    }
    nvs_commit(h);
    nvs_close(h);
}

/** Load managed cameras (MAC+IP) from NVS into s_saved_cameras[]. */
static void nvs_load_managed_cameras(void)
{
    s_saved_camera_count = 0;
    memset(s_saved_cameras, 0, sizeof(s_saved_cameras));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open (read) failed: %s", esp_err_to_name(err));
        return;
    }

    size_t blob_len = sizeof(s_saved_cameras);
    err = nvs_get_blob(h, NVS_KEY_MANAGED_MACS, s_saved_cameras, &blob_len);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) return;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS get_blob failed: %s", esp_err_to_name(err));
        return;
    }

    s_saved_camera_count = (int)(blob_len / sizeof(legacy_saved_camera_t));
    ESP_LOGI(TAG, "Loaded %d managed camera(s) from NVS", s_saved_camera_count);
    for (int i = 0; i < s_saved_camera_count; i++) {
        const uint8_t *m = s_saved_cameras[i].mac;
        char ip_str[16];
        ip_to_str(s_saved_cameras[i].ip_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "  [%d] %02X:%02X:%02X:%02X:%02X:%02X  IP=%s",
                 i, m[0], m[1], m[2], m[3], m[4], m[5], ip_str);
    }
}

/**
 * Add or update a camera in the saved list and persist to NVS.
 *
 * If the MAC is already saved, updates its IP (e.g. if it changed between
 * connections).  If new, appends it.  No-ops if the list is full.
 */
static void nvs_add_managed_camera(const uint8_t mac[6], uint32_t ip_addr)
{
    int idx = nvs_find_saved_idx(mac);
    if (idx >= 0) {
        /* Update existing entry's IP */
        s_saved_cameras[idx].ip_addr = ip_addr;
    } else {
        if (s_saved_camera_count >= LEGACY_MAX_CAMERAS) {
            ESP_LOGW(TAG, "NVS managed camera list full — cannot save");
            return;
        }
        idx = s_saved_camera_count++;
        memcpy(s_saved_cameras[idx].mac, mac, 6);
        s_saved_cameras[idx].ip_addr = ip_addr;
    }
    nvs_persist();
}

/** Remove a camera from the saved list and persist. */
static void nvs_remove_managed_mac(const uint8_t mac[6])
{
    int idx = nvs_find_saved_idx(mac);
    if (idx < 0) return;

    int remaining = s_saved_camera_count - idx - 1;
    if (remaining > 0) {
        memmove(&s_saved_cameras[idx], &s_saved_cameras[idx + 1],
                (size_t)(remaining * sizeof(legacy_saved_camera_t)));
    }
    s_saved_camera_count--;
    nvs_persist();
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

/* UDP send functions (do_udp_send, do_udp_shutter, do_wol_packet,
 * do_udp_keepalive, do_udp_status_request) are defined in control.c.
 * Their declarations are in legacy_gopro_internal.h. */

/* ============================================================
 * HTTP client  (probe/status — task-only, uses shared response buffer)
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

    /* Sync the camera's clock now that it is fully connected.
     * Skipped silently if no UTC is available yet (GPS lock not acquired);
     * the UTC-acquired callback will call legacy_gopro_sync_time_all() later. */
    legacy_control_send_date_time(ip_str);

    ESP_LOGI(TAG, "Hero4 '%s' at %s slot %d ready", cam->name, ip_str, cam->slot);
}

/* ============================================================
 * Command handlers — all run inside the task
 * ============================================================ */

static void handle_station_connect(const legacy_cmd_t *cmd)
{
    /* Wait briefly for DHCP to complete before committing to a probe IP.
     * WIFI_EVENT_AP_STACONNECTED fires before the camera sends its DHCP
     * Discover, so the station table IP is often still 0 when we arrive here.
     * Polling for up to DHCP_SETTLE_MAX_MS avoids burning 21 s probing the
     * stale saved IP when the camera has already received a different address. */
#define DHCP_SETTLE_POLL_MS   100
#define DHCP_SETTLE_MAX_MS   1500

    uint32_t probe_ip = cmd->ip;
    for (int w = 0; w * DHCP_SETTLE_POLL_MS < DHCP_SETTLE_MAX_MS; w++) {
        uint32_t live = wifi_manager_get_station_ip(cmd->mac);
        if (live != 0) {
            if (live != probe_ip) {
                char old_str[16], new_str[16];
                ip_to_str(probe_ip, old_str, sizeof(old_str));
                ip_to_str(live, new_str, sizeof(new_str));
                ESP_LOGI(TAG, "DHCP settled at %s (saved was %s) — using new IP",
                         new_str, old_str);
                probe_ip = live;
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(DHCP_SETTLE_POLL_MS));
    }
    /* If the wait expired without a DHCP event (camera uses a cached/static
     * IP and won't send a DHCP request), probe_ip is still the saved IP,
     * which is the correct behaviour. */

    char ip_str[16];
    ip_to_str(probe_ip, ip_str, sizeof(ip_str));

    /* CMD_STATION_CONNECT is only ever posted for MACs that are in NVS, so
     * the camera must be pre-loaded into s_cameras by legacy_gopro_init(). */
    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx < 0) {
        ESP_LOGE(TAG, "handle_station_connect: MAC not found in s_cameras — "
                      "should not happen");
        return;
    }

    legacy_camera_t *cam = &s_cameras[cam_idx];
    cam->active           = true;
    cam->managed          = true;
    cam->wifi_connected   = false;   /* set true inside promote_to_managed_settled */
    cam->ip_addr          = probe_ip;
    cam->recording_status = CAMERA_RECORDING_UNKNOWN;
    memcpy(cam->mac, cmd->mac, 6);
    if (cam->name[0] == '\0') {
        strncpy(cam->name, "Hero4", LEGACY_CAMERA_NAME_LEN);
    }

    ESP_LOGI(TAG, "Managed camera reconnecting — probing %s", ip_str);

    /* Verify the camera is reachable at its saved IP.  A cold-boot camera
     * takes a few seconds after L2 association before its HTTP server is
     * ready, so we retry with a delay.  If all attempts fail the IP may be
     * stale; CMD_IP_UPDATE will queue a fresh CMD_STATION_CONNECT if DHCP
     * fires with a new address. */
    char url[72];
    snprintf(url, sizeof(url), "http://%s/gp/gpControl/status", ip_str);

    int len = -1;
    for (int attempt = 1; attempt <= PROBE_ON_RECONNECT_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Probing %s (attempt %d/%d, timeout %d ms)...",
                 ip_str, attempt, PROBE_ON_RECONNECT_ATTEMPTS,
                 PROBE_ON_RECONNECT_TIMEOUT_MS);
        len = do_http_get(url, PROBE_ON_RECONNECT_TIMEOUT_MS);
        if (len > 0) break;
        if (attempt < PROBE_ON_RECONNECT_ATTEMPTS) {
            ESP_LOGW(TAG, "No response from %s (attempt %d) — waiting %d ms",
                     ip_str, attempt, PROBE_ON_RECONNECT_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(PROBE_ON_RECONNECT_RETRY_DELAY_MS));
        }
    }

    if (len <= 0) {
        ESP_LOGW(TAG, "Hero4 at %s did not respond after %d attempt(s) — "
                     "leaving inactive; AP will evict after inactivity timeout",
                 ip_str, PROBE_ON_RECONNECT_ATTEMPTS);
        return;
    }

    /* Update the camera name from the live response. */
    char name[LEGACY_CAMERA_NAME_LEN] = "Hero4";
    parse_status_json(s_http_buf, name, sizeof(name));
    if (name[0] != '\0') {
        strncpy(cam->name, name, LEGACY_CAMERA_NAME_LEN);
        cam->name[LEGACY_CAMERA_NAME_LEN - 1] = '\0';
    }

    promote_to_managed_settled(cam_idx);
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
        /* Idempotency guard: multiple CMD_DISCONNECTs queue up while the task
         * is blocked in a probe loop.  Only the first one needs to do anything;
         * subsequent calls for the same MAC while wifi_connected is already
         * false are no-ops, preventing redundant camera_manager notifications. */
        if (!cam->wifi_connected) return;

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
    /* Reject if this MAC is already managed (double-click / race condition). */
    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx >= 0 && s_cameras[cam_idx].managed) {
        ESP_LOGW(TAG, "Add camera: %02X:%02X:%02X:%02X:%02X:%02X already managed (slot %d)",
                 cmd->mac[0], cmd->mac[1], cmd->mac[2],
                 cmd->mac[3], cmd->mac[4], cmd->mac[5],
                 s_cameras[cam_idx].slot);
        return;
    }

    /* The IP must be supplied by the caller from wifi_manager's station table.
     * The /api/legacy/discovered endpoint only returns stations that already
     * have a DHCP address, so ip should never be 0 here in normal operation. */
    if (cmd->ip == 0) {
        ESP_LOGW(TAG, "Add camera: no IP address supplied — device may not "
                      "have a DHCP address yet; try again in a moment");
        return;
    }
    uint32_t probe_ip = cmd->ip;

    char ip_str[16];
    ip_to_str(probe_ip, ip_str, sizeof(ip_str));

    char url[72];
    snprintf(url, sizeof(url), "http://%s/gp/gpControl/status", ip_str);

    /* Prime the RC pairing with a keepalive before the HTTP probe. */
    do_udp_keepalive(probe_ip);

    int len = -1;
    for (int attempt = 1; attempt <= PROBE_ON_ADD_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Probing %s (attempt %d/%d)...", ip_str, attempt, PROBE_ON_ADD_ATTEMPTS);
        len = do_http_get(url, PROBE_ON_ADD_TIMEOUT_MS);
        if (len > 0) break;
        if (attempt < PROBE_ON_ADD_ATTEMPTS) {
            do_udp_keepalive(probe_ip);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (len <= 0) {
        ESP_LOGW(TAG, "%s did not respond after %d attempts — not a Hero4",
                 ip_str, PROBE_ON_ADD_ATTEMPTS);
        /* Leave in the station table; the UI will detect the MAC is still
         * present in /api/legacy/discovered and show a failure message. */
        return;
    }

    char name[LEGACY_CAMERA_NAME_LEN] = "Hero4";
    parse_status_json(s_http_buf, name, sizeof(name));
    if (name[0] == '\0') strncpy(name, "Hero4", sizeof(name));

    /* Allocate a new camera slot (it was never in s_cameras[] before). */
    cam_idx = find_free_camera_entry();
    if (cam_idx < 0) {
        ESP_LOGE(TAG, "No free camera entries for Hero4 at %s", ip_str);
        return;
    }

    legacy_camera_t *cam = &s_cameras[cam_idx];
    cam->active           = true;
    cam->managed          = true;
    cam->wifi_connected   = false;
    cam->ip_addr          = probe_ip;
    cam->recording_status = CAMERA_RECORDING_UNKNOWN;
    cam->slot             = -1;
    memcpy(cam->mac, cmd->mac, 6);
    strncpy(cam->name, name, LEGACY_CAMERA_NAME_LEN);
    cam->name[LEGACY_CAMERA_NAME_LEN - 1] = '\0';

    /* Persist MAC and IP so subsequent reconnects auto-promote without probing. */
    nvs_add_managed_camera(cmd->mac, probe_ip);

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
 * IP update (CMD_IP_UPDATE handler)
 * ============================================================ */

/**
 * A managed camera acquired a new DHCP lease — update NVS and live state.
 *
 * This is the rare case where a camera re-pairs (e.g. after a factory reset)
 * and is assigned a different IP than the one stored in NVS.  We persist the
 * new IP immediately so the next boot picks it up, then:
 *
 *  - If already connected (wifi_connected=true): redirect UDP keepalives and
 *    update camera_manager's IP record in place.
 *  - If associated but not yet connected (probe at old IP in flight or failed):
 *    queue a fresh CMD_STATION_CONNECT at the new IP so the probe retries there.
 *  - If not yet in s_cameras at all: NVS update is sufficient; the next
 *    CMD_STATION_CONNECT will use the new IP from nvs_get_saved_ip().
 */
static void handle_ip_update(const legacy_cmd_t *cmd)
{
    char new_ip_str[16];
    ip_to_str(cmd->ip, new_ip_str, sizeof(new_ip_str));

    /* Always persist the new IP so future reconnects go to the right address. */
    nvs_add_managed_camera(cmd->mac, cmd->ip);

    int cam_idx = find_camera_by_mac(cmd->mac);
    if (cam_idx < 0) {
        /* Not yet in s_cameras — NVS update is all we need. */
        ESP_LOGI(TAG, "IP update %02X:%02X:%02X:%02X:%02X:%02X → %s (not yet active)",
                 cmd->mac[0], cmd->mac[1], cmd->mac[2],
                 cmd->mac[3], cmd->mac[4], cmd->mac[5], new_ip_str);
        return;
    }

    legacy_camera_t *cam = &s_cameras[cam_idx];

    if (cam->ip_addr == cmd->ip) {
        return;  /* IP unchanged — nothing more to do */
    }

    char old_ip_str[16];
    ip_to_str(cam->ip_addr, old_ip_str, sizeof(old_ip_str));
    ESP_LOGI(TAG, "Managed camera %02X:%02X:%02X:%02X:%02X:%02X IP changed: %s → %s",
             cmd->mac[0], cmd->mac[1], cmd->mac[2],
             cmd->mac[3], cmd->mac[4], cmd->mac[5], old_ip_str, new_ip_str);

    cam->ip_addr = cmd->ip;

    if (cam->wifi_connected) {
        /* Camera is fully connected — redirect keepalives to the new IP and
         * tell camera_manager so its status IP stays accurate. */
        do_udp_keepalive(cam->ip_addr);
        if (cam->slot >= 0) {
            camera_manager_on_wifi_connected(cam->slot, cam->ip_addr);
        }
    } else if (cam->active || cam->managed) {
        /* Camera is associated at L2 but the probe at the old IP either failed,
         * is still running, or never started (managed placeholder with ip=0 in
         * NVS — on_station_wifi_associated ignored it because saved_ip was 0,
         * so this DHCP event is the first opportunity to probe it).
         * Mark active so subsequent disconnect/reconnect bookkeeping is correct. */
        cam->active = true;
        legacy_cmd_t reconnect = { .type = CMD_STATION_CONNECT, .ip = cmd->ip };
        memcpy(reconnect.mac, cmd->mac, 6);
        if (xQueueSend(s_queue, &reconnect, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Command queue full — reconnect at new IP dropped");
        }
    }
}

/* ============================================================
 * Date/time sync (CMD_SYNC_TIME handler)
 * ============================================================ */

/**
 * Send the Hero4 date/time command to every managed+connected camera.
 *
 * Called from the task in response to CMD_SYNC_TIME, which is posted by
 * legacy_gopro_sync_time_all().  Running inside the task keeps all blocking
 * HTTP calls off the caller's stack (e.g. the CAN manager task).
 */
static void handle_sync_time(void)
{
    char ip_str[16];
    int synced = 0;
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (!s_cameras[i].active)         continue;
        if (!s_cameras[i].managed)        continue;
        if (!s_cameras[i].wifi_connected) continue;
        ip_to_str(s_cameras[i].ip_addr, ip_str, sizeof(ip_str));
        if (legacy_control_send_date_time(ip_str) == ESP_OK) {
            synced++;
        }
    }
    ESP_LOGI(TAG, "Date/time sync complete — %d camera(s) updated", synced);
}

/* ============================================================
 * Periodic work (called on queue timeout)
 * ============================================================ */

static void poll_all_cameras(void)
{
    do_udp_keepalive(0);

    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        /* WOL: send for any managed camera that has not responded recently.
         * Intentionally ignores wifi_connected / active — Hero4 Wi-Fi quirks
         * mean the camera may still be reachable via broadcast even when the
         * DHCP/AP state is unclear.  last_response_tick == 0 (never responded)
         * will always satisfy the condition once WOL_TIMEOUT_MS has elapsed
         * from boot, which is the desired behaviour.
         * Set WOL_ENABLED to 1 (in the constants block above) to activate. */
#if WOL_ENABLED
        {
            TickType_t now = xTaskGetTickCount();
            if (s_cameras[i].managed &&
                (now - s_cameras[i].last_response_tick) > pdMS_TO_TICKS(WOL_TIMEOUT_MS)) {
                do_wol_packet(s_cameras[i].mac);
            }
        }
#endif

        /* Status poll: only for fully-connected cameras */
        if (!s_cameras[i].active)         continue;
        if (!s_cameras[i].managed)        continue;
        if (!s_cameras[i].wifi_connected) continue;
        if (s_cameras[i].slot < 0)        continue;

        do_udp_status_request(i);
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
            int cam_idx = find_camera_by_ip(src_addr.sin_addr.s_addr);
            if (cam_idx >= 0) {
                s_cameras[cam_idx].last_response_tick = xTaskGetTickCount();
            }

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
                s_cameras[cam_idx].recording_status   = status;
                s_cameras[cam_idx].last_response_tick = xTaskGetTickCount();
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
                case CMD_STATION_CONNECT:  handle_station_connect(&cmd);   break;
                case CMD_DISCONNECT:       handle_disconnect(&cmd);        break;
                case CMD_ADD_CAMERA:       handle_add_camera(&cmd);        break;
                case CMD_REMOVE_CAMERA:    handle_remove_camera(&cmd);     break;
                case CMD_SYNC_TIME:        handle_sync_time();             break;
                case CMD_IP_UPDATE:        handle_ip_update(&cmd);         break;
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

    nvs_load_managed_cameras();

    /* Pre-register previously-managed cameras with camera_manager so they
     * appear as "disconnected" in /api/paired-cameras immediately after boot,
     * before the camera physically reconnects.  This mirrors the BLE behaviour
     * where bonded cameras are always visible in the slot list.
     * The saved IP is stored in the entry so that when the camera associates
     * (without DHCP) we can immediately attempt a TCP probe at that address. */
    for (int i = 0; i < s_saved_camera_count; i++) {
        int cam_idx = find_free_camera_entry();
        if (cam_idx < 0) {
            ESP_LOGW(TAG, "No free camera entries for pre-registration [%d]", i);
            break;
        }
        legacy_camera_t *cam = &s_cameras[cam_idx];
        cam->managed        = true;
        cam->active         = false;   /* not yet on AP */
        cam->wifi_connected = false;
        cam->slot           = -1;
        cam->ip_addr        = s_saved_cameras[i].ip_addr;  /* may be 0 on first boot */
        memcpy(cam->mac, s_saved_cameras[i].mac, 6);
        strncpy(cam->name, "Hero4", LEGACY_CAMERA_NAME_LEN);

        int slot = camera_manager_register_wifi_camera(
                       cam->mac, cam->name, &s_legacy_driver, cam);
        if (slot >= 0) {
            cam->slot = slot;
            char ip_str[16];
            ip_to_str(cam->ip_addr, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "Pre-registered %02X:%02X:%02X:%02X:%02X:%02X  IP=%s → slot %d",
                     cam->mac[0], cam->mac[1], cam->mac[2],
                     cam->mac[3], cam->mac[4], cam->mac[5], ip_str, slot);
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

    ESP_LOGI(TAG, "Initialized (%d managed camera(s) in NVS, UDP socket %s)",
             s_saved_camera_count, s_udp_sock >= 0 ? "ready" : "FAILED");
}

void legacy_gopro_on_station_connected(uint32_t ip, const uint8_t mac[6])
{
    /* Only act on managed cameras (MAC in NVS).  Unknown MACs are tracked by
     * wifi_manager's station table and need no action here. */
    if (!nvs_mac_is_saved(mac)) {
        return;
    }
    /* Managed camera acquired a DHCP lease — this is the uncommon case where
     * the camera re-paired and may have received a different IP than the one
     * saved in NVS.  CMD_IP_UPDATE will persist the new IP and redirect live
     * traffic if the camera is already connected. */
    legacy_cmd_t cmd = { .type = CMD_IP_UPDATE, .ip = ip };
    memcpy(cmd.mac, mac, 6);
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — IP update notification dropped");
    }
}

void legacy_gopro_on_station_wifi_associated(const uint8_t mac[6])
{
    uint32_t saved_ip = nvs_get_saved_ip(mac);

    if (saved_ip != 0) {
        /* Known managed camera: probe at the saved IP immediately.  If the
         * camera also sends a DHCP request (e.g. after re-pairing with a new
         * IP), legacy_gopro_on_station_connected() will post CMD_IP_UPDATE to
         * correct both NVS and the live entry. */
        legacy_cmd_t cmd = { .type = CMD_STATION_CONNECT, .ip = saved_ip };
        memcpy(cmd.mac, mac, 6);
        if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Command queue full — managed camera reconnect dropped");
        }
    } else {
        /* Unknown MAC — not a managed camera.  wifi_manager's station table
         * already tracks this device; it will appear in /api/legacy/discovered
         * once DHCP assigns it an IP. */
        ESP_LOGD(TAG, "Unknown station %02X:%02X:%02X:%02X:%02X:%02X — not managed, ignoring",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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

bool legacy_gopro_add_camera(const uint8_t mac[6], uint32_t ip)
{
    legacy_cmd_t cmd = { .type = CMD_ADD_CAMERA, .ip = ip };
    memcpy(cmd.mac, mac, 6);
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — add camera dropped");
        return false;
    }
    return true;
}

bool legacy_gopro_is_managed_mac(const uint8_t mac[6])
{
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (s_cameras[i].managed && memcmp(s_cameras[i].mac, mac, 6) == 0) {
            return true;
        }
    }
    return false;
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

void legacy_gopro_sync_time_all(void)
{
    /* Post CMD_SYNC_TIME to the task queue so the blocking HTTP call runs
     * inside the legacy_gopro task, not on the caller's stack (which may be
     * the CAN manager task or another low-stack context). */
    legacy_cmd_t cmd = { .type = CMD_SYNC_TIME, .ip = 0 };
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — sync-time request dropped");
    }
}
