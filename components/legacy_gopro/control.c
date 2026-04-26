/**
 * @file control.c
 * @brief Legacy GoPro (Hero4) camera control commands.
 *
 * This file groups all protocol-level send operations for the Hero4:
 *
 *   UDP commands  (Hero4 RC-remote protocol, port 8484)
 *   -------------------------------------------------------
 *   Shutter ON/OFF   do_udp_shutter()        broadcast to 255.255.255.255
 *   Keepalive        do_udp_keepalive()       unicast to each connected camera
 *   Status request   do_udp_status_request()  unicast to each connected camera
 *   Wake-on-LAN      do_wol_packet()          broadcast magic packet
 *
 *   HTTP commands  (Hero4 gpControl REST API, port 80)
 *   -------------------------------------------------------
 *   Date/time sync   legacy_control_send_date_time()
 *     GET /gp/gpControl/command/setup/date_time?p=%YY%MM%DD%HH%MI%SS
 *     where each field is a percent-encoded hex byte (2-digit year, 1970-based
 *     from the UTC epoch received on CAN 0x602).
 *
 * Shared state (s_cameras[], s_udp_sock) is defined in legacy_gopro.c and
 * accessed here via extern declarations in legacy_gopro_internal.h.
 *
 * All functions in this file must only be called from the legacy_gopro_task
 * context because they share s_udp_sock without a mutex and rely on the task's
 * serial execution model.  The sole exception is legacy_control_send_date_time()
 * which is also task-only due to its blocking HTTP call.
 *
 * Hero4 date/time URL format
 * --------------------------
 * The p= query parameter encodes six fields as consecutive percent-encoded
 * bytes (lowercase hex, zero-padded to two digits):
 *
 *   %YY%MM%DD%HH%MI%SS
 *
 * where YY is the two-digit year (year % 100), not the full four-digit year.
 * This matches the Lua implementation in the RaceCapture script:
 *
 *   for _, v in ipairs({y % 100, mo, d, h, mi, s}) do
 *       dt = dt .. tH(math.floor(v))
 *   end
 *   sT("GET /gp/gpControl/command/setup/date_time?p=" .. dt .. " HTTP/1.0\r\n\r\n")
 *
 * Example: 2023-01-31 03:04:05 UTC → p=%17%01%1f%03%04%05
 */

#include "legacy_gopro_internal.h"
#include "legacy_gopro.h"
#include "can_manager.h"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"    /* inet_pton */

static const char *TAG = "LEGACY_GOPRO";

/* ============================================================
 * Constants
 * ============================================================ */

#define GOPRO_UDP_CMD_PORT      8484
#define GOPRO_UDP_CMD_LEN       14
#define GOPRO_KEEPALIVE_STR     "_GPHD_:0:0:2:0.000000\n"
#define GOPRO_KEEPALIVE_LEN     22

#define WOL_PORT                9       /* standard WOL destination UDP port */
#define WOL_BURST_COUNT         5       /* packets per burst for reliability  */

/** HTTP timeout for the date/time command.  3 s gives the camera time to
 *  respond without holding the task for too long. */
#define DATE_TIME_TIMEOUT_MS    3000

/* ============================================================
 * Hero4 UDP payload constants
 * ============================================================ */

/* Byte[9] must be 0x01 (confirmed from working RaceCapture Lua implementation) */
static const uint8_t UDP_SHUTTER_ON[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x53, 0x48, 0x02    /* 'S','H', param=2 (start) */
};
static const uint8_t UDP_SHUTTER_OFF[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x53, 0x48, 0x00    /* 'S','H', param=0 (stop)  */
};
static const uint8_t UDP_STATUS_REQ[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x73, 0x74           /* 's','t'                  */
};

/* ============================================================
 * Internal helper
 * ============================================================ */

static void ip_to_str(uint32_t ip, char *buf, size_t buf_len)
{
    snprintf(buf, buf_len, "%d.%d.%d.%d",
             (int)(ip         & 0xFF),
             (int)((ip >>  8) & 0xFF),
             (int)((ip >> 16) & 0xFF),
             (int)((ip >> 24) & 0xFF));
}

/* ============================================================
 * UDP send primitive
 * ============================================================ */

void do_udp_send(const void *payload, size_t len,
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

/* ============================================================
 * Shutter command
 * ============================================================ */

/**
 * @brief Broadcast a shutter command to all cameras on the AP subnet.
 *
 * A single packet reaches every Hero4 simultaneously, providing the best
 * possible recording-start synchronisation.
 *
 * @param on  true = start recording, false = stop recording.
 */
void do_udp_shutter(bool on)
{
    const uint8_t *payload = on ? UDP_SHUTTER_ON : UDP_SHUTTER_OFF;
    do_udp_send(payload, GOPRO_UDP_CMD_LEN,
                htonl(INADDR_BROADCAST), GOPRO_UDP_CMD_PORT,
                on ? "shutter-ON(bcast)" : "shutter-OFF(bcast)");
}

/* ============================================================
 * Wake-on-LAN
 * ============================================================ */

/**
 * @brief Send a Wake-on-LAN magic packet for a specific camera MAC.
 *
 * Magic packet: 6 bytes of 0xFF followed by the target MAC repeated 16 times
 * (102 bytes total).  Broadcast to 255.255.255.255:WOL_PORT (port 9) using the
 * existing s_udp_sock, which already has SO_BROADCAST enabled.  Sent
 * WOL_BURST_COUNT times in rapid succession for reliability.
 *
 * @param mac  Target MAC address (6 bytes).
 */
void do_wol_packet(const uint8_t mac[6])
{
    uint8_t packet[102];
    memset(packet, 0xFF, 6);
    for (int i = 0; i < 16; i++) {
        memcpy(packet + 6 + i * 6, mac, 6);
    }
    ESP_LOGI(TAG, "WOL → %02X:%02X:%02X:%02X:%02X:%02X (×%d)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], WOL_BURST_COUNT);
    for (int i = 0; i < WOL_BURST_COUNT; i++) {
        do_udp_send(packet, sizeof(packet),
                    htonl(INADDR_BROADCAST), WOL_PORT, "WOL");
        vTaskDelay(pdMS_TO_TICKS(10)); /* let the TX queue drain between packets */
    }
}

/* ============================================================
 * Keepalive
 * ============================================================ */

/**
 * @brief Send keepalive to every managed+connected camera, plus an optional
 *        extra IP.
 *
 * extra_ip is used during probe and settle to send to a camera that is not
 * yet wifi_connected=true in the table.  Pass 0 to skip.
 *
 * @param extra_ip  Additional target IP (network byte order), or 0.
 */
void do_udp_keepalive(uint32_t extra_ip)
{
    if (s_udp_sock < 0) return;
    for (int i = 0; i < LEGACY_MAX_CAMERAS; i++) {
        if (!s_cameras[i].active)         continue;
        if (!s_cameras[i].managed)        continue;
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
 * Status request
 * ============================================================ */

/**
 * @brief Send a status-request datagram to the camera at s_cameras[cam_idx].
 *
 * @param cam_idx  Index into s_cameras[].
 */
void do_udp_status_request(int cam_idx)
{
    do_udp_send(UDP_STATUS_REQ, sizeof(UDP_STATUS_REQ),
                s_cameras[cam_idx].ip_addr, GOPRO_UDP_CMD_PORT, "status-req");
}

/* ============================================================
 * HTTP date/time sync
 * ============================================================ */

/**
 * @brief Send the Hero4 HTTP date/time command to the given camera IP.
 *
 * Reads the current UTC from can_manager_get_utc_ms() and sends a raw HTTP/1.0
 * GET request over a TCP socket:
 *
 *   GET /gp/gpControl/command/setup/date_time?p=%yy%mm%dd%hh%mi%ss HTTP/1.0\r\n\r\n
 *
 * Each field is percent-encoded as a two-digit lowercase hex byte.  The year
 * field uses only the two lowest digits (year % 100) to match the Hero4 API
 * and the original Lua implementation.
 *
 * We use a raw TCP socket rather than esp_http_client because the Hero4's
 * minimal embedded HTTP server is sensitive to the request format for command
 * endpoints: it responds 500 to HTTP/1.1 requests with extra headers (Host,
 * Connection, etc.) but accepts the bare HTTP/1.0 request that the reference
 * Lua script sends.
 *
 * Returns ESP_ERR_INVALID_STATE immediately if no valid UTC has been received
 * from the RaceCapture yet (GPS lock not acquired).
 *
 * This call blocks for up to DATE_TIME_TIMEOUT_MS milliseconds.  It must only
 * be called from the legacy_gopro_task context.
 *
 * @param ip_str  Camera IP as a dotted-decimal string (e.g. "10.71.79.2").
 * @return        ESP_OK on HTTP 200 success.
 *                ESP_ERR_INVALID_STATE if UTC not available.
 *                Other esp_err_t on network / socket failure.
 */
esp_err_t legacy_control_send_date_time(const char *ip_str)
{
    /* Fetch the current best-estimate UTC. Returns false if GPS lock has not
     * yet been established on the RaceCapture. */
    uint64_t epoch_ms;
    if (!can_manager_get_utc_ms(&epoch_ms)) {
        ESP_LOGW(TAG, "Hero4 %s: date/time skipped — UTC not yet available (no GPS lock)",
                 ip_str);
        return ESP_ERR_INVALID_STATE;
    }

    /* Apply the stored timezone offset so cameras receive local time rather
     * than raw UTC.  Cast through int64_t to handle negative offsets safely. */
    int64_t tz_ms = (int64_t)can_manager_get_tz_offset_hours() * 3600LL * 1000LL;
    epoch_ms = (uint64_t)((int64_t)epoch_ms + tz_ms);

    /* Break epoch into calendar fields. */
    time_t    t = (time_t)(epoch_ms / 1000);
    struct tm ti;
    gmtime_r(&t, &ti);

    /* Hero4 API uses a 2-digit year (year % 100), not the full 4-digit year.
     * This matches the Lua: for _, v in ipairs({y % 100, mo, d, h, mi, s}) */
    int year   = (ti.tm_year + 1900) % 100;
    int month  = ti.tm_mon + 1;    /* tm_mon is 0-based */
    int day    = ti.tm_mday;
    int hour   = ti.tm_hour;
    int minute = ti.tm_min;
    int second = ti.tm_sec;

    /* Build the bare HTTP/1.0 request that the Hero4 expects.
     * Format exactly mirrors the Lua sT() function:
     *   "GET /gp/gpControl/command/setup/date_time?p=" .. dt .. " HTTP/1.0\r\n\r\n" */
    char request[128];
    int req_len = snprintf(request, sizeof(request),
        "GET /gp/gpControl/command/setup/date_time"
        "?p=%%%02x%%%02x%%%02x%%%02x%%%02x%%%02x"
        " HTTP/1.0\r\n\r\n",
        year, month, day, hour, minute, second);

    ESP_LOGI(TAG, "Hero4 %s: SetDateTime → 20%02d-%02d-%02d %02d:%02d:%02d local (UTC%+d)",
             ip_str, year, month, day, hour, minute, second,
             (int)can_manager_get_tz_offset_hours());

    /* Open a dedicated TCP connection to port 80, send the request, and close.
     * This is intentionally identical to the Lua sT() helper. */
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(80),
    };
    if (inet_pton(AF_INET, ip_str, &dest.sin_addr) != 1) {
        ESP_LOGE(TAG, "Hero4 %s: invalid IP address", ip_str);
        return ESP_ERR_INVALID_ARG;
    }

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Hero4 %s: TCP socket create failed (errno %d)", ip_str, errno);
        return ESP_FAIL;
    }

    /* Apply a connect + send timeout so we don't block indefinitely. */
    struct timeval tv = { .tv_sec = DATE_TIME_TIMEOUT_MS / 1000,
                          .tv_usec = (DATE_TIME_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGW(TAG, "Hero4 %s: TCP connect failed (errno %d)", ip_str, errno);
        close(sock);
        return ESP_FAIL;
    }

    int sent = send(sock, request, req_len, 0);
    if (sent < 0) {
        ESP_LOGW(TAG, "Hero4 %s: date/time send failed (errno %d)", ip_str, errno);
        close(sock);
        return ESP_FAIL;
    }

    /* Read the HTTP response status line to confirm 200. */
    char resp[64] = {0};
    int  n        = recv(sock, resp, sizeof(resp) - 1, 0);
    close(sock);

    if (n > 0) {
        /* Response starts with "HTTP/1.x 200 OK" or similar. */
        if (strstr(resp, " 200")) {
            ESP_LOGI(TAG, "Hero4 %s: date/time set OK", ip_str);
            return ESP_OK;
        } else {
            /* Null-terminate and trim any trailing whitespace for the log. */
            for (int i = n - 1; i >= 0 && (resp[i] == '\r' || resp[i] == '\n'); i--) {
                resp[i] = '\0';
            }
            ESP_LOGW(TAG, "Hero4 %s: date/time unexpected response: %.40s", ip_str, resp);
            return ESP_FAIL;
        }
    } else {
        /* Timeout or connection closed before any response — treat as success.
         * Some Hero4 firmware versions close the connection without sending a
         * response to command endpoints. */
        ESP_LOGI(TAG, "Hero4 %s: date/time sent (no response received — assuming OK)",
                 ip_str);
        return ESP_OK;
    }
}
