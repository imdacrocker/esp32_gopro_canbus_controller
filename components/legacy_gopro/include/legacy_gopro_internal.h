/**
 * @file legacy_gopro_internal.h
 * @brief Internal shared types and state for the legacy_gopro component.
 *
 * This header is included by both legacy_gopro.c (which defines the shared
 * state) and control.c (which implements the protocol-level command
 * functions).  It is NOT part of the component's public API — callers
 * outside this component should use legacy_gopro.h only.
 *
 * Layout
 * ------
 *  legacy_gopro.c   — state machine, NVS, FreeRTOS task, public API
 *  control.c        — UDP and HTTP command functions (all wire-level I/O)
 *
 * The shared state (s_cameras[], s_udp_sock) is defined in legacy_gopro.c
 * and accessed from control.c via the extern declarations below.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "camera_manager.h"    /* camera_recording_status_t */
#include "legacy_gopro.h"      /* LEGACY_CAMERA_NAME_LEN   */
#include "esp_err.h"

/* ============================================================
 * Shared constants
 * ============================================================ */

/** Maximum number of cameras tracked simultaneously. */
#define LEGACY_MAX_CAMERAS  4

/* ============================================================
 * Per-camera runtime state
 * ============================================================ */

/**
 * @brief Per-camera runtime state.
 *
 * The driver_ctx pointer stored in camera_manager points directly into the
 * s_cameras[] array — entries must not be moved after registration.
 */
typedef struct {
    bool     active;          /**< true while station is connected to the SoftAP */
    bool     managed;         /**< true after user explicitly adds this camera   */
    bool     wifi_connected;  /**< true while camera is on the AP and registered */
    uint32_t ip_addr;         /**< current IP, network byte order                */
    uint8_t  mac[6];
    int      slot;            /**< camera_manager slot; -1 until managed         */
    char     name[LEGACY_CAMERA_NAME_LEN];
    camera_recording_status_t recording_status;
    volatile TickType_t last_response_tick; /**< FreeRTOS tick of last UDP response; 0 = never */
} legacy_camera_t;

/* ============================================================
 * Shared state (defined in legacy_gopro.c)
 * ============================================================ */

/** Array of per-camera runtime state entries. */
extern legacy_camera_t s_cameras[LEGACY_MAX_CAMERAS];

/** UDP socket bound to GOPRO_UDP_LOCAL_PORT (8383). */
extern int s_udp_sock;

/* ============================================================
 * Functions defined in control.c — called from legacy_gopro.c
 * ============================================================ */

/**
 * @brief Send a raw UDP datagram.
 *
 * @param payload   Data to send.
 * @param len       Byte count.
 * @param dest_ip   Destination IP in network byte order.
 * @param dest_port Destination UDP port (host byte order).
 * @param label     Short label for debug logging.
 */
void do_udp_send(const void *payload, size_t len,
                 uint32_t dest_ip, uint16_t dest_port,
                 const char *label);

/**
 * @brief Broadcast a shutter command to all cameras.
 *
 * @param on true = start recording, false = stop recording.
 */
void do_udp_shutter(bool on);

/**
 * @brief Send a Wake-on-LAN magic packet for the given MAC.
 *
 * @param mac  Target MAC address (6 bytes).
 */
void do_wol_packet(const uint8_t mac[6]);

/**
 * @brief Send a keepalive to every managed+connected camera.
 *
 * @param extra_ip  Additional IP to send to even if not in s_cameras[]
 *                  (used during settle/probe).  Pass 0 to skip.
 */
void do_udp_keepalive(uint32_t extra_ip);

/**
 * @brief Send a status-request datagram to camera at index cam_idx.
 *
 * @param cam_idx  Index into s_cameras[].
 */
void do_udp_status_request(int cam_idx);

/**
 * @brief Send the Hero4 HTTP date/time command to the given camera IP.
 *
 * Reads the current UTC from can_manager_get_utc_ms() and issues an HTTP
 * GET to /gp/gpControl/command/setup/date_time?p=... using the Hero4
 * percent-encoded two-digit-year format.  Returns ESP_ERR_INVALID_STATE if
 * no valid UTC is available yet.
 *
 * This call is blocking (HTTP timeout up to DATE_TIME_TIMEOUT_MS ms).  It
 * must only be invoked from the legacy_gopro_task context.
 *
 * @param ip_str  Camera IP as a dotted-decimal string (e.g. "10.71.79.2").
 * @return        ESP_OK on HTTP 200, ESP_ERR_INVALID_STATE if no UTC,
 *                or an error code on network failure.
 */
esp_err_t legacy_control_send_date_time(const char *ip_str);
