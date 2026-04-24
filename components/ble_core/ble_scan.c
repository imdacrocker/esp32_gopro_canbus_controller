/**
 * @file ble_scan.c
 * @brief BLE discovery scan management — active scans, passive background scans.
 *
 * Two scan modes are used:
 *
 *  Active discovery scan (30 s):
 *    Started by ble_core_start_discovery().  Surfaces all advertisement packets
 *    to the on_disc callback with deduplication disabled.  Used when the user
 *    requests a camera scan from the web UI.  Cancelled by
 *    ble_core_stop_discovery(), which transitions back to the passive background
 *    scan if any cameras are still disconnected.
 *
 *  Passive background scan:
 *    Started automatically after boot reconnect (ble_init.c) if any known
 *    cameras are not yet connected, and after an active scan ends.  Only sends
 *    advertisements to the BLE stack for reconnect processing — the on_disc
 *    callback is NOT invoked during passive scans.  Suppressed when
 *    has_disconnected_cameras() returns false (all cameras connected).
 *
 * The BLE_GAP_EVENT_DISC_COMPLETE event fires at the end of the 30-second
 * active scan window and is handled here to restart the background scan.
 */

#include "ble_core_internal.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"

static const char *TAG = "ble_core";

/* State for event-posted direct connect */
static ble_addr_t           s_connect_target;
static struct ble_npl_event s_connect_by_addr_event;

/* Forward declaration for the scan event callback */
static int scan_event_cb(struct ble_gap_event *event, void *arg);

/* --------------------------------------------------------------------------
 * Background scan — passive, runs forever, deduplicates at controller level
 * -------------------------------------------------------------------------- */
void start_scan(void)
{
    struct ble_gap_disc_params disc_params = {
        .itvl              = 0,
        .window            = 0,
        .filter_policy     = 0,
        .limited           = 0,
        .passive           = 1,
        /* Deduplicate so the host task is not called for every advertisement
         * from an already-seen device.  The filter resets each scan interval,
         * so cameras that come back online are still detected. */
        //.filter_duplicates = 1, // Disabled this, as we are now scanning forever.  I think this is correct?  - DAC
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

/* --------------------------------------------------------------------------
 * Discovery scan — passive, runs for 120 seconds, no deduplication so every
 * advertisement packet is surfaced to the on_disc callback.
 * Runs on the NimBLE host task (posted via event queue).
 * -------------------------------------------------------------------------- */
static void start_discovery_cb(struct ble_npl_event *ev)
{
    struct ble_gap_disc_params disc_params = {
        .itvl              = 0,
        .window            = 0,
        .filter_policy     = 0,
        .limited           = 0,
        .passive           = 1,
        .filter_duplicates = 0,
    };

    ble_gap_disc_cancel(); /* stop background scan; ignore error if not running */

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 120000,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc (discovery) failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Discovery scan started — 120 seconds");
    }
}

static struct ble_npl_event s_start_disc_event;

/* --------------------------------------------------------------------------
 * Conditional scan start — only scans if disconnected cameras exist.
 * Safe to call from any context where start_scan() would be called.
 * -------------------------------------------------------------------------- */
void start_scan_if_needed(void)
{
    if (g_ble_core_cbs.has_disconnected_cameras &&
        !g_ble_core_cbs.has_disconnected_cameras()) {
        ESP_LOGI(TAG, "All cameras connected — background scan suppressed");
        return;
    }
    start_scan();
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void ble_core_start_discovery(void)
{
    /* Safe to call from any task — posts an event to the NimBLE host task */
    ESP_LOGI(TAG, "Discovery mode requested");
    ble_npl_event_init(&s_start_disc_event, start_discovery_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_start_disc_event);
}

void ble_core_stop_discovery(void)
{
    ble_gap_disc_cancel();
    start_scan_if_needed();
}

/* --------------------------------------------------------------------------
 * Direct connect — runs on the NimBLE host task.
 *
 * Cancels any running scan and calls ble_gap_connect() immediately, putting
 * the controller into initiating mode.  No advertisement needs to be seen
 * first; the controller will connect as soon as the peer starts advertising.
 * -------------------------------------------------------------------------- */
static void connect_by_addr_cb(struct ble_npl_event *ev)
{
    if (s_connecting) {
        ESP_LOGW(TAG, "connect_by_addr: connection already in progress — ignoring");
        return;
    }

    ble_gap_disc_cancel(); /* stop any running scan; no-op if none active */

    const uint8_t *a = s_connect_target.val;
    ESP_LOGI(TAG, "Direct connect — %02X:%02X:%02X:%02X:%02X:%02X",
             a[5], a[4], a[3], a[2], a[1], a[0]);

    s_connecting = true;
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &s_connect_target,
                             BLE_HS_FOREVER, NULL, connection_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        s_connecting = false;
        start_scan_if_needed();
    }
}

void ble_core_connect_by_addr(const ble_addr_t *addr)
{
    /* Safe to call from any task — posts an event to the NimBLE host task,
     * which cancels the current scan and initiates a direct connection. */
    s_connect_target = *addr;
    ble_npl_event_init(&s_connect_by_addr_event, connect_by_addr_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_connect_by_addr_event);
}

/* --------------------------------------------------------------------------
 * Scan event callback — runs on the NimBLE host task.
 *
 * Responsibilities:
 *   1. Fire on_disc callback for every parsed advertisement.
 *   2. Safety net — reconnect a known camera seen advertising after a
 *      post-disconnect ble_gap_connect() attempt timed out.
 *
 * User-initiated pairings are handled by connect_by_addr_cb() via the event
 * queue and do not go through this path.
 * -------------------------------------------------------------------------- */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        ESP_LOGI(TAG, "Discovery scan complete");
        start_scan_if_needed();
        return 0;
    }

    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    struct ble_hs_adv_fields fields;
    int rc = ble_hs_adv_parse_fields(&fields,
                                     event->disc.data,
                                     event->disc.length_data);
    if (rc != 0) {
        return 0;
    }

    /* Fire the discovery callback for every advertisement that parses OK.
     * Higher layers (gopro_ble) apply their own UUID filters. */
    if (g_ble_core_cbs.on_disc) {
        g_ble_core_cbs.on_disc(&event->disc, &fields);
    }

    const uint8_t *a = event->disc.addr.val;

    /* Safety net: reconnect a known camera seen advertising during the
     * background scan (e.g. after a post-disconnect ble_gap_connect() timed
     * out and scanning resumed).  User-initiated pairings go through
     * connect_by_addr_cb() via the event queue and never reach this path. */
    if (!g_ble_core_cbs.is_known_addr ||
        !g_ble_core_cbs.is_known_addr(&event->disc.addr)) {
        return 0; /* unknown camera — skip */
    }

    /* --- Connect --- */
    if (s_connecting) {
        return 0;
    }

    /* Skip if already connected to this address */
    struct ble_gap_conn_desc existing;
    if (ble_gap_conn_find_by_addr(&event->disc.addr, &existing) == 0) {
        ESP_LOGD(TAG, "Already connected to %02X:%02X:%02X:%02X:%02X:%02X — skipping",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        return 0;
    }

    s_connecting = true;

    ESP_LOGI(TAG, "Connecting — %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm",
             a[5], a[4], a[3], a[2], a[1], a[0], event->disc.rssi);

    ble_gap_disc_cancel();

    int rc2 = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                              BLE_HS_FOREVER, NULL,
                              connection_event_cb, NULL);
    if (rc2 != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc2);
        s_connecting = false;
        start_scan();
    }

    return 0;
}
