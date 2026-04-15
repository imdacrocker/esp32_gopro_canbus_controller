#include "ble_core_internal.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"

static const char *TAG = "ble_core";

/* Targeted connect state — set from any task, acted on by scan_event_cb */
static volatile bool s_connect_requested = false;
static ble_addr_t    s_connect_addr;

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
        .filter_duplicates = 1,
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

/* --------------------------------------------------------------------------
 * Discovery scan — passive, runs for 30 seconds, no deduplication so every
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

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 30000,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc (discovery) failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Discovery scan started — 30 seconds");
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

void ble_core_connect_by_addr(const ble_addr_t *addr)
{
    /* Safe to call from any task — only sets flags.
     * scan_event_cb sees the target camera and performs the actual connect. */
    memcpy(&s_connect_addr, addr, sizeof(ble_addr_t));
    s_connect_requested = true;
}

/* --------------------------------------------------------------------------
 * Scan event callback — runs on the NimBLE host task.
 *
 * All BLE API calls happen here; other tasks only set flags.
 *
 * Responsibilities:
 *   1. Fire on_disc callback for every parsed advertisement.
 *   2. Targeted connect — connect to the explicitly requested address.
 *   3. Safety net — reconnect a known camera that is seen advertising after a
 *      post-disconnect ble_gap_connect() attempt timed out.
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

    /* --- Targeted connect: only connect to the explicitly requested camera --- */
    if (s_connect_requested) {
        if (memcmp(a, s_connect_addr.val, 6) != 0) {
            return 0; /* not our target */
        }
        s_connect_requested = false;
        /* fall through to connect logic */
    } else {
        /* Safety net: known camera advertising after a reconnect attempt.
         * Use the registered is_known_addr callback to avoid ble_core depending
         * on gopro_ble or camera_manager directly. */
        if (!g_ble_core_cbs.is_known_addr ||
            !g_ble_core_cbs.is_known_addr(&event->disc.addr)) {
            return 0; /* unknown camera — ignore */
        }
        /* fall through to connect logic */
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
