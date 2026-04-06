#include "ble_scanner.h"

#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

static const char *TAG = "BLE_SCANNER";

/* Forward declarations */
static int  scan_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void ble_host_task(void *param);

/* --------------------------------------------------------------------------
 * NimBLE host task - runs the BLE stack on its own FreeRTOS task
 * -------------------------------------------------------------------------- */
static void ble_host_task(void *param)
{
    nimble_port_run();              /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* --------------------------------------------------------------------------
 * Sync callback - called when the BLE host and controller are ready
 * -------------------------------------------------------------------------- */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack ready, starting scan...");
    start_scan();
}

/* --------------------------------------------------------------------------
 * Reset callback - called if the BLE stack crashes or resets
 * -------------------------------------------------------------------------- */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE stack reset (reason: %d)", reason);
}

/* --------------------------------------------------------------------------
 * Start a passive scan that runs forever
 * -------------------------------------------------------------------------- */
static void start_scan(void)
{
    struct ble_gap_disc_params disc_params = {
        .itvl             = 0,  /* default: 100 ms */
        .window           = 0,  /* default:  50 ms */
        .filter_policy    = 0,  /* accept all advertisers */
        .limited          = 0,  /* general (not limited) discovery */
        .passive          = 1,  /* passive — no scan requests sent */
        .filter_duplicates = 0, /* report every advertisement packet */
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

/* --------------------------------------------------------------------------
 * Scan event callback - called for each received advertisement
 * -------------------------------------------------------------------------- */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    const uint8_t *a = event->disc.addr.val;

    /* NimBLE stores address bytes little-endian (byte 0 = LSB).
     * Print MSB-first so it matches what you see in phone BLE scanners. */
    ESP_LOGI(TAG, "ADDR: %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm",
             a[5], a[4], a[3], a[2], a[1], a[0],
             event->disc.rssi);

    return 0;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */
void ble_scanner_init(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    nimble_port_freertos_init(ble_host_task);
}

void ble_scanner_start(void)
{
    /* Scanning begins automatically once the stack calls on_sync().
     * Nothing to do here yet — this function exists so main.c has
     * a clear place to trigger scanning in the future. */
    ESP_LOGI(TAG, "Waiting for BLE stack sync...");
}
