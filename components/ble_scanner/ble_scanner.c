#include "ble_scanner.h"

#include <stdbool.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

static const char *TAG = "BLE_SCANNER";

/* Forward declarations */
static int  scan_event_cb(struct ble_gap_event *event, void *arg);
static int  connection_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void ble_host_task(void *param);

/* True while a connection attempt is in progress — prevents duplicate connects */
static bool s_connecting = false;

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
 * Connection event callback - called for connect, pairing, and disconnect
 * -------------------------------------------------------------------------- */
static int connection_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected — handle: %d", event->connect.conn_handle);
            /* Trigger pairing — this prompts the user to accept on the camera */
            ble_gap_security_initiate(event->connect.conn_handle);
        } else {
            ESP_LOGE(TAG, "Connection failed — status: %d", event->connect.status);
            s_connecting = false;
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Paired successfully — handle: %d", event->enc_change.conn_handle);
        } else {
            ESP_LOGE(TAG, "Pairing failed — status: %d", event->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected — reason: %d", event->disconnect.reason);
        s_connecting = false;
        start_scan();
        break;

    default:
        break;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Scan event callback - called for each received advertisement
 * -------------------------------------------------------------------------- */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    /* Parse the advertisement payload */
    struct ble_hs_adv_fields fields;
    int rc = ble_hs_adv_parse_fields(&fields,
                                     event->disc.data,
                                     event->disc.length_data);
    if (rc != 0) {
        return 0;
    }

    /* Filter: only process devices advertising GoPro service UUID 0xFEA6 */
    bool is_gopro = false;
    for (int i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == 0xFEA6) {
            is_gopro = true;
            break;
        }
    }
    if (!is_gopro) {
        return 0;
    }

    /* Prevent duplicate connection attempts if multiple ad packets arrive */
    if (s_connecting) {
        return 0;
    }
    s_connecting = true;

    const uint8_t *a = event->disc.addr.val;
    ESP_LOGI(TAG, "GoPro found — ADDR: %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm",
             a[5], a[4], a[3], a[2], a[1], a[0],
             event->disc.rssi);

    /* Stop scanning and connect */
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
