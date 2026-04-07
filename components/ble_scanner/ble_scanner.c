#include "ble_scanner.h"

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_gap.h"
#include "host/ble_store.h"

static const char *TAG = "BLE_SCANNER";

/* Forward declarations */
static int  scan_event_cb(struct ble_gap_event *event, void *arg);
static int  connection_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void ble_host_task(void *param);
static void start_discovery_cb(struct ble_npl_event *ev);

/* Connection state */
static bool s_connecting = false;

/* Discovery mode — set from HTTP task via event queue, read from scan_event_cb */
static volatile bool      s_discovery_mode   = false;
static gopro_device_t     s_discovered[GOPRO_MAX_DISCOVERED];
static int                s_discovered_count = 0;
static struct ble_npl_event s_start_disc_event;

/* Targeted connect — set from HTTP task, acted on by scan_event_cb */
static volatile bool s_connect_requested = false;
static ble_addr_t    s_connect_addr;


/* Bond purge — keep list populated by caller before posting the event */
static struct {
    ble_addr_t keep[CONFIG_BT_NIMBLE_MAX_BONDS];
    int        keep_count;
} s_purge_ctx;

static struct ble_npl_event s_purge_event;

/* --------------------------------------------------------------------------
 * Bond count helper — iterates peer security store and returns the total.
 * Must be called from the NimBLE host task.
 * -------------------------------------------------------------------------- */
static int count_bonds_cb(int obj_type, union ble_store_value *val, void *cookie)
{
    (*(int *)cookie)++;
    return 0;
}

static void log_bond_count(void)
{
    int count = 0;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, count_bonds_cb, &count);
    ESP_LOGI(TAG, "Stored bonds: %d / %d", count, CONFIG_BT_NIMBLE_MAX_BONDS);
}

/* --------------------------------------------------------------------------
 * Bond purge callback — runs on the NimBLE host task.
 * Deletes any peer security entry not present in s_purge_ctx.keep[].
 * -------------------------------------------------------------------------- */

/* Addresses collected during iteration; deleted after the walk completes. */
static ble_addr_t s_delete_list[CONFIG_BT_NIMBLE_MAX_BONDS];
static int        s_delete_count;

static int collect_purge_cb(int obj_type, union ble_store_value *val, void *cookie)
{
    ble_addr_t *addr = &val->sec.peer_addr;

    for (int i = 0; i < s_purge_ctx.keep_count; i++) {
        if (memcmp(&s_purge_ctx.keep[i], addr, sizeof(ble_addr_t)) == 0) {
            return 0; /* on the keep list */
        }
    }

    if (s_delete_count < (int)(sizeof(s_delete_list) / sizeof(s_delete_list[0]))) {
        s_delete_list[s_delete_count++] = *addr;
    }
    return 0;
}

static void purge_bonds_cb(struct ble_npl_event *ev)
{
    log_bond_count();

    s_delete_count = 0;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, collect_purge_cb, NULL);

    for (int i = 0; i < s_delete_count; i++) {
        const uint8_t *a = s_delete_list[i].val;
        ESP_LOGI(TAG, "Removing bond: %02X:%02X:%02X:%02X:%02X:%02X",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        ble_store_util_delete_peer(&s_delete_list[i]);
    }

    if (s_delete_count > 0) {
        ESP_LOGI(TAG, "Purged %d bond(s)", s_delete_count);
        log_bond_count();
    }
}

/* --------------------------------------------------------------------------
 * NimBLE host-task callback — cancels the background scan and restarts it
 * with a 30-second timeout in discovery mode.
 * Runs on the NimBLE host task so NimBLE APIs are safe to call directly.
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

    s_discovery_mode = true;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 30000,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc (discovery) failed: %d", rc);
        s_discovery_mode = false;
    } else {
        ESP_LOGI(TAG, "Discovery scan started — 30 seconds");
    }
}

/* --------------------------------------------------------------------------
 * NimBLE host task
 * -------------------------------------------------------------------------- */
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* --------------------------------------------------------------------------
 * Sync callback — BLE stack is ready
 * -------------------------------------------------------------------------- */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack ready!");
    log_bond_count();
    // start_scan();
}

/* --------------------------------------------------------------------------
 * Reset callback
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
        .itvl              = 0,
        .window            = 0,
        .filter_policy     = 0,
        .limited           = 0,
        .passive           = 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

/* --------------------------------------------------------------------------
 * Connection event callback
 * -------------------------------------------------------------------------- */
static int connection_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            uint16_t handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected — handle: %d", handle);

            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(handle, &desc);

            struct ble_store_key_sec key;
            struct ble_store_value_sec sec;
            memset(&key, 0, sizeof(key));

            key.peer_addr = desc.peer_id_addr;
            bool bonded = (ble_store_read_peer_sec(&key, &sec) == 0);
            if (!bonded) {
                key.peer_addr = desc.peer_ota_addr;
                bonded = (ble_store_read_peer_sec(&key, &sec) == 0);
            }

            if (bonded) {
                ESP_LOGI(TAG, "Known camera — restoring encryption with saved keys");
            } else {
                ESP_LOGI(TAG, "New camera — initiating first-time pairing");
            }

            int sec_rc = ble_gap_security_initiate(handle);
            if (sec_rc != 0) {
                ESP_LOGE(TAG, "Security initiate failed: %d", sec_rc);
            }

            /* Reset connecting flag and restart scan so that:
             * - discovery mode can find more cameras while connected
             * - additional cameras can be auto-connected later
             * Connected cameras stop advertising, so we won't re-connect them. */
            s_connecting = false;
            start_scan();
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

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        ESP_LOGW(TAG, "Repeat pairing — deleting stale bond and re-pairing");
        struct ble_gap_conn_desc rp_desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &rp_desc);
        ble_store_util_delete_peer(&rp_desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected — reason: %d", event->disconnect.reason);
        s_connecting = false;
        start_scan();
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGW(TAG, "Passkey action requested — action: %d",
                event->passkey.params.action);
        break;

    default:
        break;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Scan event callback — runs on the NimBLE host task.
 * All BLE API calls stay here; the HTTP task only sets flags.
 * -------------------------------------------------------------------------- */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        s_discovery_mode = false;
        ESP_LOGI(TAG, "Discovery scan complete — %d camera(s) found", s_discovered_count);
        start_scan(); /* restart background scan with no timeout */
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

    /* Filter: GoPro service UUID 0xFEA6 */
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

    const uint8_t *a = event->disc.addr.val;

    /* --- Discovery mode: collect cameras, do not connect --- */
    if (s_discovery_mode) {
        for (int i = 0; i < s_discovered_count; i++) {
            if (memcmp(s_discovered[i].addr.val, a, 6) == 0) {
                return 0; /* already listed */
            }
        }
        if (s_discovered_count < GOPRO_MAX_DISCOVERED) {
            gopro_device_t *d = &s_discovered[s_discovered_count++];
            memcpy(&d->addr, &event->disc.addr, sizeof(ble_addr_t));
            d->rssi = event->disc.rssi;
            if (fields.name != NULL && fields.name_len > 0) {
                int len = fields.name_len < (int)(sizeof(d->name) - 1)
                        ? fields.name_len : (int)(sizeof(d->name) - 1);
                memcpy(d->name, fields.name, len);
                d->name[len] = '\0';
            } else {
                snprintf(d->name, sizeof(d->name), "GoPro %02X%02X", a[1], a[0]);
            }
            ESP_LOGI(TAG, "Discovered: %s  %02X:%02X:%02X:%02X:%02X:%02X",
                     d->name, a[5], a[4], a[3], a[2], a[1], a[0]);
        }
        return 0;
    }

    /* --- Targeted connect: only connect to the explicitly requested camera --- */
    if (s_connect_requested) {
        if (memcmp(a, s_connect_addr.val, 6) != 0) {
            return 0; /* not our target */
        }
        s_connect_requested = false;
        /* fall through to connect logic below */
    } else {
        return 0; /* not in discovery mode and no pair request — do not auto-connect */
    }

    /* --- Normal / targeted connect --- */
    if (s_connecting) {
        return 0;
    }

    /* Skip if already connected to this address */
    struct ble_gap_conn_desc existing;
    if (ble_gap_conn_find_by_addr(&event->disc.addr, &existing) == 0) {
        ESP_LOGI(TAG, "Already connected to %02X:%02X:%02X:%02X:%02X:%02X — skipping",
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

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */
void ble_init(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_device_name_set("ESP32 Controller");

    nimble_port_freertos_init(ble_host_task);
}

void ble_scanner_start_discovery(void)
{
    /* Safe to call from any task — posts an event to the NimBLE host task.
     * start_discovery_cb() then calls the actual NimBLE APIs on the correct task. */
    ESP_LOGI(TAG, "Discovery mode requested");
    s_discovered_count = 0;

    ble_npl_event_init(&s_start_disc_event, start_discovery_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_start_disc_event);
}

int ble_scanner_get_discovered(gopro_device_t *out, int max_count)
{
    int n = s_discovered_count < max_count ? s_discovered_count : max_count;
    memcpy(out, s_discovered, n * sizeof(gopro_device_t));
    return n;
}

void ble_scanner_connect_by_addr(const ble_addr_t *addr)
{
    /* Safe to call from any task — only sets flags.
     * scan_event_cb sees the target camera and does the actual connect. */
    s_discovery_mode = false;
    memcpy(&s_connect_addr, addr, sizeof(ble_addr_t));
    s_connect_requested = true;
}

void ble_scanner_purge_unknown_bonds(const ble_addr_t *keep, int keep_count)
{
    int n = keep_count < CONFIG_BT_NIMBLE_MAX_BONDS ? keep_count : CONFIG_BT_NIMBLE_MAX_BONDS;
    memset(&s_purge_ctx, 0, sizeof(s_purge_ctx));
    for (int i = 0; i < n; i++) {
        s_purge_ctx.keep[i] = keep[i];
    }
    s_purge_ctx.keep_count = n;

    ble_npl_event_init(&s_purge_event, purge_bonds_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_purge_event);
}
