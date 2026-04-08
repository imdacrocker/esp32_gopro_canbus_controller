#include "ble_scanner.h"
#include "gopro_manager.h"

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
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
static void reconnect_next(void);

/* -------------------------------------------------------------------------
 * GoPro 128-bit UUID definitions
 *
 * Base pattern: b5f9XXXX-aa8d-11e3-9046-0002a5d5c51b
 *
 * NimBLE stores 128-bit UUIDs in little-endian byte order.
 * For b5f9XXXX-aa8d-11e3-9046-0002a5d5c51b the LE byte array is:
 *   [0] = 0x1b  [1] = 0xc5  [2] = 0xd5  [3] = 0xa5
 *   [4] = 0x02  [5] = 0x00  [6] = 0x46  [7] = 0x90
 *   [8] = 0xe3  [9] = 0x11  [10]= 0x8d  [11]= 0xaa
 *   [12]= lo(XXXX) [13]= hi(XXXX) [14]= 0xf9 [15]= 0xb5
 * ------------------------------------------------------------------------- */
#define GP_UUID128_DECLARE(xxxx)                                           \
    BLE_UUID128_INIT(0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x46, 0x90,    \
                     0xe3, 0x11, 0x8d, 0xaa,                               \
                     (uint8_t)((xxxx) & 0xFF),                             \
                     (uint8_t)(((xxxx) >> 8) & 0xFF),                      \
                     0xf9, 0xb5)

static const ble_uuid128_t s_gp_uuid_cmd            = GP_UUID128_DECLARE(0x0072);
static const ble_uuid128_t s_gp_uuid_cmd_resp       = GP_UUID128_DECLARE(0x0073);
static const ble_uuid128_t s_gp_uuid_settings       = GP_UUID128_DECLARE(0x0074);
static const ble_uuid128_t s_gp_uuid_settings_resp  = GP_UUID128_DECLARE(0x0075);
static const ble_uuid128_t s_gp_uuid_query          = GP_UUID128_DECLARE(0x0076);
static const ble_uuid128_t s_gp_uuid_query_resp     = GP_UUID128_DECLARE(0x0077);
static const ble_uuid128_t s_gp_uuid_net_mgmt_cmd   = GP_UUID128_DECLARE(0x0091);
static const ble_uuid128_t s_gp_uuid_net_mgmt_resp  = GP_UUID128_DECLARE(0x0092);
static const ble_uuid128_t s_gp_uuid_wifi_ssid      = GP_UUID128_DECLARE(0x0002);
static const ble_uuid128_t s_gp_uuid_wifi_pass      = GP_UUID128_DECLARE(0x0003);
static const ble_uuid128_t s_gp_uuid_wifi_power     = GP_UUID128_DECLARE(0x0004);
static const ble_uuid128_t s_gp_uuid_wifi_state     = GP_UUID128_DECLARE(0x0005);

/* -------------------------------------------------------------------------
 * GATT discovery context
 *
 * One context per active connection.  Tracks three sequential phases:
 *   1. Collect all services via ble_gattc_disc_all_svcs()
 *   2. For each service, collect notifiable characteristics
 *   3. Write 0x0001/0x0002 to each characteristic's CCCD (val_handle + 1)
 *
 * All GATT callbacks run on the NimBLE host task so no locking is needed.
 * ------------------------------------------------------------------------- */
#define GATT_MAX_SERVICES    16
#define GATT_MAX_NOTIFY_CHRS 16

typedef struct {
    uint16_t conn_handle;
    bool     active;

    /* Phase 1 – collected services */
    struct {
        uint16_t start_handle;
        uint16_t end_handle;
    } svcs[GATT_MAX_SERVICES];
    int svc_count;
    int svc_idx;          /* next service to run char discovery on */

    /* Phase 2/3 – notifiable characteristics */
    struct {
        uint16_t val_handle;
        uint16_t cccd_val;  /* 0x0001 = notify, 0x0002 = indicate */
    } notify_chrs[GATT_MAX_NOTIFY_CHRS];
    int notify_count;
    int notify_idx;       /* next characteristic to subscribe to */

    /* Accumulated handles for gopro_manager */
    gopro_gatt_handles_t handles;
} gatt_disc_ctx_t;

static gatt_disc_ctx_t s_disc_ctx[CONFIG_BT_NIMBLE_MAX_BONDS];

static gatt_disc_ctx_t *find_disc_ctx(uint16_t conn_handle)
{
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_disc_ctx[i].active && s_disc_ctx[i].conn_handle == conn_handle) {
            return &s_disc_ctx[i];
        }
    }
    return NULL;
}

static gatt_disc_ctx_t *alloc_disc_ctx(uint16_t conn_handle)
{
    /* Reuse an existing slot for this handle, or find an empty one */
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (!s_disc_ctx[i].active ||
            s_disc_ctx[i].conn_handle == conn_handle) {
            memset(&s_disc_ctx[i], 0, sizeof(s_disc_ctx[i]));
            s_disc_ctx[i].conn_handle = conn_handle;
            s_disc_ctx[i].active      = true;
            return &s_disc_ctx[i];
        }
    }
    return NULL;
}

static void free_disc_ctx(uint16_t conn_handle)
{
    gatt_disc_ctx_t *ctx = find_disc_ctx(conn_handle);
    if (ctx) {
        ctx->active = false;
    }
}

/* Forward declarations for discovery chain */
static void start_char_discovery(gatt_disc_ctx_t *ctx);
static void start_cccd_subscriptions(gatt_disc_ctx_t *ctx);

/* -------------------------------------------------------------------------
 * Identify a characteristic UUID and store its handle
 * ------------------------------------------------------------------------- */
static void record_chr_handle(gatt_disc_ctx_t *ctx, const struct ble_gatt_chr *chr)
{
    const ble_uuid_t *u = &chr->uuid.u;

#define MATCH(gpuuid, field) \
    if (ble_uuid_cmp(u, &(gpuuid).u) == 0) { ctx->handles.field = chr->val_handle; return; }

    MATCH(s_gp_uuid_cmd,           cmd_write)
    MATCH(s_gp_uuid_cmd_resp,      cmd_resp_notify)
    MATCH(s_gp_uuid_settings,      settings_write)
    MATCH(s_gp_uuid_settings_resp, settings_resp_notify)
    MATCH(s_gp_uuid_query,         query_write)
    MATCH(s_gp_uuid_query_resp,    query_resp_notify)
    MATCH(s_gp_uuid_net_mgmt_cmd,  net_mgmt_cmd_write)
    MATCH(s_gp_uuid_net_mgmt_resp, net_mgmt_resp_notify)
    MATCH(s_gp_uuid_wifi_ssid,     wifi_ssid_read)
    MATCH(s_gp_uuid_wifi_pass,     wifi_pass_read)
    MATCH(s_gp_uuid_wifi_power,    wifi_power_write)
    MATCH(s_gp_uuid_wifi_state,    wifi_state_indicate)
#undef MATCH
}

/* -------------------------------------------------------------------------
 * Phase 3 – CCCD write callback
 * Called after each individual CCCD write completes (or errors).
 * Advances to the next subscription, or finalises when all are done.
 * ------------------------------------------------------------------------- */
static int cccd_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    gatt_disc_ctx_t *ctx = find_disc_ctx(conn_handle);
    if (!ctx) {
        return 0;
    }

    if (error->status != 0) {
        uint16_t h = attr ? attr->handle : 0;
        ESP_LOGW(TAG, "CCCD write error — handle 0x%04x status %d", h, error->status);
        /* Non-fatal: log and continue to the next characteristic */
    } else {
        uint16_t h = attr ? attr->handle : 0;
        ESP_LOGI(TAG, "  Subscribed CCCD handle 0x%04x", h);
    }

    ctx->notify_idx++;

    if (ctx->notify_idx < ctx->notify_count) {
        /* Write the next CCCD */
        uint16_t val_h  = ctx->notify_chrs[ctx->notify_idx].val_handle;
        uint16_t cccd_h = val_h + 1;
        uint16_t cccd_v = ctx->notify_chrs[ctx->notify_idx].cccd_val;
        int rc = ble_gattc_write_flat(conn_handle, cccd_h,
                                      &cccd_v, sizeof(cccd_v),
                                      cccd_write_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "CCCD write failed rc=%d — skipping", rc);
            /* Simulate a completed callback to keep the chain moving */
            struct ble_gatt_error fake_err = { .status = rc };
            cccd_write_cb(conn_handle, &fake_err, NULL, NULL);
        }
    } else {
        /* All subscriptions done — push handles to gopro_manager */
        int slot = gopro_manager_find_by_handle(conn_handle);
        if (slot >= 0) {
            gopro_manager_set_gatt_handles(slot, &ctx->handles);
            gopro_manager_set_gatt_ready(slot, true);
            ESP_LOGI(TAG, "GATT setup complete for slot %d (%d notification(s))",
                     slot, ctx->notify_count);
        } else {
            ESP_LOGW(TAG, "GATT setup complete but camera slot not found (handle %d)",
                     conn_handle);
        }
        free_disc_ctx(conn_handle);
    }

    return 0;
}

static void start_cccd_subscriptions(gatt_disc_ctx_t *ctx)
{
    if (ctx->notify_count == 0) {
        ESP_LOGW(TAG, "No notifiable characteristics found for handle %d",
                 ctx->conn_handle);
        /* Still push any write/read handles we discovered */
        int slot = gopro_manager_find_by_handle(ctx->conn_handle);
        if (slot >= 0) {
            gopro_manager_set_gatt_handles(slot, &ctx->handles);
        }
        free_disc_ctx(ctx->conn_handle);
        return;
    }

    ESP_LOGI(TAG, "Subscribing to %d characteristic(s)...", ctx->notify_count);
    ctx->notify_idx = 0;

    uint16_t val_h  = ctx->notify_chrs[0].val_handle;
    uint16_t cccd_h = val_h + 1;
    uint16_t cccd_v = ctx->notify_chrs[0].cccd_val;

    int rc = ble_gattc_write_flat(ctx->conn_handle, cccd_h,
                                  &cccd_v, sizeof(cccd_v),
                                  cccd_write_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "First CCCD write failed rc=%d", rc);
        free_disc_ctx(ctx->conn_handle);
    }
}

/* -------------------------------------------------------------------------
 * Phase 2 – characteristic discovery callback
 * Called once per characteristic within a service, then once with EDONE.
 * ------------------------------------------------------------------------- */
static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    gatt_disc_ctx_t *ctx = find_disc_ctx(conn_handle);
    if (!ctx) {
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        /* Move to next service */
        ctx->svc_idx++;
        start_char_discovery(ctx);
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Chr discovery error svc_idx=%d status=%d",
                 ctx->svc_idx, error->status);
        ctx->svc_idx++;
        start_char_discovery(ctx);
        return 0;
    }

    /* Record the known handle for this characteristic */
    record_chr_handle(ctx, chr);

    /* Collect notify/indicate characteristics for CCCD subscription.
     * Skip standard 16-bit BLE characteristics (e.g. Service Changed 0x2a05,
     * Battery Level 0x2a19) — those are not GoPro data channels and several
     * don't have a writable CCCD, which would produce spurious warnings. */
    bool has_notify   = (chr->properties & BLE_GATT_CHR_F_NOTIFY)   != 0;
    bool has_indicate = (chr->properties & BLE_GATT_CHR_F_INDICATE)  != 0;
    bool is_std_16bit = (chr->uuid.u.type == BLE_UUID_TYPE_16);

    if ((has_notify || has_indicate) && !is_std_16bit &&
        ctx->notify_count < GATT_MAX_NOTIFY_CHRS) {

        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&chr->uuid.u, uuid_str);
        ESP_LOGI(TAG, "  [%s] val_handle=0x%04x uuid=%s",
                 has_notify ? "NOTIFY" : "INDICATE",
                 chr->val_handle, uuid_str);

        ctx->notify_chrs[ctx->notify_count].val_handle = chr->val_handle;
        ctx->notify_chrs[ctx->notify_count].cccd_val   =
            has_notify ? 0x0001 : 0x0002;
        ctx->notify_count++;
    }

    return 0;
}

static void start_char_discovery(gatt_disc_ctx_t *ctx)
{
    if (ctx->svc_idx >= ctx->svc_count) {
        /* All services processed — begin CCCD subscription phase */
        ESP_LOGI(TAG, "All services scanned (%d notify chr(s) collected)",
                 ctx->notify_count);
        start_cccd_subscriptions(ctx);
        return;
    }

    uint16_t start = ctx->svcs[ctx->svc_idx].start_handle;
    uint16_t end   = ctx->svcs[ctx->svc_idx].end_handle;

    int rc = ble_gattc_disc_all_chrs(ctx->conn_handle, start, end,
                                     chr_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "disc_all_chrs failed svc_idx=%d rc=%d — skipping",
                 ctx->svc_idx, rc);
        ctx->svc_idx++;
        start_char_discovery(ctx);
    }
}

/* -------------------------------------------------------------------------
 * Phase 1 – service discovery callback
 * Called once per service, then once with EDONE.
 * ------------------------------------------------------------------------- */
static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    gatt_disc_ctx_t *ctx = find_disc_ctx(conn_handle);
    if (!ctx) {
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery done — %d service(s) found", ctx->svc_count);
        start_char_discovery(ctx);
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
        free_disc_ctx(conn_handle);
        return 0;
    }

    if (ctx->svc_count < GATT_MAX_SERVICES) {
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&svc->uuid.u, uuid_str);
        ESP_LOGI(TAG, "  Service start=0x%04x end=0x%04x uuid=%s",
                 svc->start_handle, svc->end_handle, uuid_str);

        ctx->svcs[ctx->svc_count].start_handle = svc->start_handle;
        ctx->svcs[ctx->svc_count].end_handle   = svc->end_handle;
        ctx->svc_count++;
    }

    return 0;
}

/* -------------------------------------------------------------------------
 * Entry point — kick off GATT discovery after encryption is established.
 * Called from connection_event_cb on BLE_GAP_EVENT_ENC_CHANGE success.
 * ------------------------------------------------------------------------- */
static void start_gatt_discovery(uint16_t conn_handle)
{
    ESP_LOGI(TAG, "Starting GATT discovery for handle %d", conn_handle);

    gatt_disc_ctx_t *ctx = alloc_disc_ctx(conn_handle);
    if (!ctx) {
        ESP_LOGE(TAG, "No free discovery context slots");
        return;
    }

    int rc = ble_gattc_disc_all_svcs(conn_handle, svc_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
        free_disc_ctx(conn_handle);
    }
}

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

/* Boot reconnect — peers collected from bond store in on_sync, worked off one
 * at a time via reconnect_next().  No manual whitelist needed. */
static ble_addr_t s_pending_reconnect[CONFIG_BT_NIMBLE_MAX_BONDS];
static int        s_pending_count = 0;
static int        s_pending_idx   = 0;

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
 * Boot reconnect helpers
 *
 * collect_bonded_peers_cb() walks the NimBLE peer-security store and builds
 * a deduplicated list of peer addresses to reconnect to on startup.
 *
 * reconnect_next() works off that list one at a time, calling ble_gap_connect()
 * directly — no scan phase required.  When all known peers have been attempted
 * the function falls through to start_scan() for new-camera discovery.
 * -------------------------------------------------------------------------- */
static int collect_bonded_peers_cb(int obj_type, union ble_store_value *val, void *cookie)
{
    if (s_pending_count >= CONFIG_BT_NIMBLE_MAX_BONDS) {
        return 0;
    }

    /* The bond store can hold both a master-sec and a peer-sec entry for the
     * same device.  Deduplicate by address before adding to the list. */
    for (int i = 0; i < s_pending_count; i++) {
        if (memcmp(&s_pending_reconnect[i], &val->sec.peer_addr,
                   sizeof(ble_addr_t)) == 0) {
            return 0;
        }
    }

    s_pending_reconnect[s_pending_count++] = val->sec.peer_addr;
    return 0;
}

static void reconnect_next(void)
{
    if (s_pending_idx >= s_pending_count) {
        /* All known peers have been attempted — start background scan so that
         * new cameras can be discovered and any missed reconnects can be
         * retried via gopro_manager_find_by_addr(). */
        ESP_LOGI(TAG, "Boot reconnect phase complete — starting background scan");
        start_scan();
        return;
    }

    ble_addr_t *addr = &s_pending_reconnect[s_pending_idx++];
    const uint8_t *a = addr->val;

    /* Skip if this peer is already connected (shouldn't happen on boot, but
     * be safe in case reconnect_next() is called more than once). */
    struct ble_gap_conn_desc existing;
    if (ble_gap_conn_find_by_addr(addr, &existing) == 0) {
        ESP_LOGI(TAG, "Already connected to %02X:%02X:%02X:%02X:%02X:%02X — skipping",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        reconnect_next();
        return;
    }

    ESP_LOGI(TAG, "Boot reconnect %d/%d — %02X:%02X:%02X:%02X:%02X:%02X",
             s_pending_idx, s_pending_count,
             a[5], a[4], a[3], a[2], a[1], a[0]);

    /* Connect directly — no scan required.  The BLE controller enters
     * initiating state and connects as soon as this peer starts advertising.
     * BLE_HS_FOREVER: waits indefinitely so a camera that is still booting
     * when the ESP32 powers on is not skipped.  reconnect_next() is only
     * called from connection_event_cb on success or explicit failure. */
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, addr,
                             BLE_HS_FOREVER, NULL, connection_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d — skipping", rc);
        reconnect_next();
    }
}

/* --------------------------------------------------------------------------
 * Sync callback — BLE stack is ready
 * -------------------------------------------------------------------------- */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack ready!");
    log_bond_count();

    /* Collect all bonded peers and kick off the reconnect chain.
     * ble_gap_connect() is used directly — no scan phase needed. */
    s_pending_count = 0;
    s_pending_idx   = 0;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, collect_bonded_peers_cb, NULL);

    if (s_pending_count > 0) {
        ESP_LOGI(TAG, "Found %d bonded peer(s) — starting reconnect chain",
                 s_pending_count);
        reconnect_next();
    } else {
        ESP_LOGI(TAG, "No bonded peers — starting background scan");
        start_scan();
    }
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
        /* Deduplicate at the controller level so the host task isn't called
         * for every advertisement packet from an already-seen device.  The
         * filter resets each scan interval, so we still catch cameras that
         * come back online. */
        .filter_duplicates = 1,
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

            /* Update runtime state in gopro_manager */
            int cam_slot = gopro_manager_find_by_addr(&desc.peer_id_addr);
            if (cam_slot >= 0) {
                gopro_manager_set_connected(cam_slot, handle);
            }

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
        } else {
            ESP_LOGE(TAG, "Connection failed — status: %d", event->connect.status);
        }

        /* Advance the boot reconnect chain (or start background scan once all
         * known peers have been attempted).  Safe to call after the chain is
         * already complete — reconnect_next() becomes a no-op start_scan(). */
        s_connecting = false;
        reconnect_next();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            uint16_t enc_handle = event->enc_change.conn_handle;
            ESP_LOGI(TAG, "Encryption established — handle: %d", enc_handle);

            /* Ensure this camera has a slot in gopro_manager and that
             * set_connected has been called.
             *
             * Two scenarios reach here:
             *   A) Camera was already in gopro_manager NVS → find_by_addr()
             *      succeeded in BLE_GAP_EVENT_CONNECT and set_connected was
             *      called there.  Nothing extra needed.
             *   B) Camera is bonded in NimBLE but was never saved to
             *      gopro_manager (e.g. first boot with new firmware, or NVS
             *      was erased).  find_by_addr() returned -1 so set_connected
             *      was never called.  Create the slot now.
             */
            struct ble_gap_conn_desc enc_desc;
            if (ble_gap_conn_find(enc_handle, &enc_desc) == 0) {
                int slot = gopro_manager_find_by_addr(&enc_desc.peer_id_addr);

                if (slot < 0) {
                    /* Camera not yet in gopro_manager — register it */
                    slot = gopro_manager_find_free_slot();
                    if (slot >= 0) {
                        gopro_camera_t *cam = gopro_manager_get(slot);
                        cam->mac_address = enc_desc.peer_id_addr;
                        cam->is_paired   = true;
                        /* Placeholder name — overwritten when hardware info
                         * is read during GoPro-specific setup */
                        const uint8_t *a = enc_desc.peer_id_addr.val;
                        snprintf(cam->camera_name, GOPRO_NAME_LEN,
                                 "GoPro %02X%02X", a[1], a[0]);
                        gopro_manager_save(slot);
                        ESP_LOGI(TAG, "Registered new camera in slot %d (%s)",
                                 slot, cam->camera_name);
                    } else {
                        ESP_LOGE(TAG, "No free camera slots — cannot register camera");
                    }
                }

                /* Guarantee set_connected has been called regardless of path */
                if (slot >= 0) {
                    gopro_camera_t *cam = gopro_manager_get(slot);
                    if (cam->bt_handle == BLE_HS_CONN_HANDLE_NONE) {
                        gopro_manager_set_connected(slot, enc_handle);
                    }
                }
            }

            /* Discover all GATT services and subscribe to every notifiable
             * characteristic.  Per the OpenGoPro spec the GoPro camera does
             * not cache CCCD subscriptions, so this must run on every
             * connection — not just the first-time pairing. */
            start_gatt_discovery(enc_handle);
        } else {
            ESP_LOGE(TAG, "Pairing/encryption failed — status: %d",
                     event->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        ESP_LOGW(TAG, "Repeat pairing — deleting stale bond and re-pairing");
        struct ble_gap_conn_desc rp_desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &rp_desc);
        ble_store_util_delete_peer(&rp_desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGI(TAG, "Disconnected — reason: %d", event->disconnect.reason);
        gopro_manager_set_disconnected(event->disconnect.conn.conn_handle);
        free_disc_ctx(event->disconnect.conn.conn_handle);
        s_connecting = false;

        /* Cancel any active scan before calling ble_gap_connect().
         * ble_gap_connect() returns BLE_HS_EBUSY if a scan is running. */
        ble_gap_disc_cancel();

        /* Attempt a direct reconnect to the camera that just dropped.
         * BLE_HS_FOREVER: the controller stays in initiating state and connects
         * the moment the peer starts advertising, regardless of how long it
         * takes to power back on.  No timeout needed — the scan-based safety
         * net in scan_event_cb handles the rare case where this must be
         * abandoned (e.g. call ble_gap_connect_cancel() explicitly). */
        const ble_addr_t *peer = &event->disconnect.conn.peer_id_addr;
        const uint8_t *a = peer->val;
        ESP_LOGI(TAG, "Attempting reconnect to %02X:%02X:%02X:%02X:%02X:%02X",
                 a[5], a[4], a[3], a[2], a[1], a[0]);

        int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, peer,
                                 BLE_HS_FOREVER, NULL, connection_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Post-disconnect reconnect failed: %d — scanning", rc);
            start_scan();
        }
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t conn_h = event->notify_rx.conn_handle;
        uint16_t attr_h = event->notify_rx.attr_handle;

        int slot = gopro_manager_find_by_handle(conn_h);
        if (slot < 0) break;

        gopro_camera_t *cam = gopro_manager_get(slot);
        if (!cam || !cam->gatt_ready) break;

        if (attr_h == cam->gatt.query_resp_notify) {
            struct os_mbuf *om = event->notify_rx.om;
            uint16_t pkt_len  = OS_MBUF_PKTLEN(om);
            if (pkt_len > 0 && pkt_len <= 64) {
                uint8_t buf[64];
                os_mbuf_copydata(om, 0, pkt_len, buf);
                gopro_manager_handle_query_response(slot, buf, pkt_len);
            }
        }
        break;
    }

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
 *
 * During the background scan this callback handles two cases only:
 *   1. Discovery mode  — collect GoPros, do not connect.
 *   2. Targeted connect — connect to the explicitly requested address.
 *
 * Known-camera reconnects are handled via ble_gap_connect() directly (no
 * scan required), so there is no whitelist check here.  As a safety net,
 * if a known camera is seen advertising after a post-disconnect reconnect
 * attempt times out, gopro_manager_find_by_addr() catches it and reconnects.
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
        /* Safety net: a known camera is advertising, which means a post-disconnect
         * ble_gap_connect() attempt timed out and we fell back to scanning.
         * Reconnect it now. */
        if (gopro_manager_find_by_addr(&event->disc.addr) < 0) {
            return 0; /* unknown camera — ignore */
        }
        /* fall through to connect logic below */
    }

    /* --- Connect --- */
    if (s_connecting) {
        return 0;
    }

    /* Skip if already connected to this address */
    struct ble_gap_conn_desc existing;
    if (ble_gap_conn_find_by_addr(&event->disc.addr, &existing) == 0) {
        /* Camera is still advertising while connected — normal BLE behaviour.
         * Log at DEBUG level only so the serial output isn't flooded. */
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

/* --------------------------------------------------------------------------
 * GATT write helper — public API
 *
 * The NimBLE GATT client APIs must be called from the NimBLE host task.
 * This helper schedules the write on that task via the default event queue,
 * exactly as ble_scanner_start_discovery() and ble_scanner_purge_unknown_bonds()
 * do for their respective operations.
 *
 * At most one write can be pending at a time (static context).  For simple
 * camera-control commands (shutter on/off) this is sufficient — the HTTP
 * handler issues one command per button press and waits for the response
 * notification before allowing further commands.
 * -------------------------------------------------------------------------- */
#define GATT_WRITE_MAX_LEN 20

typedef struct {
    uint16_t conn_handle;
    uint16_t attr_handle;
    uint8_t  data[GATT_WRITE_MAX_LEN];
    uint16_t len;
} gatt_write_ctx_t;

static gatt_write_ctx_t      s_gatt_write_ctx;
static struct ble_npl_event  s_gatt_write_event;

static int gatt_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGW(TAG, "GATT write error on handle %d: status=%d",
                 conn_handle, error->status);
    } else {
        ESP_LOGD(TAG, "GATT write ack — handle %d attr 0x%04x",
                 conn_handle, attr ? attr->handle : 0);
    }
    return 0;
}

static void gatt_write_cb_event(struct ble_npl_event *ev)
{
    gatt_write_ctx_t *ctx = (gatt_write_ctx_t *)ble_npl_event_get_arg(ev);

    int rc = ble_gattc_write_flat(ctx->conn_handle, ctx->attr_handle,
                                  ctx->data, ctx->len,
                                  gatt_write_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_write_flat failed: rc=%d (conn=%d attr=0x%04x)",
                 rc, ctx->conn_handle, ctx->attr_handle);
    }
}

esp_err_t ble_scanner_gatt_write(uint16_t conn_handle, uint16_t attr_handle,
                                  const uint8_t *data, uint16_t len)
{
    if (len > GATT_WRITE_MAX_LEN) {
        ESP_LOGE(TAG, "ble_scanner_gatt_write: payload too large (%d > %d)",
                 len, GATT_WRITE_MAX_LEN);
        return ESP_ERR_INVALID_SIZE;
    }

    s_gatt_write_ctx.conn_handle = conn_handle;
    s_gatt_write_ctx.attr_handle = attr_handle;
    memcpy(s_gatt_write_ctx.data, data, len);
    s_gatt_write_ctx.len = len;

    ble_npl_event_init(&s_gatt_write_event, gatt_write_cb_event, &s_gatt_write_ctx);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_gatt_write_event);
    return ESP_OK;
}
