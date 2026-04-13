#include "open_gopro_ble_internal.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "camera_manager.h"
#include "ble_core.h"

static const char *TAG = "open_gopro_ble";

/* GoPro 128-bit UUID definitions */
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

/* GATT discovery context */
#define GATT_MAX_SERVICES    16
#define GATT_MAX_NOTIFY_CHRS 16

typedef struct {
    uint16_t conn_handle;
    bool     active;

    struct {
        uint16_t start_handle;
        uint16_t end_handle;
    } svcs[GATT_MAX_SERVICES];
    int svc_count;
    int svc_idx;

    struct {
        uint16_t val_handle;
        uint16_t cccd_val;
    } notify_chrs[GATT_MAX_NOTIFY_CHRS];
    int notify_count;
    int notify_idx;

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

void free_gatt_disc_ctx(uint16_t conn_handle)
{
    gatt_disc_ctx_t *ctx = find_disc_ctx(conn_handle);
    if (ctx) {
        ctx->active = false;
    }
}

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

static int cccd_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg);
static void start_char_discovery(gatt_disc_ctx_t *ctx);
static void start_cccd_subscriptions(gatt_disc_ctx_t *ctx);

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
    } else {
        uint16_t h = attr ? attr->handle : 0;
        ESP_LOGI(TAG, "  Subscribed CCCD handle 0x%04x", h);
    }

    ctx->notify_idx++;

    if (ctx->notify_idx < ctx->notify_count) {
        uint16_t val_h  = ctx->notify_chrs[ctx->notify_idx].val_handle;
        uint16_t cccd_h = val_h + 1;
        uint16_t cccd_v = ctx->notify_chrs[ctx->notify_idx].cccd_val;
        int rc = ble_gattc_write_flat(conn_handle, cccd_h,
                                      &cccd_v, sizeof(cccd_v),
                                      cccd_write_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "CCCD write failed rc=%d — skipping", rc);
            struct ble_gatt_error fake_err = { .status = rc };
            cccd_write_cb(conn_handle, &fake_err, NULL, NULL);
        }
    } else {
        /* All subscriptions done — push handles into driver context, then start
         * the OpenGoPro BLE readiness poll.  gatt_ready is NOT set here;
         * readiness.c will call camera_manager_set_gatt_ready(slot, true)
         * only after GetHardwareInfo returns status 0 (camera ready). */
        int slot = camera_manager_find_by_handle(conn_handle);
        if (slot >= 0) {
            void *driver_ctx = camera_manager_get_driver_ctx(slot);
            gopro_driver_set_gatt_handles(driver_ctx, &ctx->handles);
            ESP_LOGI(TAG, "GATT setup complete for slot %d (%d notification(s))"
                     " — starting BLE readiness poll", slot, ctx->notify_count);
            free_gatt_disc_ctx(conn_handle);
            gopro_readiness_start(conn_handle);
        } else {
            ESP_LOGW(TAG, "GATT setup complete but camera slot not found (handle %d)",
                     conn_handle);
            free_gatt_disc_ctx(conn_handle);
        }
    }

    return 0;
}

static void start_cccd_subscriptions(gatt_disc_ctx_t *ctx)
{
    if (ctx->notify_count == 0) {
        ESP_LOGW(TAG, "No notifiable characteristics found for handle %d",
                 ctx->conn_handle);
        /* Still push any write/read handles we discovered */
        int slot = camera_manager_find_by_handle(ctx->conn_handle);
        if (slot >= 0) {
            void *driver_ctx = camera_manager_get_driver_ctx(slot);
            gopro_driver_set_gatt_handles(driver_ctx, &ctx->handles);
        }
        free_gatt_disc_ctx(ctx->conn_handle);
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
        free_gatt_disc_ctx(ctx->conn_handle);
    }
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    gatt_disc_ctx_t *ctx = find_disc_ctx(conn_handle);
    if (!ctx) {
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
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

    record_chr_handle(ctx, chr);

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
        free_gatt_disc_ctx(conn_handle);
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

/* Begin service + characteristic discovery (called after MTU exchange). */
static void begin_service_discovery(uint16_t conn_handle)
{
    int rc = ble_gattc_disc_all_svcs(conn_handle, svc_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed: %d", rc);
        free_gatt_disc_ctx(conn_handle);
    }
}

/* MTU exchange callback — proceed to service discovery regardless of result. */
static int mtu_exchange_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg)
{
    if (error->status != 0) {
        ESP_LOGW(TAG, "MTU exchange failed (handle %d status %d) — "
                 "proceeding with default MTU", conn_handle, error->status);
    } else {
        ESP_LOGI(TAG, "MTU negotiated: %d bytes (handle %d)", mtu, conn_handle);
    }
    begin_service_discovery(conn_handle);
    return 0;
}

void start_gatt_discovery(uint16_t conn_handle)
{
    ESP_LOGI(TAG, "Starting GATT discovery for handle %d", conn_handle);

    gatt_disc_ctx_t *ctx = alloc_disc_ctx(conn_handle);
    if (!ctx) {
        ESP_LOGE(TAG, "No free discovery context slots");
        return;
    }

    /* Negotiate the largest possible ATT MTU before service discovery.
     *
     * Why this matters for GetHardwareInfoRsp:
     *   The response is ~88 bytes.  With the default MTU of 23 bytes only
     *   20 bytes fit per ATT notification, so the camera fragments it into
     *   ~5 continuation packets.  With MTU = 517 (BLE_ATT_MTU_MAX) the
     *   entire response arrives in one notification, eliminating the need
     *   for application-layer reassembly.
     *
     * ble_att_set_preferred_mtu() sets the local preferred value; the actual
     * negotiated MTU is the minimum of both sides' preferences.  Most GoPro
     * cameras support at least 517 bytes. */
    ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);

    int rc = ble_gattc_exchange_mtu(conn_handle, mtu_exchange_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gattc_exchange_mtu failed rc=%d — skipping to service discovery",
                 rc);
        begin_service_discovery(conn_handle);
    }
}
