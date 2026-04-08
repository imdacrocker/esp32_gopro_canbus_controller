#include "gopro_ble_internal.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_hs.h"
#include "ble_core.h"
#include "camera_manager.h"

static const char *TAG = "gopro_ble";

/* -------------------------------------------------------------------------
 * Driver vtable implementation
 * ------------------------------------------------------------------------- */

static esp_err_t gopro_driver_start_recording(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx || gctx->gatt.cmd_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    /* OpenGoPro TLV: [length=3][cmd_id=0x01][param_len=1][param=1] */
    uint8_t pkt[4] = { 0x03, 0x01, 0x01, 0x01 };
    return ble_core_gatt_write(gctx->conn_handle, gctx->gatt.cmd_write,
                               pkt, sizeof(pkt));
}

static esp_err_t gopro_driver_stop_recording(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx || gctx->gatt.cmd_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    /* OpenGoPro TLV: [length=3][cmd_id=0x01][param_len=1][param=0] */
    uint8_t pkt[4] = { 0x03, 0x01, 0x01, 0x00 };
    return ble_core_gatt_write(gctx->conn_handle, gctx->gatt.cmd_write,
                               pkt, sizeof(pkt));
}

static camera_recording_status_t gopro_driver_get_recording_status(void *ctx)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)ctx;
    if (!gctx) {
        return CAMERA_RECORDING_UNKNOWN;
    }
    return gctx->recording_status;
}

static const camera_driver_t s_gopro_ble_driver = {
    .start_recording      = gopro_driver_start_recording,
    .stop_recording       = gopro_driver_stop_recording,
    .get_recording_status = gopro_driver_get_recording_status,
};

const camera_driver_t *gopro_ble_get_driver(void)
{
    return &s_gopro_ble_driver;
}

void *gopro_ble_create_driver_ctx(void)
{
    gopro_ble_ctx_t *ctx = calloc(1, sizeof(gopro_ble_ctx_t));
    if (ctx) {
        ctx->conn_handle      = BLE_HS_CONN_HANDLE_NONE;
        ctx->recording_status = CAMERA_RECORDING_UNKNOWN;
    }
    return ctx;
}

/* Copy GATT handles into the right field of the driver context.
 * Called by gopro_gatt.c once CCCD subscription phase completes. */
void gopro_driver_set_gatt_handles(void *driver_ctx,
                                    const gopro_gatt_handles_t *handles)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)driver_ctx;
    if (gctx && handles) {
        gctx->gatt = *handles;
    }
}

/* -------------------------------------------------------------------------
 * Status polling timer — queries recording state of every connected camera
 * ------------------------------------------------------------------------- */

static esp_timer_handle_t s_status_poll_timer = NULL;

/* Query packet written to GP-0076 to request encoding_active (status ID 8) */
static const uint8_t k_status_query_pkt[] = { 0x02, 0x13, 0x08 };

static void status_poll_timer_cb(void *arg)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (!camera_manager_is_gatt_ready(i)) continue;

        uint16_t conn_h = camera_manager_get_handle(i);
        if (conn_h == BLE_HS_CONN_HANDLE_NONE) continue;

        gopro_ble_ctx_t *ctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(i);
        if (!ctx || ctx->gatt.query_write == 0) continue;

        esp_err_t err = ble_core_gatt_write(conn_h, ctx->gatt.query_write,
                                            k_status_query_pkt,
                                            sizeof(k_status_query_pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "slot %d: status query write failed (%s)",
                     i, esp_err_to_name(err));
        }
    }
}

static void start_status_poll_timer(void)
{
    if (s_status_poll_timer != NULL) {
        return;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = status_poll_timer_cb,
        .arg      = NULL,
        .name     = "gopro_status_poll",
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_status_poll_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create status poll timer: %s", esp_err_to_name(err));
        return;
    }

    err = esp_timer_start_periodic(s_status_poll_timer,
                                   (uint64_t)STATUS_POLL_INTERVAL_MS * 1000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start status poll timer: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Status poll timer started — interval %d ms",
                 STATUS_POLL_INTERVAL_MS);
    }
}

/* -------------------------------------------------------------------------
 * Discovery list management
 * ------------------------------------------------------------------------- */

static gopro_device_t s_discovered[GOPRO_MAX_DISCOVERED];
static int            s_discovered_count = 0;

void gopro_ble_start_discovery(void)
{
    s_discovered_count = 0;
    ble_core_start_discovery();
}

int gopro_ble_get_discovered(gopro_device_t *out, int max_count)
{
    int n = s_discovered_count < max_count ? s_discovered_count : max_count;
    memcpy(out, s_discovered, n * sizeof(gopro_device_t));
    return n;
}

void gopro_ble_connect_by_addr(const ble_addr_t *addr)
{
    ble_core_connect_by_addr(addr);
}

/* -------------------------------------------------------------------------
 * ble_core callbacks — defined in other files, forward-declared here
 * ------------------------------------------------------------------------- */
extern void gopro_on_connected_cb(uint16_t conn_handle, const ble_addr_t *addr);
extern void gopro_on_encrypted_cb(uint16_t conn_handle, const ble_addr_t *addr);
extern void gopro_on_disconnected_cb(uint16_t conn_handle, const ble_addr_t *addr,
                                      int reason);
extern void gopro_on_notify_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   const uint8_t *data, uint16_t len);

/* Discovery callback registered with ble_core */
static void gopro_on_disc_cb(const struct ble_gap_disc_desc *disc,
                              const struct ble_hs_adv_fields *fields)
{
    /* Filter: only GoPro cameras advertise service UUID 0xFEA6 */
    bool is_gopro = false;
    for (int i = 0; i < fields->num_uuids16; i++) {
        if (ble_uuid_u16(&fields->uuids16[i].u) == 0xFEA6) {
            is_gopro = true;
            break;
        }
    }
    if (!is_gopro) {
        return;
    }

    const uint8_t *a = disc->addr.val;

    /* Skip already-discovered entries */
    for (int i = 0; i < s_discovered_count; i++) {
        if (memcmp(s_discovered[i].addr.val, a, 6) == 0) {
            return;
        }
    }

    if (s_discovered_count >= GOPRO_MAX_DISCOVERED) {
        return;
    }

    gopro_device_t *d = &s_discovered[s_discovered_count++];
    memcpy(&d->addr, &disc->addr, sizeof(ble_addr_t));
    d->rssi = disc->rssi;

    if (fields->name != NULL && fields->name_len > 0) {
        int len = fields->name_len < (int)(sizeof(d->name) - 1)
                ? fields->name_len : (int)(sizeof(d->name) - 1);
        memcpy(d->name, fields->name, len);
        d->name[len] = '\0';
    } else {
        snprintf(d->name, sizeof(d->name), "GoPro %02X%02X", a[1], a[0]);
    }

    ESP_LOGI(TAG, "Discovered: %s  %02X:%02X:%02X:%02X:%02X:%02X",
             d->name, a[5], a[4], a[3], a[2], a[1], a[0]);
}

/* -------------------------------------------------------------------------
 * Initialisation
 * Module-level static so C99 designated-initialiser can use function addrs.
 * ------------------------------------------------------------------------- */
static const ble_core_callbacks_t s_ble_cbs = {
    .on_disc         = gopro_on_disc_cb,
    .on_connected    = gopro_on_connected_cb,
    .on_encrypted    = gopro_on_encrypted_cb,
    .on_disconnected = gopro_on_disconnected_cb,
    .on_notify_rx    = gopro_on_notify_rx_cb,
    .is_known_addr   = camera_manager_is_known_addr,
};

void gopro_ble_init(void)
{
    camera_manager_register_driver(CAMERA_TYPE_GOPRO_BLE,
                                    gopro_ble_get_driver(),
                                    gopro_ble_create_driver_ctx);

    ble_core_register_callbacks(&s_ble_cbs);

    start_status_poll_timer();

    ESP_LOGI(TAG, "GoPro BLE initialized");
}
