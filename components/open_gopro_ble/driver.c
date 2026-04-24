/**
 * @file driver.c
 * @brief OpenGoPro BLE driver — camera_driver_t vtable, context allocation, and init.
 *
 * Responsibilities
 * ----------------
 *  - Provides the camera_driver_t vtable (start_recording, stop_recording,
 *    get_recording_status) registered with camera_manager.
 *  - Allocates and zeroes per-camera gopro_ble_ctx_t contexts via
 *    open_gopro_ble_create_driver_ctx().
 *  - Maintains the discovery list (gopro_device_t[GOPRO_MAX_DISCOVERED]) populated
 *    during BLE scans.
 *  - Wires ble_core callbacks to the pairing.c handlers on init.
 */

#include "open_gopro_ble_internal.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "ble_core.h"
#include "camera_manager.h"

static const char *TAG = "open_gopro_ble";

/* -------------------------------------------------------------------------
 * Driver vtable
 *
 * Recording-command implementations live in control.c; the vtable holds
 * function pointers to them.  get/set recording status also lives there.
 * ------------------------------------------------------------------------- */

static const camera_driver_t s_gopro_ble_driver = {
    .start_recording      = control_start_recording,
    .stop_recording       = control_stop_recording,
    .get_recording_status = control_get_recording_status,
};

const camera_driver_t *open_gopro_ble_get_driver(void)
{
    return &s_gopro_ble_driver;
}

void *open_gopro_ble_create_driver_ctx(void)
{
    gopro_ble_ctx_t *ctx = calloc(1, sizeof(gopro_ble_ctx_t));
    if (ctx) {
        ctx->conn_handle      = BLE_HS_CONN_HANDLE_NONE;
        ctx->recording_status = CAMERA_RECORDING_UNKNOWN;
    }
    return ctx;
}

/* Copy GATT handles into the right field of the driver context.
 * Called by gatt.c once CCCD subscription phase completes. */
void gopro_driver_set_gatt_handles(void *driver_ctx,
                                    const gopro_gatt_handles_t *handles)
{
    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)driver_ctx;
    if (gctx && handles) {
        gctx->gatt = *handles;
    }
}

/* -------------------------------------------------------------------------
 * Discovery list management
 * ------------------------------------------------------------------------- */

static gopro_device_t s_discovered[GOPRO_MAX_DISCOVERED];
static int            s_discovered_count = 0;

void open_gopro_ble_start_discovery(void)
{
    s_discovered_count = 0;
    ble_core_start_discovery();
}

void open_gopro_ble_stop_discovery(void)
{
    ble_core_stop_discovery();
}

int open_gopro_ble_get_discovered(gopro_device_t *out, int max_count)
{
    int n = s_discovered_count < max_count ? s_discovered_count : max_count;
    memcpy(out, s_discovered, n * sizeof(gopro_device_t));
    return n;
}

void open_gopro_ble_connect_by_addr(const ble_addr_t *addr)
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
 * ------------------------------------------------------------------------- */

static const ble_core_callbacks_t s_ble_cbs = {
    .on_disc                  = gopro_on_disc_cb,
    .on_connected             = gopro_on_connected_cb,
    .on_encrypted             = gopro_on_encrypted_cb,
    .on_disconnected          = gopro_on_disconnected_cb,
    .on_notify_rx             = gopro_on_notify_rx_cb,
    .is_known_addr            = camera_manager_is_known_addr,
    .has_disconnected_cameras = camera_manager_has_disconnected_cameras,
};

void open_gopro_ble_init(void)
{
    camera_manager_register_driver(CAMERA_TYPE_GOPRO_BLE,
                                    open_gopro_ble_get_driver(),
                                    open_gopro_ble_create_driver_ctx);

    ble_core_register_callbacks(&s_ble_cbs);

    /* Start the status-poll timer (5 s) and keep-alive timer (3 s).
     * Both iterate all camera slots on each fire and operate only on
     * slots where camera_manager_is_gatt_ready() returns true. */
    open_gopro_control_start_timers();

    ESP_LOGI(TAG, "OpenGoPro BLE initialized");
}
