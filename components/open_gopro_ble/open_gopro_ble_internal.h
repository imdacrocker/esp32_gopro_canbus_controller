#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "open_gopro_ble.h"
#include "camera_driver.h"

/* -------------------------------------------------------------------------
 * Internal driver context — shared across control.c, gatt.c, pairing.c,
 * notify.c, query.c, and driver.c.  Not exposed in the public header.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t                  conn_handle;
    gopro_gatt_handles_t      gatt;
    camera_recording_status_t recording_status;
    /** True only on the very first pairing of this camera (slot was unknown
     *  when gopro_on_encrypted_cb fired).  Cleared after RequestPairingFinish
     *  is sent in control_send_pairing_complete(). */
    bool                      is_first_pairing;
} gopro_ble_ctx_t;

/* -------------------------------------------------------------------------
 * driver.c — driver vtable and context helpers
 * ------------------------------------------------------------------------- */

/** Set the discovered GATT handles into a driver context.
 *  Called by gatt.c when discovery completes. */
void gopro_driver_set_gatt_handles(void *driver_ctx,
                                    const gopro_gatt_handles_t *handles);

/* -------------------------------------------------------------------------
 * gatt.c — GATT discovery lifecycle
 * ------------------------------------------------------------------------- */

void start_gatt_discovery(uint16_t conn_handle);
void free_gatt_disc_ctx(uint16_t conn_handle);

/* -------------------------------------------------------------------------
 * query.c — OpenGoPro on-demand query commands
 *
 * gopro_query_send_hw_info()         Send a GetHardwareInfo (0x3C) command to
 *                                    the camera.  The response arrives on
 *                                    cmd_resp_notify and is routed here by
 *                                    notify.c.
 * gopro_query_handle_cmd_response()  Called by notify.c for every
 *                                    cmd_resp_notify notification received.
 *                                    Handles GPBS reassembly for multi-fragment
 *                                    responses and dispatches by cmd_id.
 * gopro_query_free()                 Release any reassembly context for this
 *                                    connection handle; call on disconnect.
 * ------------------------------------------------------------------------- */

void gopro_query_send_hw_info(uint16_t conn_handle);
void gopro_query_handle_cmd_response(uint16_t conn_handle,
                                     const uint8_t *data, uint16_t len);
void gopro_query_free(uint16_t conn_handle);

/* -------------------------------------------------------------------------
 * control.c — camera control commands and periodic timers
 *
 * Recording command implementations (invoked via the camera_driver_t vtable).
 * The vtable in driver.c holds function pointers to these; they are not
 * intended to be called directly from outside the component.
 *
 * open_gopro_control_start_timers()  Start the status-poll and keep-alive
 *                                    timers.  Called once from
 *                                    open_gopro_ble_init() in driver.c.
 *
 * control_send_pairing_complete()    Send RequestPairingFinish to dismiss the
 *                                    camera's pairing screen.  Must be called
 *                                    only on first-time pairing, after GATT
 *                                    char discovery (net_mgmt_cmd_write must
 *                                    be populated) but before CCCD subscriptions
 *                                    start.  Fire-and-forget; response on
 *                                    GP-0092 is not awaited.
 * ------------------------------------------------------------------------- */

esp_err_t control_start_recording(void *ctx);
esp_err_t control_stop_recording(void *ctx);
camera_recording_status_t control_get_recording_status(void *ctx);

void open_gopro_control_start_timers(void);
void control_send_pairing_complete(uint16_t conn_handle);
