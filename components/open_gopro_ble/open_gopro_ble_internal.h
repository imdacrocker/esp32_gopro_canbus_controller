#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "open_gopro_ble.h"
#include "camera_driver.h"

/* -------------------------------------------------------------------------
 * Internal driver context — shared across control.c, gatt.c, pairing.c,
 * notify.c, readiness.c, and driver.c.  Not exposed in the public header.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t                  conn_handle;
    gopro_gatt_handles_t      gatt;
    camera_recording_status_t recording_status;
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
 * readiness.c — OpenGoPro BLE readiness polling
 *
 * gopro_readiness_start()            Begin polling GetHardwareInfo after CCCD
 *                                    subscriptions complete; gates gatt_ready.
 * gopro_readiness_handle_response()  Called by notify.c for every
 *                                    cmd_resp_notify notification received.
 * gopro_readiness_free()             Cancel polling and release the timer;
 *                                    must be called on disconnect.
 * ------------------------------------------------------------------------- */

void gopro_readiness_start(uint16_t conn_handle);
void gopro_readiness_handle_response(uint16_t conn_handle,
                                      const uint8_t *data, uint16_t len);
void gopro_readiness_free(uint16_t conn_handle);

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
 * ------------------------------------------------------------------------- */

esp_err_t control_start_recording(void *ctx);
esp_err_t control_stop_recording(void *ctx);
camera_recording_status_t control_get_recording_status(void *ctx);

void open_gopro_control_start_timers(void);
