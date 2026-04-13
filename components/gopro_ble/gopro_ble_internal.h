#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gopro_ble.h"
#include "camera_driver.h"

/* -------------------------------------------------------------------------
 * Internal driver context — shared across gopro_gatt.c, gopro_pairing.c,
 * gopro_notify.c, and gopro_driver.c.  Not exposed in the public header.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t                  conn_handle;
    gopro_gatt_handles_t      gatt;
    camera_recording_status_t recording_status;
} gopro_ble_ctx_t;

/* Set the discovered GATT handles into a driver context.
 * Defined in gopro_driver.c; called by gopro_gatt.c when discovery completes. */
void gopro_driver_set_gatt_handles(void *driver_ctx,
                                    const gopro_gatt_handles_t *handles);

/* GATT discovery lifecycle — defined in gopro_gatt.c, used in gopro_pairing.c */
void start_gatt_discovery(uint16_t conn_handle);
void free_gatt_disc_ctx(uint16_t conn_handle);

/* BLE readiness polling — defined in gopro_readiness.c
 *
 * gopro_readiness_start()         — begin polling GetHardwareInfo after CCCD
 *                                   subscriptions complete; gates gatt_ready.
 * gopro_readiness_handle_response() — called by gopro_notify.c for every
 *                                   cmd_resp_notify notification received.
 * gopro_readiness_free()          — cancel polling and release the timer;
 *                                   must be called on disconnect. */
void gopro_readiness_start(uint16_t conn_handle);
void gopro_readiness_handle_response(uint16_t conn_handle,
                                      const uint8_t *data, uint16_t len);
void gopro_readiness_free(uint16_t conn_handle);
