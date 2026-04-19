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
    /**
     * Set to true the moment a SetShutter(start) command is dispatched.
     * Cleared by handle_query_response() when the status poll confirms the
     * camera has transitioned to RECORDING (happy path) or back to IDLE after
     * a confirmed RECORDING (recovery path — lets the tick resend).
     * Also cleared by control_stop_recording() and on disconnect.
     *
     * Purpose: prevents the camera_manager tick from sending duplicate start
     * commands while the camera is still processing the first one.  The system
     * assumes the command was received; recovery is driven by status polling,
     * not by retrying the command immediately.
     */
    bool                      start_cmd_pending;
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

/**
 * Send a SetDateTime command (0x0D) to the camera on GP-0072.
 *
 * Fetches the current UTC from can_manager_get_utc_ms() and sends it.
 * Must be called after GATT discovery and CCCD subscriptions are complete
 * (cmd_write handle must be populated and cmd_resp_notify subscribed).
 *
 * Returns ESP_ERR_INVALID_STATE if UTC is not yet available (no GPS lock).
 * The caller is responsible for any retry logic in that case.
 */
esp_err_t control_send_set_date_time(uint16_t conn_handle);

/* -------------------------------------------------------------------------
 * presets.c — OpenGoPro preset commands
 *
 * gopro_presets_request_video()
 *   Phase 1 of the two-phase preset flow.  Sends RequestGetPresetStatus
 *   (Protobuf, Feature 0xF5, Action 0x72) to GP-0076 (query_write).  Must
 *   be called after GATT discovery and CCCD subscriptions are complete.
 *   The response arrives asynchronously on GP-0077 and is routed to
 *   gopro_presets_handle_notify_status() by notify.c.
 *
 * gopro_presets_handle_notify_status()
 *   Phase 2 of the two-phase preset flow.  Called by notify.c when a
 *   GP-0077 notification carries Feature ID 0xF5 (Preset feature).
 *   Parses the NotifyPresetStatus Protobuf payload to find the first preset
 *   in the Video group, then sends Load Preset (TLV 0x40) to GP-0072.
 *   The Load Preset response on GP-0073 is handled by the 0x40 case in
 *   gopro_query_handle_cmd_response() (query.c).
 * ------------------------------------------------------------------------- */

void gopro_presets_request_video(uint16_t conn_handle);

/** Handle a single-packet NotifyPresetStatus notification on GP-0077.
 *  Called by notify.c when frame_type=0 and Feature ID=0xF5. */
void gopro_presets_handle_notify_status(uint16_t conn_handle,
                                         const uint8_t *data, uint16_t len);

/** Handle a first-fragment or continuation-fragment GP-0077 notification
 *  belonging to the Preset Protobuf feature.  Called by notify.c for
 *  frame_type >= 1 when Feature ID=0xF5 or any continuation fragment.
 *  Manages reassembly; calls load_first_video_preset() when complete. */
void gopro_presets_handle_query_fragment(uint16_t conn_handle,
                                          const uint8_t *data, uint16_t len);

/** Release any in-progress preset reassembly context for this connection.
 *  Must be called from gopro_on_disconnected_cb() in pairing.c. */
void gopro_presets_free(uint16_t conn_handle);
