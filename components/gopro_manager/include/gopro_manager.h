#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/ble_gap.h"
#include "sdkconfig.h"

/* Max cameras equals the NimBLE bond limit — one bond per camera.
 * Change CONFIG_BT_NIMBLE_MAX_BONDS in sdkconfig to adjust both. */
#define GOPRO_MAX_CAMERAS   CONFIG_BT_NIMBLE_MAX_BONDS
#define GOPRO_NAME_LEN      32
#define GOPRO_MODEL_LEN     32

/**
 * Live recording state — populated at runtime, never written to NVS.
 */
typedef enum {
    GOPRO_RECORDING_UNKNOWN = 0,
    GOPRO_RECORDING_IDLE,
    GOPRO_RECORDING_ACTIVE,
} gopro_recording_status_t;

/**
 * GATT characteristic handles discovered after connection.
 * All values are ATT value handles (the writable/readable/notifiable handle,
 * not the characteristic declaration handle).  0 = not yet discovered.
 *
 * GoPro 128-bit UUID base: b5f9XXXX-aa8d-11e3-9046-0002a5d5c51b
 */
typedef struct {
    /* Control & Query Service (FEA6) */
    uint16_t cmd_write;              /* GP-0072  Command              (write)  */
    uint16_t cmd_resp_notify;        /* GP-0073  Command Response     (notify) */
    uint16_t settings_write;         /* GP-0074  Settings             (write)  */
    uint16_t settings_resp_notify;   /* GP-0075  Settings Response    (notify) */
    uint16_t query_write;            /* GP-0076  Query                (write)  */
    uint16_t query_resp_notify;      /* GP-0077  Query Response       (notify) */

    /* Camera Management Service (GP-0090) */
    uint16_t net_mgmt_cmd_write;     /* GP-0091  Net Mgmt Command     (write)  */
    uint16_t net_mgmt_resp_notify;   /* GP-0092  Net Mgmt Response    (notify) */

    /* WiFi AP Service (GP-0001) */
    uint16_t wifi_ssid_read;         /* GP-0002  WiFi AP SSID         (read)   */
    uint16_t wifi_pass_read;         /* GP-0003  WiFi AP Password     (read)   */
    uint16_t wifi_power_write;       /* GP-0004  WiFi AP Power        (write)  */
    uint16_t wifi_state_indicate;    /* GP-0005  WiFi AP State        (indicate)*/
} gopro_gatt_handles_t;

/**
 * Full camera record.
 *
 * Persistent fields (saved to NVS):
 *   camera_name, camera_model, mac_address, is_paired
 *
 * Runtime fields (populated after connect, lost on restart):
 *   bt_handle, recording_status, gatt, gatt_ready
 */
typedef struct {
    /* --- Persistent --- */
    char      camera_name[GOPRO_NAME_LEN];
    char      camera_model[GOPRO_MODEL_LEN];
    ble_addr_t mac_address;
    bool      is_paired;           // true = this slot holds a valid camera

    /* --- Runtime only --- */
    uint16_t  bt_handle;           // NimBLE connection handle; BLE_HS_CONN_HANDLE_NONE = not connected
    char      ip_address[16];      // "xxx.xxx.xxx.xxx\0"; cleared on init
    gopro_recording_status_t recording_status;

    /* GATT handles populated during post-connect discovery */
    gopro_gatt_handles_t gatt;
    bool                 gatt_ready; // true once all notify subscriptions are set up
} gopro_camera_t;

/**
 * Load all paired cameras from NVS into RAM.
 * Call once from app_main before anything uses the camera list.
 * Unpaired slots are zeroed out.
 */
void gopro_manager_init(void);

/**
 * Return a pointer to the internal camera record for the given slot (0–4).
 * Returns NULL if slot is out of range.
 * Caller may read or modify fields directly; call gopro_manager_save() to persist.
 */
gopro_camera_t *gopro_manager_get(int slot);

/**
 * Find a paired camera by BLE address.
 * Returns the slot index (0–4), or -1 if not found.
 */
int gopro_manager_find_by_addr(const ble_addr_t *addr);

/**
 * Write the persistent fields of the given slot to NVS.
 * Returns ESP_OK on success.
 */
esp_err_t gopro_manager_save(int slot);

/**
 * Mark a slot as unpaired, zero it out, and erase it from NVS.
 * Returns ESP_OK on success.
 */
esp_err_t gopro_manager_remove(int slot);

/**
 * Return the number of slots that have is_paired == true.
 */
int gopro_manager_remembered_count(void);

/**
 * Return the index of the first slot where is_paired == false, or -1 if all
 * slots are occupied.
 */
int gopro_manager_find_free_slot(void);

/**
 * Return the number of slots that are currently connected (bt_handle != 0).
 */
int gopro_manager_connected_count(void);

/**
 * Record that the camera in the given slot is now connected with the given handle.
 */
void gopro_manager_set_connected(int slot, uint16_t handle);

/**
 * Clear the bt_handle for whichever slot holds the given connection handle.
 * Also clears gatt_ready and all GATT handles for that slot.
 * Safe to call with an unknown handle — does nothing if not found.
 */
void gopro_manager_set_disconnected(uint16_t handle);

/**
 * Find a connected camera by its NimBLE connection handle.
 * Returns the slot index (0 – GOPRO_MAX_CAMERAS-1), or -1 if not found.
 */
int gopro_manager_find_by_handle(uint16_t handle);

/**
 * Store the discovered GATT characteristic handles for a given slot.
 * Copies the provided handle map into the camera record.
 */
void gopro_manager_set_gatt_handles(int slot, const gopro_gatt_handles_t *handles);

/**
 * Mark whether GATT setup (service discovery + notification subscriptions)
 * has completed for the given slot.
 */
void gopro_manager_set_gatt_ready(int slot, bool ready);

/**
 * Send a TLV command to every camera that is currently connected and GATT-ready.
 *
 * The function builds the standard OpenGoPro TLV packet:
 *   [length][cmd_id][param_len][param...]
 * and writes it to each camera's Command characteristic (GP-0072).
 *
 * @param cmd_id    Command ID byte (e.g. 0x01 for Set Shutter).
 * @param params    Optional parameter bytes.  May be NULL when param_len == 0.
 * @param param_len Number of parameter bytes (0 for commands with no parameters).
 * @return Number of cameras the command was dispatched to (0 = none connected/ready).
 */
int gopro_manager_send_command_all(uint8_t cmd_id, const uint8_t *params, uint8_t param_len);
