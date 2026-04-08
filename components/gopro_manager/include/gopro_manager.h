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
 * Full camera record.
 *
 * Persistent fields (saved to NVS):
 *   camera_name, camera_model, mac_address, is_paired
 *
 * Runtime fields (populated after connect, lost on restart):
 *   bt_handle, recording_status
 */
typedef struct {
    /* --- Persistent --- */
    char      camera_name[GOPRO_NAME_LEN];
    char      camera_model[GOPRO_MODEL_LEN];
    ble_addr_t mac_address;
    bool      is_paired;           // true = this slot holds a valid camera

    /* --- Runtime only --- */
    uint16_t  bt_handle;           // NimBLE connection handle; 0 = not connected
    char      ip_address[16];      // "xxx.xxx.xxx.xxx\0"; cleared on init
    gopro_recording_status_t recording_status;
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
 * Return the number of slots that are currently connected (bt_handle != 0).
 */
int gopro_manager_connected_count(void);

/**
 * Record that the camera in the given slot is now connected with the given handle.
 */
void gopro_manager_set_connected(int slot, uint16_t handle);

/**
 * Clear the bt_handle for whichever slot holds the given connection handle.
 * Safe to call with an unknown handle — does nothing if not found.
 */
void gopro_manager_set_disconnected(uint16_t handle);
