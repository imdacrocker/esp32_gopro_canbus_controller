#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "camera_driver.h"
#include "host/ble_gap.h"
#include "sdkconfig.h"

/* Max cameras equals the NimBLE bond limit — one bond per camera. */
#define CAMERA_MAX_SLOTS    CONFIG_BT_NIMBLE_MAX_BONDS
#define CAMERA_NAME_LEN     32

#define CAMERA_STATUS_NOT_CONFIGURED  (-1)
#define CAMERA_STATUS_DISCONNECTED      0
#define CAMERA_STATUS_CONNECTED         1
#define CAMERA_STATUS_RECORDING         2

typedef struct {
    int        index;
    char       name[CAMERA_NAME_LEN];
    ble_addr_t mac_address;
    bool       is_configured;
    int        status;
} camera_slot_info_t;

void camera_manager_init(void);
int  camera_manager_find_by_addr(const ble_addr_t *addr);
int  camera_manager_find_free_slot(void);
int  camera_manager_remembered_count(void);
int  camera_manager_connected_count(void);
void camera_manager_on_connected(int slot, uint16_t conn_handle);
void camera_manager_on_disconnected(uint16_t conn_handle);
void camera_manager_set_gatt_ready(int slot, bool ready);
int  camera_manager_register_new(const ble_addr_t *addr, const char *name,
                                  const camera_driver_t *driver, void *driver_ctx,
                                  camera_type_t type);
int  camera_manager_find_by_handle(uint16_t conn_handle);
uint16_t camera_manager_get_handle(int slot);
bool camera_manager_is_gatt_ready(int slot);
void *camera_manager_get_driver_ctx(int slot);
esp_err_t camera_manager_save_slot(int slot);
esp_err_t camera_manager_remove_slot(int slot);
camera_slot_info_t camera_manager_get_slot_info(int slot);
void camera_manager_set_desired_recording(bool recording);
int  camera_manager_start_recording_all(void);
int  camera_manager_stop_recording_all(void);
bool camera_manager_is_known_addr(const ble_addr_t *addr);
void camera_manager_register_driver(camera_type_t type,
                                     const camera_driver_t *driver,
                                     void *(*create_ctx)(void));

/**
 * @brief Callback fired when a camera slot's derived status changes.
 *
 * Invoked from camera_manager_tick() (every 2 s) and immediately from
 * on_connected / on_disconnected / set_gatt_ready so that the CAN status
 * broadcast stays current without polling.
 *
 * @param slot    Camera slot index (0-based).
 * @param status  One of the CAMERA_STATUS_* constants defined above.
 * @param ctx     Opaque context registered with the callback.
 */
typedef void (*camera_state_change_fn_t)(int slot, int status, void *ctx);

/**
 * @brief Register a callback to receive camera slot state changes.
 *
 * Call before camera_manager_init() so that no transitions are missed
 * during the initial slot load.  Only one callback is supported at a time.
 *
 * @param cb   Callback function.
 * @param ctx  Opaque context pointer passed to the callback unchanged.
 */
void camera_manager_register_state_change_callback(camera_state_change_fn_t cb, void *ctx);
