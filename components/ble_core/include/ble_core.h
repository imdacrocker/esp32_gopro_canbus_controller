#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/ble_gap.h"
#include "esp_err.h"

typedef void (*ble_core_on_disc_cb_t)(const struct ble_gap_disc_desc *disc,
                                       const struct ble_hs_adv_fields *fields);
typedef void (*ble_core_on_connected_cb_t)(uint16_t conn_handle, const ble_addr_t *addr);
typedef void (*ble_core_on_encrypted_cb_t)(uint16_t conn_handle, const ble_addr_t *addr);
typedef void (*ble_core_on_disconnected_cb_t)(uint16_t conn_handle, const ble_addr_t *addr, int reason);
typedef void (*ble_core_on_notify_rx_cb_t)(uint16_t conn_handle, uint16_t attr_handle,
                                            const uint8_t *data, uint16_t len);
typedef bool (*ble_core_is_known_addr_cb_t)(const ble_addr_t *addr);

typedef struct {
    ble_core_on_disc_cb_t          on_disc;
    ble_core_on_connected_cb_t     on_connected;
    ble_core_on_encrypted_cb_t     on_encrypted;
    ble_core_on_disconnected_cb_t  on_disconnected;
    ble_core_on_notify_rx_cb_t     on_notify_rx;
    ble_core_is_known_addr_cb_t    is_known_addr;
} ble_core_callbacks_t;

void ble_core_init(void);
void ble_core_start_discovery(void);
void ble_core_stop_discovery(void);
void ble_core_connect_by_addr(const ble_addr_t *addr);
void ble_core_purge_unknown_bonds(const ble_addr_t *keep, int keep_count);
esp_err_t ble_core_gatt_write(uint16_t conn_handle, uint16_t attr_handle,
                               const uint8_t *data, uint16_t len);
void ble_core_register_callbacks(const ble_core_callbacks_t *cbs);
