#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "host/ble_gap.h"
#include "camera_driver.h"

#define GOPRO_MAX_DISCOVERED 10

typedef struct {
    char       name[32];
    ble_addr_t addr;
    int8_t     rssi;
} gopro_device_t;

typedef struct {
    uint16_t cmd_write;
    uint16_t cmd_resp_notify;
    uint16_t settings_write;
    uint16_t settings_resp_notify;
    uint16_t query_write;
    uint16_t query_resp_notify;
    uint16_t net_mgmt_cmd_write;
    uint16_t net_mgmt_resp_notify;
    uint16_t wifi_ssid_read;
    uint16_t wifi_pass_read;
    uint16_t wifi_power_write;
    uint16_t wifi_state_indicate;
} gopro_gatt_handles_t;

#define STATUS_POLL_INTERVAL_MS  5000

void gopro_ble_init(void);
void gopro_ble_start_discovery(void);
int  gopro_ble_get_discovered(gopro_device_t *out, int max_count);
void gopro_ble_connect_by_addr(const ble_addr_t *addr);

/* Called by camera_manager_register_driver pattern */
const camera_driver_t *gopro_ble_get_driver(void);
void *gopro_ble_create_driver_ctx(void);
