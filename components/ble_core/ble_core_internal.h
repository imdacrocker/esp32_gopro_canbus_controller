#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "ble_core.h"
#include "host/ble_gap.h"

/* Registered callbacks (defined in ble_init.c) */
extern ble_core_callbacks_t g_ble_core_cbs;

/* Shared connection state (defined in ble_connect.c) */
extern bool       s_connecting;
extern ble_addr_t s_pending_reconnect[];
extern int        s_pending_count;
extern int        s_pending_idx;

/* Internal functions shared across ble_core source files */
void start_scan(void);
void reconnect_next(void);

/* Connection event callback — defined in ble_connect.c, used in ble_scan.c */
int connection_event_cb(struct ble_gap_event *event, void *arg);
