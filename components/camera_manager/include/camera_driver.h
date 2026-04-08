#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    CAMERA_RECORDING_UNKNOWN = 0,
    CAMERA_RECORDING_IDLE,
    CAMERA_RECORDING_ACTIVE,
} camera_recording_status_t;

typedef enum {
    CAMERA_TYPE_NONE = 0,
    CAMERA_TYPE_GOPRO_BLE,
} camera_type_t;

typedef struct camera_driver camera_driver_t;

struct camera_driver {
    esp_err_t (*start_recording)(void *ctx);
    esp_err_t (*stop_recording)(void *ctx);
    camera_recording_status_t (*get_recording_status)(void *ctx);
};
