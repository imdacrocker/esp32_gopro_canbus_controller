#include "ble_core.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_gatt.h"
#include "nimble/nimble_port.h"
#include "freertos/portmacro.h"

static const char *TAG = "ble_core";

#define GATT_WRITE_QUEUE_SIZE  4
#define GATT_WRITE_MAX_LEN    20

typedef struct {
    uint16_t             conn_handle;
    uint16_t             attr_handle;
    uint8_t              data[GATT_WRITE_MAX_LEN];
    uint16_t             len;
    struct ble_npl_event event;
} gatt_write_entry_t;

static gatt_write_entry_t s_write_queue[GATT_WRITE_QUEUE_SIZE];
static int s_queue_head = 0;
static int s_queue_tail = 0;
static int s_queue_count = 0;
static portMUX_TYPE s_queue_lock = portMUX_INITIALIZER_UNLOCKED;

static int gatt_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGW(TAG, "GATT write error on handle %d: status=%d",
                 conn_handle, error->status);
    } else {
        ESP_LOGD(TAG, "GATT write ack — handle %d attr 0x%04x",
                 conn_handle, attr ? attr->handle : 0);
    }
    return 0;
}

static void gatt_write_event_cb(struct ble_npl_event *ev)
{
    gatt_write_entry_t *entry = (gatt_write_entry_t *)ble_npl_event_get_arg(ev);

    int rc = ble_gattc_write_flat(entry->conn_handle, entry->attr_handle,
                                  entry->data, entry->len,
                                  gatt_write_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_write_flat failed: rc=%d (conn=%d attr=0x%04x)",
                 rc, entry->conn_handle, entry->attr_handle);
    }

    /* Advance queue pointer */
    portENTER_CRITICAL(&s_queue_lock);
    s_queue_tail = (s_queue_tail + 1) % GATT_WRITE_QUEUE_SIZE;
    s_queue_count--;
    portEXIT_CRITICAL(&s_queue_lock);
}

esp_err_t ble_core_gatt_write(uint16_t conn_handle, uint16_t attr_handle,
                               const uint8_t *data, uint16_t len)
{
    if (len > GATT_WRITE_MAX_LEN) {
        ESP_LOGE(TAG, "ble_core_gatt_write: payload too large (%d > %d)",
                 len, GATT_WRITE_MAX_LEN);
        return ESP_ERR_INVALID_SIZE;
    }

    portENTER_CRITICAL(&s_queue_lock);

    if (s_queue_count >= GATT_WRITE_QUEUE_SIZE) {
        portEXIT_CRITICAL(&s_queue_lock);
        ESP_LOGW(TAG, "GATT write queue full");
        return ESP_ERR_NO_MEM;
    }

    gatt_write_entry_t *entry = &s_write_queue[s_queue_head];
    entry->conn_handle = conn_handle;
    entry->attr_handle = attr_handle;
    memcpy(entry->data, data, len);
    entry->len = len;

    s_queue_head = (s_queue_head + 1) % GATT_WRITE_QUEUE_SIZE;
    s_queue_count++;

    portEXIT_CRITICAL(&s_queue_lock);

    /* Post event to NimBLE host task */
    ble_npl_event_init(&entry->event, gatt_write_event_cb, entry);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &entry->event);

    return ESP_OK;
}
