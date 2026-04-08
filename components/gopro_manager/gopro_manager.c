#include "gopro_manager.h"

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "host/ble_hs.h"   /* BLE_HS_CONN_HANDLE_NONE */
#include "ble_scanner.h"   /* ble_scanner_gatt_write() */

static const char *TAG = "gopro_manager";

/* NVS key used within each camera namespace to store the blob */
#define NVS_CAMERA_KEY   "camera"

/* Forward declaration — defined in the "Recording-status polling" section below */
static void start_status_poll_timer(void);

/* NVS namespace for a given slot — built on demand so it scales with
 * CONFIG_BT_NIMBLE_MAX_BONDS without needing a hardcoded list. */
static void slot_namespace(int slot, char *buf, size_t len)
{
    snprintf(buf, len, "gopro_%d", slot);
}

/* In-RAM camera list */
static gopro_camera_t s_cameras[GOPRO_MAX_CAMERAS];

/* -----------------------------------------------------------------------
 * Persistent blob layout — only the fields we actually save to NVS.
 * Keeping this separate from gopro_camera_t means runtime fields never
 * accidentally get written to flash.
 * --------------------------------------------------------------------- */
typedef struct {
    char       camera_name[GOPRO_NAME_LEN];
    char       camera_model[GOPRO_MODEL_LEN];
    ble_addr_t mac_address;
    bool       is_paired;
} gopro_nv_record_t;

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

static void load_slot(int slot)
{
    char ns[16];
    slot_namespace(slot, ns, sizeof(ns));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* No record yet — leave slot zeroed */
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(%s) failed: %s", ns, esp_err_to_name(err));
        return;
    }

    gopro_nv_record_t rec;
    size_t len = sizeof(rec);
    err = nvs_get_blob(handle, NVS_CAMERA_KEY, &rec, &len);
    nvs_close(handle);

    if (err != ESP_OK || len != sizeof(rec)) {
        ESP_LOGW(TAG, "slot %d: blob missing or wrong size", slot);
        return;
    }

    /* Copy persistent fields into RAM record */
    memcpy(s_cameras[slot].camera_name, rec.camera_name, GOPRO_NAME_LEN);
    memcpy(s_cameras[slot].camera_model, rec.camera_model, GOPRO_MODEL_LEN);
    s_cameras[slot].mac_address = rec.mac_address;
    s_cameras[slot].is_paired   = rec.is_paired;

    ESP_LOGI(TAG, "slot %d loaded: %s", slot, s_cameras[slot].camera_name);
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void gopro_manager_init(void)
{
    memset(s_cameras, 0, sizeof(s_cameras));

    /* 0 is a valid NimBLE connection handle; use BLE_HS_CONN_HANDLE_NONE
     * (0xFFFF) as the "not connected" sentinel for every slot. */
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        s_cameras[i].bt_handle = BLE_HS_CONN_HANDLE_NONE;
    }

    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        load_slot(i);
    }

    start_status_poll_timer();
}

gopro_camera_t *gopro_manager_get(int slot)
{
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS) {
        return NULL;
    }
    return &s_cameras[slot];
}

int gopro_manager_find_by_addr(const ble_addr_t *addr)
{
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        if (!s_cameras[i].is_paired) {
            continue;
        }
        if (memcmp(&s_cameras[i].mac_address, addr, sizeof(ble_addr_t)) == 0) {
            return i;
        }
    }
    return -1;
}

esp_err_t gopro_manager_save(int slot)
{
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }

    gopro_nv_record_t rec;
    memcpy(rec.camera_name, s_cameras[slot].camera_name, GOPRO_NAME_LEN);
    memcpy(rec.camera_model, s_cameras[slot].camera_model, GOPRO_MODEL_LEN);
    rec.mac_address = s_cameras[slot].mac_address;
    rec.is_paired   = s_cameras[slot].is_paired;

    char ns[16];
    slot_namespace(slot, ns, sizeof(ns));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) failed: %s", ns, esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, NVS_CAMERA_KEY, &rec, sizeof(rec));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "slot %d save failed: %s", slot, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "slot %d saved: %s", slot, s_cameras[slot].camera_name);
    }
    return err;
}

esp_err_t gopro_manager_remove(int slot)
{
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_cameras[slot], 0, sizeof(gopro_camera_t));

    char ns[16];
    slot_namespace(slot, ns, sizeof(ns));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK; /* Nothing stored, nothing to erase */
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, NVS_CAMERA_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK; /* Key didn't exist, that's fine */
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "slot %d removed", slot);
    return err;
}

/* -----------------------------------------------------------------------
 * Runtime status helpers
 * --------------------------------------------------------------------- */

int gopro_manager_remembered_count(void)
{
    int count = 0;
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        if (s_cameras[i].is_paired) count++;
    }
    return count;
}

int gopro_manager_find_free_slot(void)
{
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        if (!s_cameras[i].is_paired) return i;
    }
    return -1;
}

int gopro_manager_connected_count(void)
{
    int count = 0;
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        if (s_cameras[i].is_paired &&
            s_cameras[i].bt_handle != BLE_HS_CONN_HANDLE_NONE) {
            count++;
        }
    }
    return count;
}

void gopro_manager_set_connected(int slot, uint16_t handle)
{
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS) return;
    s_cameras[slot].bt_handle = handle;
    ESP_LOGI(TAG, "slot %d connected — handle: %d", slot, handle);
}

void gopro_manager_set_disconnected(uint16_t handle)
{
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        if (s_cameras[i].bt_handle == handle) {
            s_cameras[i].bt_handle  = BLE_HS_CONN_HANDLE_NONE;
            s_cameras[i].gatt_ready = false;
            memset(&s_cameras[i].gatt, 0, sizeof(s_cameras[i].gatt));
            ESP_LOGI(TAG, "slot %d disconnected", i);
            return;
        }
    }
}

int gopro_manager_find_by_handle(uint16_t handle)
{
    if (handle == BLE_HS_CONN_HANDLE_NONE) return -1;
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        if (s_cameras[i].bt_handle == handle) {
            return i;
        }
    }
    return -1;
}

void gopro_manager_set_gatt_handles(int slot, const gopro_gatt_handles_t *handles)
{
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS || handles == NULL) return;
    s_cameras[slot].gatt = *handles;
}

void gopro_manager_set_gatt_ready(int slot, bool ready)
{
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS) return;
    s_cameras[slot].gatt_ready = ready;
    ESP_LOGI(TAG, "slot %d gatt_ready = %s", slot, ready ? "true" : "false");
}

/* -----------------------------------------------------------------------
 * Recording-status polling
 *
 * OpenGoPro Query Service (GP-0076 write / GP-0077 notify):
 *   Request  – [0x02][0x13][0x08]
 *                       ^Get Status  ^Status ID 8 = encoding_active
 *   Response – [len][0x13][result][0x08][value_len][value]
 *                                             ^0x00=idle  0x01=recording
 * --------------------------------------------------------------------- */

/* Query packet written to GP-0076 to ask for encoding_active status */
static const uint8_t k_status_query_pkt[] = { 0x02, 0x13, 0x08 };
#define GP_QUERY_GET_STATUS  0x13
#define GP_STATUS_ID_ENCODING 0x08

static esp_timer_handle_t s_status_poll_timer = NULL;

static void status_poll_timer_cb(void *arg)
{
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        gopro_camera_t *cam = &s_cameras[i];
        if (!cam->is_paired)                              continue;
        if (cam->bt_handle == BLE_HS_CONN_HANDLE_NONE)   continue;
        if (!cam->gatt_ready)                             continue;
        if (cam->gatt.query_write == 0)                   continue;

        esp_err_t err = ble_scanner_gatt_write(cam->bt_handle,
                                               cam->gatt.query_write,
                                               k_status_query_pkt,
                                               sizeof(k_status_query_pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "slot %d: status query write failed (%s)",
                     i, esp_err_to_name(err));
        }
    }
}

static void start_status_poll_timer(void)
{
    if (s_status_poll_timer != NULL) {
        return; /* Already running */
    }

    const esp_timer_create_args_t timer_args = {
        .callback = status_poll_timer_cb,
        .arg      = NULL,
        .name     = "gopro_status_poll",
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_status_poll_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create status poll timer: %s", esp_err_to_name(err));
        return;
    }

    err = esp_timer_start_periodic(s_status_poll_timer,
                                   (uint64_t)STATUS_POLL_INTERVAL_MS * 1000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start status poll timer: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Status poll timer started — interval %d ms", STATUS_POLL_INTERVAL_MS);
    }
}

void gopro_manager_handle_query_response(int slot, const uint8_t *data, uint16_t len)
{
    /* Minimum valid packet: [len][query_id][result][status_id][val_len][val] = 6 bytes */
    if (slot < 0 || slot >= GOPRO_MAX_CAMERAS || data == NULL || len < 6) {
        return;
    }

    uint8_t query_id = data[1];
    uint8_t result   = data[2];

    if (query_id != GP_QUERY_GET_STATUS) {
        return; /* Not a response we care about yet */
    }

    if (result != 0x00) {
        ESP_LOGW(TAG, "slot %d: status query returned error 0x%02x", slot, result);
        return;
    }

    /* Walk the packed TLV status entries that follow the header */
    uint16_t idx = 3;
    while (idx + 2 <= len) {
        uint8_t status_id  = data[idx++];
        uint8_t value_len  = data[idx++];

        if (idx + value_len > len) {
            break; /* Truncated packet */
        }

        if (status_id == GP_STATUS_ID_ENCODING && value_len >= 1) {
            gopro_recording_status_t new_status =
                data[idx] ? GOPRO_RECORDING_ACTIVE : GOPRO_RECORDING_IDLE;

            if (s_cameras[slot].recording_status != new_status) {
                s_cameras[slot].recording_status = new_status;
                ESP_LOGI(TAG, "slot %d: recording status → %s", slot,
                         new_status == GOPRO_RECORDING_ACTIVE ? "RECORDING" : "IDLE");
            }
        }

        idx += value_len;
    }
}

/* -----------------------------------------------------------------------
 * Generic TLV command broadcast
 * --------------------------------------------------------------------- */
int gopro_manager_send_command_all(uint8_t cmd_id, const uint8_t *params, uint8_t param_len)
{
    /* OpenGoPro TLV packet layout:
     *   Byte 0: total length of remaining bytes  (1 [cmd_id] + 1 [param_len] + param_len)
     *   Byte 1: command ID
     *   Byte 2: parameter length
     *   Bytes 3…: parameter value(s)
     *
     * Maximum supported payload is 17 parameter bytes so the whole packet
     * fits within the 20-byte BLE ATT MTU limit.
     */
    if (param_len > 17) {
        ESP_LOGE(TAG, "send_command_all: param_len %d exceeds 17", param_len);
        return 0;
    }

    uint8_t pkt[20];
    pkt[0] = (uint8_t)(2 + param_len);  /* length: cmd_id + param_len_field + params */
    pkt[1] = cmd_id;
    pkt[2] = param_len;
    if (param_len > 0 && params != NULL) {
        memcpy(&pkt[3], params, param_len);
    }
    uint16_t pkt_len = (uint16_t)(3 + param_len);

    int dispatched = 0;
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        gopro_camera_t *cam = &s_cameras[i];
        if (!cam->is_paired) continue;
        if (cam->bt_handle == BLE_HS_CONN_HANDLE_NONE) continue;
        if (!cam->gatt_ready) continue;
        if (cam->gatt.cmd_write == 0) continue;

        esp_err_t err = ble_scanner_gatt_write(cam->bt_handle,
                                               cam->gatt.cmd_write,
                                               pkt, pkt_len);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "slot %d: dispatched cmd 0x%02x", i, cmd_id);
            dispatched++;
        } else {
            ESP_LOGW(TAG, "slot %d: gatt_write failed (%s)", i, esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "send_command_all(0x%02x): dispatched to %d camera(s)", cmd_id, dispatched);
    return dispatched;
}
