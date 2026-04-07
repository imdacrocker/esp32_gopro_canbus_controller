#include "gopro_manager.h"

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "gopro_manager";

/* NVS key used within each camera namespace to store the blob */
#define NVS_CAMERA_KEY   "camera"

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
    for (int i = 0; i < GOPRO_MAX_CAMERAS; i++) {
        load_slot(i);
    }
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
