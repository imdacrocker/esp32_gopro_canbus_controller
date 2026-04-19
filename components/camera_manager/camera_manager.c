#include "camera_manager.h"

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "host/ble_hs.h"

static const char *TAG = "camera_manager";

/* NVS namespace and key for a given slot */
static void slot_namespace(int slot, char *buf, size_t len)
{
    snprintf(buf, len, "cam_%d", slot);
}

#define NVS_CAMERA_KEY   "camera"

/* Persistent record layout — only fields saved to NVS */
typedef struct {
    char          camera_name[CAMERA_NAME_LEN];
    ble_addr_t    mac_address;
    bool          is_configured;
    camera_type_t type;
} camera_nv_record_t;

/* In-RAM camera slot structure */
typedef struct {
    char              name[CAMERA_NAME_LEN];
    char              model_name[CAMERA_MODEL_NAME_LEN]; /**< Populated from GetHardwareInfo after GATT is ready. */
    ble_addr_t        mac_address;
    bool              is_configured;
    camera_type_t     type;
    uint16_t          bt_handle;
    bool              gatt_ready;
    bool              desired_recording;
    const camera_driver_t *driver;
    void             *driver_ctx;
} camera_slot_t;

static camera_slot_t s_slots[CAMERA_MAX_SLOTS];

/* Automatic camera control flag.
 * When true (the default on every boot), the CAN logging state drives camera
 * recording.  When false, CAN transitions are ignored and the camera must be
 * controlled manually via the web UI.  Never persisted to NVS. */
static bool s_auto_control = true;

/* Driver registry for decoupled driver loading */
typedef struct {
    camera_type_t        type;
    const camera_driver_t *driver;
    void *(*create_ctx)(void);
} driver_registry_entry_t;

static driver_registry_entry_t s_registry[4];
static int s_registry_count = 0;

static esp_timer_handle_t s_tick_timer = NULL;

static camera_state_change_fn_t s_state_cb     = NULL;
static void                    *s_state_cb_ctx = NULL;

void camera_manager_register_state_change_callback(camera_state_change_fn_t cb, void *ctx)
{
    s_state_cb     = cb;
    s_state_cb_ctx = ctx;
}

/* -----------------------------------------------------------------------
 * State helpers
 * --------------------------------------------------------------------- */

/**
 * Derive the CAMERA_STATUS_* value for a slot from its current in-RAM state.
 * This mirrors the logic in camera_manager_get_slot_info() but returns a
 * plain int so callers don't need to allocate a full info struct.
 */
static int compute_slot_status(int i)
{
    camera_slot_t *slot = &s_slots[i];

    if (!slot->is_configured) {
        return CAMERA_STATUS_NOT_CONFIGURED;
    }
    if (slot->bt_handle == BLE_HS_CONN_HANDLE_NONE) {
        return CAMERA_STATUS_DISCONNECTED;
    }
    if (!slot->driver || !slot->gatt_ready) {
        /* Connected at the BLE level but GATT not yet set up. */
        return CAMERA_STATUS_CONNECTED;
    }
    camera_recording_status_t rec =
        slot->driver->get_recording_status(slot->driver_ctx);
    return (rec == CAMERA_RECORDING_ACTIVE) ? CAMERA_STATUS_RECORDING
                                             : CAMERA_STATUS_CONNECTED;
}

/** Notify the registered callback with the current status of slot i. */
static void notify_slot_state(int i)
{
    if (s_state_cb) {
        s_state_cb(i, compute_slot_status(i), s_state_cb_ctx);
    }
}

/* -----------------------------------------------------------------------
 * NVS Storage
 * --------------------------------------------------------------------- */

static void load_slot(int slot)
{
    char ns[16];
    slot_namespace(slot, ns, sizeof(ns));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return; /* No record yet */
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(%s) failed: %s", ns, esp_err_to_name(err));
        return;
    }

    camera_nv_record_t rec;
    size_t len = sizeof(rec);
    err = nvs_get_blob(handle, NVS_CAMERA_KEY, &rec, &len);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* Key was erased by camera_manager_remove_slot() or never written.
         * The namespace exists because IDF cannot delete namespaces, but the
         * slot is genuinely empty — not an error worth warning about. */
        nvs_close(handle);
        return;
    }

    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %d: nvs_get_blob failed (%s)", slot, esp_err_to_name(err));
        return;
    }

    if (len != sizeof(rec)) {
        ESP_LOGW(TAG, "slot %d: blob size mismatch (stored %zu bytes, expected %zu) — "
                 "schema changed or stale data; slot ignored, re-pair required",
                 slot, len, sizeof(rec));
        return;
    }

    /* Copy persistent fields into RAM record */
    memcpy(s_slots[slot].name, rec.camera_name, CAMERA_NAME_LEN);
    s_slots[slot].mac_address = rec.mac_address;
    s_slots[slot].is_configured = rec.is_configured;
    s_slots[slot].type = rec.type;

    /* Create driver context if driver is registered */
    for (int i = 0; i < s_registry_count; i++) {
        if (s_registry[i].type == rec.type) {
            s_slots[slot].driver = s_registry[i].driver;
            s_slots[slot].driver_ctx = s_registry[i].create_ctx();
            ESP_LOGI(TAG, "slot %d loaded: %s (type %d)", slot, s_slots[slot].name, rec.type);
            return;
        }
    }

    ESP_LOGW(TAG, "slot %d: no driver registered for type %d", slot, rec.type);
}

esp_err_t camera_manager_save_slot(int slot)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }

    camera_nv_record_t rec;
    memcpy(rec.camera_name, s_slots[slot].name, CAMERA_NAME_LEN);
    rec.mac_address = s_slots[slot].mac_address;
    rec.is_configured = s_slots[slot].is_configured;
    rec.type = s_slots[slot].type;

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
        ESP_LOGI(TAG, "slot %d saved: %s", slot, s_slots[slot].name);
    }
    return err;
}

esp_err_t camera_manager_remove_slot(int slot)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_slots[slot], 0, sizeof(camera_slot_t));
    s_slots[slot].bt_handle = BLE_HS_CONN_HANDLE_NONE;

    char ns[16];
    slot_namespace(slot, ns, sizeof(ns));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, NVS_CAMERA_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "slot %d removed", slot);
    return err;
}

/* -----------------------------------------------------------------------
 * Status polling
 * --------------------------------------------------------------------- */

static void camera_manager_tick(void *arg)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        camera_slot_t *slot = &s_slots[i];
        if (!slot->is_configured) continue;
        if (slot->bt_handle == BLE_HS_CONN_HANDLE_NONE) continue;
        if (!slot->gatt_ready) continue;
        if (!slot->driver) continue;

        if (slot->desired_recording) {
            camera_recording_status_t status =
                slot->driver->get_recording_status(slot->driver_ctx);
            if (status != CAMERA_RECORDING_ACTIVE) {
                slot->driver->start_recording(slot->driver_ctx);
            }
        }
    }

    /* Publish current status for every slot (including unconfigured ones) so
     * the CAN status broadcast always reflects the latest state. */
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        notify_slot_state(i);
    }
}

static void start_tick_timer(void)
{
    if (s_tick_timer != NULL) {
        return;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = camera_manager_tick,
        .arg      = NULL,
        .name     = "camera_tick",
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_tick_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tick timer: %s", esp_err_to_name(err));
        return;
    }

    err = esp_timer_start_periodic(s_tick_timer, 2000 * 1000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start tick timer: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Tick timer started");
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void camera_manager_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));

    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        s_slots[i].bt_handle = BLE_HS_CONN_HANDLE_NONE;
        load_slot(i);
    }

    start_tick_timer();
}

int camera_manager_find_by_addr(const ble_addr_t *addr)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (!s_slots[i].is_configured) continue;
        if (memcmp(&s_slots[i].mac_address, addr, sizeof(ble_addr_t)) == 0) {
            return i;
        }
    }
    return -1;
}

int camera_manager_find_free_slot(void)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (!s_slots[i].is_configured) return i;
    }
    return -1;
}

int camera_manager_remembered_count(void)
{
    int count = 0;
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (s_slots[i].is_configured) count++;
    }
    return count;
}

int camera_manager_connected_count(void)
{
    int count = 0;
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (s_slots[i].is_configured &&
            s_slots[i].bt_handle != BLE_HS_CONN_HANDLE_NONE) {
            count++;
        }
    }
    return count;
}

void camera_manager_on_connected(int slot, uint16_t conn_handle)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return;
    s_slots[slot].bt_handle = conn_handle;
    ESP_LOGI(TAG, "slot %d connected — handle: %d", slot, conn_handle);
    notify_slot_state(slot);
}

void camera_manager_on_disconnected(uint16_t conn_handle)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (s_slots[i].bt_handle == conn_handle) {
            s_slots[i].bt_handle = BLE_HS_CONN_HANDLE_NONE;
            s_slots[i].gatt_ready = false;
            ESP_LOGI(TAG, "slot %d disconnected", i);
            notify_slot_state(i);
            return;
        }
    }
}

void camera_manager_set_gatt_ready(int slot, bool ready)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return;
    s_slots[slot].gatt_ready = ready;
    ESP_LOGI(TAG, "slot %d gatt_ready = %s", slot, ready ? "true" : "false");
    notify_slot_state(slot);
}

void camera_manager_set_model_name(int slot, const char *model_name)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return;
    if (!model_name) return;
    strncpy(s_slots[slot].model_name, model_name, CAMERA_MODEL_NAME_LEN - 1);
    s_slots[slot].model_name[CAMERA_MODEL_NAME_LEN - 1] = '\0';
    ESP_LOGI(TAG, "slot %d model_name: %s", slot, s_slots[slot].model_name);
}

int camera_manager_register_new(const ble_addr_t *addr, const char *name,
                                 const camera_driver_t *driver, void *driver_ctx,
                                 camera_type_t type)
{
    int slot = camera_manager_find_by_addr(addr);
    if (slot >= 0) {
        return slot; /* Already registered */
    }

    slot = camera_manager_find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "No free camera slots");
        return -1;
    }

    memcpy(s_slots[slot].name, name, CAMERA_NAME_LEN - 1);
    s_slots[slot].name[CAMERA_NAME_LEN - 1] = '\0';
    s_slots[slot].mac_address = *addr;
    s_slots[slot].is_configured = true;
    s_slots[slot].type = type;
    s_slots[slot].driver = driver;
    s_slots[slot].driver_ctx = driver_ctx;
    s_slots[slot].bt_handle = BLE_HS_CONN_HANDLE_NONE;
    s_slots[slot].gatt_ready = false;

    ESP_LOGI(TAG, "slot %d registered: %s", slot, s_slots[slot].name);
    return slot;
}

int camera_manager_find_by_handle(uint16_t conn_handle)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return -1;
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        if (s_slots[i].bt_handle == conn_handle) {
            return i;
        }
    }
    return -1;
}

uint16_t camera_manager_get_handle(int slot)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return BLE_HS_CONN_HANDLE_NONE;
    return s_slots[slot].bt_handle;
}

bool camera_manager_is_gatt_ready(int slot)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return false;
    return s_slots[slot].gatt_ready;
}

void *camera_manager_get_driver_ctx(int slot)
{
    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return NULL;
    return s_slots[slot].driver_ctx;
}

camera_slot_info_t camera_manager_get_slot_info(int slot)
{
    camera_slot_info_t info = {
        .index = slot,
        .is_configured = false,
        .status = CAMERA_STATUS_NOT_CONFIGURED,
    };

    if (slot < 0 || slot >= CAMERA_MAX_SLOTS) return info;

    memcpy(info.name, s_slots[slot].name, CAMERA_NAME_LEN);
    memcpy(info.model_name, s_slots[slot].model_name, CAMERA_MODEL_NAME_LEN);
    info.mac_address = s_slots[slot].mac_address;
    info.is_configured = s_slots[slot].is_configured;

    if (!info.is_configured) {
        return info;
    }

    if (s_slots[slot].bt_handle == BLE_HS_CONN_HANDLE_NONE) {
        info.status = CAMERA_STATUS_DISCONNECTED;
    } else if (s_slots[slot].driver && s_slots[slot].gatt_ready) {
        camera_recording_status_t rec_status =
            s_slots[slot].driver->get_recording_status(s_slots[slot].driver_ctx);
        if (rec_status == CAMERA_RECORDING_ACTIVE) {
            info.status = CAMERA_STATUS_RECORDING;
        } else {
            info.status = CAMERA_STATUS_CONNECTED;
        }
    } else {
        info.status = CAMERA_STATUS_CONNECTED;
    }

    return info;
}

void camera_manager_set_desired_recording(bool recording)
{
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        s_slots[i].desired_recording = recording;
    }
}

int camera_manager_start_recording_all(void)
{
    int dispatched = 0;
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        camera_slot_t *slot = &s_slots[i];
        if (!slot->is_configured) continue;
        if (slot->bt_handle == BLE_HS_CONN_HANDLE_NONE) continue;
        if (!slot->gatt_ready) continue;
        if (!slot->driver) continue;

        if (slot->driver->start_recording(slot->driver_ctx) == ESP_OK) {
            dispatched++;
        }
    }
    return dispatched;
}

int camera_manager_stop_recording_all(void)
{
    int dispatched = 0;
    for (int i = 0; i < CAMERA_MAX_SLOTS; i++) {
        camera_slot_t *slot = &s_slots[i];
        if (!slot->is_configured) continue;
        if (slot->bt_handle == BLE_HS_CONN_HANDLE_NONE) continue;
        if (!slot->gatt_ready) continue;
        if (!slot->driver) continue;

        if (slot->driver->stop_recording(slot->driver_ctx) == ESP_OK) {
            dispatched++;
        }
    }
    return dispatched;
}

bool camera_manager_is_known_addr(const ble_addr_t *addr)
{
    return camera_manager_find_by_addr(addr) >= 0;
}

void camera_manager_set_auto_control(bool enabled)
{
    s_auto_control = enabled;
    ESP_LOGI(TAG, "automatic_camera_control = %s", enabled ? "true" : "false");
}

bool camera_manager_get_auto_control(void)
{
    return s_auto_control;
}

bool camera_manager_has_disconnected_cameras(void)
{
    return camera_manager_remembered_count() > camera_manager_connected_count();
}

void camera_manager_register_driver(camera_type_t type,
                                     const camera_driver_t *driver,
                                     void *(*create_ctx)(void))
{
    if (s_registry_count >= 4) {
        ESP_LOGW(TAG, "Driver registry full");
        return;
    }

    s_registry[s_registry_count].type = type;
    s_registry[s_registry_count].driver = driver;
    s_registry[s_registry_count].create_ctx = create_ctx;
    s_registry_count++;

    ESP_LOGI(TAG, "Driver registered for type %d", type);
}
