#include "nvs_flash.h"
#include "ble_scanner.h"
#include "wifi_manager.h"
#include "gopro_manager.h"

void app_main(void)
{
    /* NVS is required by the BLE stack for storing bonding info etc. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Load remembered cameras from NVS into RAM */
    gopro_manager_init();

    wifi_manager_init();

    ble_init();
}