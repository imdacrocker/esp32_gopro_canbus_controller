#include "nvs_flash.h"
#include "ble_core.h"
#include "gopro_ble.h"
#include "camera_manager.h"
#include "wifi_manager.h"

void app_main(void)
{
    /* NVS is required by the BLE stack for storing bonding info. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Register camera drivers BEFORE camera_manager_init() loads NVS so that
     * the driver factory is available when stored camera records are rehydrated
     * into driver contexts. */
    gopro_ble_init();

    /* Load remembered cameras from NVS into RAM and start the tick timer. */
    camera_manager_init();

    wifi_manager_init();

    /* Start the NimBLE stack.  on_sync fires once the stack is ready and
     * kicks off the boot reconnect chain. */
    ble_core_init();
}
