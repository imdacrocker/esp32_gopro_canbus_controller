idf_component_register(SRCS "ble_init.c" "ble_scan.c" "ble_connect.c" "ble_gatt_write.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash freertos)
