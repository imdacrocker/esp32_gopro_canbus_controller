idf_component_register(SRCS "driver.c" "control.c" "gatt.c" "pairing.c" "notify.c" "query.c"
                       INCLUDE_DIRS "include"
                       REQUIRES camera_manager ble_core esp_timer bt)
