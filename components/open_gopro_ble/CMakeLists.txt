idf_component_register(SRCS "driver.c" "control.c" "gatt.c" "pairing.c" "notify.c" "readiness.c"
                       INCLUDE_DIRS "include"
                       REQUIRES camera_manager ble_core esp_timer bt)
