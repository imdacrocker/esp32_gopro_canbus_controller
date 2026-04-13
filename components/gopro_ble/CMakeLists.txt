idf_component_register(SRCS "gopro_readiness.c" "gopro_gatt.c" "gopro_pairing.c" "gopro_notify.c" "gopro_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES camera_manager ble_core bt esp_timer)
