idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES camera_manager open_gopro_ble ble_core nvs_flash wifi_manager can_manager legacy_gopro)
