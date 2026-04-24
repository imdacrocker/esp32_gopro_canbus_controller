idf_component_register(SRCS "legacy_gopro.c"
                             "control.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_client camera_manager freertos lwip nvs_flash
                                can_manager wifi_manager)
