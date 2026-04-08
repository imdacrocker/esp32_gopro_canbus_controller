idf_component_register(SRCS "camera_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash esp_timer freertos)
