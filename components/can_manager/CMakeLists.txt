idf_component_register(
    SRCS "can_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_driver_twai" "freertos" "log" "esp_timer"
)
