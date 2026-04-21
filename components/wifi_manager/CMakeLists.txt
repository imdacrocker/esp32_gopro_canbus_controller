idf_component_register(SRCS "wifi_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_http_server esp_netif esp_event
                                camera_manager open_gopro_ble ble_core legacy_gopro
                       EMBED_TXTFILES "www/index.html")
