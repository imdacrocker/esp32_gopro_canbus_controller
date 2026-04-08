idf_component_register(SRCS "gopro_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash ble_scanner)