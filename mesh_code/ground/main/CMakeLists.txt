idf_component_register(SRCS "mesh_main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash esp_wifi esp_driver_uart esp_driver_gpio)
