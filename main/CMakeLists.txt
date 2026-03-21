set(SOURCES "main.c")

idf_component_register(SRCS "${SOURCES}"
                    INCLUDE_DIRS "."
                    REQUIRES bt wifi_provisioning esp_http_server nvs_flash esp_wifi esp_driver_tsens)
