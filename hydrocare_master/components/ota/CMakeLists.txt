idf_component_register(SRCS "ota.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        config
                        esp_wifi 
                        esp_event 
                        nvs_flash 
                        esp_http_client 
                        esp_https_ota 
                        app_update
                        mbedtls
)
