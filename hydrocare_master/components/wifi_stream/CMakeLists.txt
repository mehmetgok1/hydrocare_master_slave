idf_component_register(SRCS "wifi_stream.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES cjson
                    REQUIRES
                        esp_timer
                        esp_http_client 
                        config
                        ota
                        esp_timer
                        sdcard
                        ble
)
