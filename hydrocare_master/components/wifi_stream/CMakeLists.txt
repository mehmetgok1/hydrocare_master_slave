idf_component_register(SRCS "wifi_stream.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        config
                        ota
                        esp_timer
                        sdcard
)
