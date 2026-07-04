idf_component_register(SRCS "communication.c"
                    INCLUDE_DIRS "." 
                    REQUIRES
                        config
                        wifi_stream
                        ui
                        ota
                        bt
                        nvs_flash   
                        esp_timer                   
)
