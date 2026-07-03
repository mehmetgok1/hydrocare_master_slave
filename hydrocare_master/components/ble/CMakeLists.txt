idf_component_register(SRCS "ble.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES measurement timer ota ui memory config communication wifi_stream
                    # PRIV_REQUIRES are for linking private dependencies of components
                    # The esp_bt component is needed for the native bluedroid stack
                    PRIV_REQUIRES esp_wifi esp_bt
                    )