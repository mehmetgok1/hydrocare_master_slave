idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES spi_flash
                    REQUIRES esp_adc 
                        esp_driver_gpio 
                        config
                        bme680
                        esp32-camera
                        communication
                    )
