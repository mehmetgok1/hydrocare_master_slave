idf_component_register(SRCS "config.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        # Drivers used for peripherals
                        esp_driver_gpio
                        esp_driver_spi
                        esp_driver_i2c
                        esp_adc
                        # Other components used
                        esp32-camera
                        bme680
)