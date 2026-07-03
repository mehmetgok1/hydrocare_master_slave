idf_component_register(SRCS "config.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        esp_driver_gpio
                        esp_driver_spi
                        esp_adc
                        led_strip
)
