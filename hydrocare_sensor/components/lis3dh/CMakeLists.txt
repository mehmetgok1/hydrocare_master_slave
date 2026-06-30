idf_component_register(SRCS "lis3dh.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc esp_driver_gpio esp_driver_spi esp_timer
)