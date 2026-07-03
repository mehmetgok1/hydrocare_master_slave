idf_component_register(SRCS "config.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        esp_driver_gpio
                        esp_driver_spi
                        esp_driver_i2c
                        esp_adc
                        esp_driver_ledc
                        esp32-camera
                        bme680
                        lis3dh
                        mlx90641
)