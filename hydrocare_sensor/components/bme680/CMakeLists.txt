idf_component_register(SRCS "bme680.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES spi_flash
                    REQUIRES esp_adc esp_driver_gpio esp_driver_spi)
