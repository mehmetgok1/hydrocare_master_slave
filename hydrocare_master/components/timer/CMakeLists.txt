idf_component_register(SRCS "timer.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        config
                        esp_driver_gptimer
)
