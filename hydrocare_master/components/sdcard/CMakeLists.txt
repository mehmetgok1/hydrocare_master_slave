idf_component_register(SRCS "sd.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        config
                        fatfs 
                        sd_card
                        WHOLE_ARCHIVE
)
