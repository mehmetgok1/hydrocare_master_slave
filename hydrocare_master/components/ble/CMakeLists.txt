file(GLOB_RECURSE srcs "main.c" "src/*.c")
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "." "./include"
                    REQUIRES
                        sdcard
                        wifi_stream
                        ui
                        ota
                        bt
                        nvs_flash                      
)
