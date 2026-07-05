file(GLOB_RECURSE srcs "*.c" "src/*.c")
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "." "./include"
                    REQUIRES
                        sdcard
                        wifi_stream
                        communication
                        measurement
                        ui
                        ota
                        timer
                        bt
                        nvs_flash                      
)
