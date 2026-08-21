idf_component_register(SRCS "src/soldered_l86_m33.c" "src/l86_m33_nmea.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_timer)
