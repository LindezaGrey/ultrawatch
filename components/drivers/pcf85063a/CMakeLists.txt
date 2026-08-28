idf_component_register(
    SRCS "pcf85063a.c" "pcf85063a_time.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        log
)
