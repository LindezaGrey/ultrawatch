idf_component_register(
    SRCS "axp2101.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        log
)
