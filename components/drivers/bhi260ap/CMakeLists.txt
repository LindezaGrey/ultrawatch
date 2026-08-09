idf_component_register(
    SRCS "bhi260ap.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        log
)
