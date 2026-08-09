idf_component_register(
    SRCS "t3902.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2s
        esp_common
        log
)
