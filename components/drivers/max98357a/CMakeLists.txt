idf_component_register(
    SRCS "max98357a.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2s
        esp_common
        log
)
