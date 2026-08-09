idf_component_register(
    SRCS "st25r3916.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
        esp_common
        log
)
