idf_component_register(
    SRCS "spi2_power.c"
    INCLUDE_DIRS "include"
    REQUIRES
        axp2101
        esp_common
        log
)
