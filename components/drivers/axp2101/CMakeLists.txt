idf_component_register(
    SRCS "axp2101.c" "axp2101_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        log
    PRIV_REQUIRES
        i2c_bus
)
