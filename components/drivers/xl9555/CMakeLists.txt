idf_component_register(
    SRCS "xl9555.c" "xl9555_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        log
    PRIV_REQUIRES
        i2c_bus
        freertos
)
