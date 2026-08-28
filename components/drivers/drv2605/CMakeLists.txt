idf_component_register(
    SRCS "drv2605.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
        log
    PRIV_REQUIRES
        nvs_flash
        i2c_bus
)