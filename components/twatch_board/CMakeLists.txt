idf_component_register(
    SRCS "twatch_board.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_driver_spi
        esp_driver_gpio
        esp_driver_i2s
        esp_common
        log
    PRIV_REQUIRES
        axp2101
        xl9555
        spi2_power
        sx1262
        freertos
)
