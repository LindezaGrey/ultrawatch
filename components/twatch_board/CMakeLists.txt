idf_component_register(
    SRCS "twatch_board.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_driver_spi
        esp_driver_gpio
        esp_common
        log
    PRIV_REQUIRES
        axp2101
        xl9555
        pcf85063a
        cst9217
        bhi260ap
        drv2605
        co5300
        max98357a
        t3902
        m10q
        sx1262
        st25r3916
        freertos
)
