idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        twatch_board
        axp2101
        bhi260ap
        co5300
        cst9217
        drv2605
        i2c_bus
        m10q
        max98357a
        pcf85063a
        spi2_power
        st25r3916
        sx1262
        t3902
        xl9555
        nvs_flash
        esp_common
        log
)
