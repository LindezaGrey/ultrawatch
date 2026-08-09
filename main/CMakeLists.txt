idf_component_register(
    SRCS "uwatch_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        twatch_board
        axp2101
        xl9555
        co5300
        esp_driver_i2c
        esp_common
        esp_timer
        log
)
