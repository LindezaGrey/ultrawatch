idf_component_register(SRCS "twatch_board.c" "twatch_display.c" INCLUDE_DIRS "include"
    REQUIRES esp_lcd
    PRIV_REQUIRES esp_timer esp_driver_i2c esp_driver_spi esp_driver_gpio)
