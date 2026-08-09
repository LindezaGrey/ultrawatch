idf_component_register(
    SRCS "co5300.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd_sh8601
        esp_lcd
        esp_driver_spi
        esp_driver_gpio
        esp_common
        freertos
        log
)
