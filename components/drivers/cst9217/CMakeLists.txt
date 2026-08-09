idf_component_register(
    SRCS "cst9217.c"
    INCLUDE_DIRS "include"
    REQUIRES
        waveshare__esp_lcd_touch_cst9217
        esp_lcd
        esp_lcd_touch
        esp_driver_i2c
        esp_common
        freertos
        log
)
