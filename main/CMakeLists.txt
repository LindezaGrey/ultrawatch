idf_component_register(SRCS "main.c" "battery_observation.c" "lvgl_app.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_app_format esp_system esp_psram spi_flash heap freertos twatch_board
        brookesia_twatch_bridge esp_timer lvgl__lvgl)
