idf_component_register(SRCS "main.c" "battery_observation.c" "lvgl_app.c" "brookesia_system_entry.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_app_format esp_system esp_psram spi_flash heap freertos twatch_board
        brookesia_twatch_bridge bhi260ap_core esp_timer lvgl__lvgl)
