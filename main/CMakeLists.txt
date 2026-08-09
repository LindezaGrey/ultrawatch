idf_component_register(
    SRCS "uwatch_main.c" "lvgl_app.c"
    INCLUDE_DIRS "."
    REQUIRES
        twatch_board
        axp2101
        xl9555
        co5300
        esp_lvgl_adapter
        lvgl
        spiffs
        esp_driver_i2c
        esp_common
        esp_timer
        log
)

# Pack the assets/ folder (Roboto TTF, UI overlay) into the SPIFFS partition.
spiffs_create_partition_image(assets ../assets FLASH_IN_PROJECT)
