idf_component_register(
    SRCS "sd_log.c"
    INCLUDE_DIRS "include"
    REQUIRES
        fatfs
        sdmmc
        esp_driver_sdspi
        esp_driver_gpio
        espressif__libpng
        log
        esp_common
        twatch_board
        axp2101
)
