idf_component_register(
    SRCS "bhi260ap.c" "bhy2.c" "bhy2_hif.c" "bhy2_parse.c"
    INCLUDE_DIRS "include" "."
    REQUIRES
        esp_driver_i2c
        esp_driver_gpio
        nvs_flash
        esp_common
        log
        spiffs
        esp_timer
    PRIV_REQUIRES
        i2c_bus
)
