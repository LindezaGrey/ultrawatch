idf_component_register(
    SRCS "bhi260ap.c" "bhy2.c" "bhy2_hif.c" "bhy2_parse.c"
    INCLUDE_DIRS "include" "."
    REQUIRES
        esp_driver_i2c
        esp_common
        log
        spiffs
)
