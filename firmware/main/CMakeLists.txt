idf_component_register(
    SRCS "app_main.c" "ble_rtc.c" "gps.c"
         "vendor/bhy2/bhy2.c" "vendor/bhy2/bhy2_hif.c"
         "vendor/bhy2/bhy2_parse.c"
    INCLUDE_DIRS "." "vendor/bhy2"
    PRIV_REQUIRES bt driver esp_driver_i2s esp_pm esp_hw_support nvs_flash fatfs sdmmc
)

target_add_binary_data(${COMPONENT_LIB}
    "vendor/bhy2/firmware/BHI260AP.fw" BINARY)
