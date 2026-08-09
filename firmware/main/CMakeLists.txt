idf_component_register(
    SRCS "app_main.c" "ble_rtc.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES bt driver nvs_flash
)
