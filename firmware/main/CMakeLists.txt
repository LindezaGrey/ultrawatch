idf_component_register(
    SRCS "app_main.c" "ble_rtc.c" "gps.c" "offline_map.c" "sd_storage.c"
         "wifi_manager.c" "weather.c" "shared_spi2.c" "mesh_service.c"
         "sx1262.c" "meshtastic_radio.c" "meshtastic_crypto.c"
         "meshtastic_packet.c" "meshtastic_data.c" "meshtastic_user.c"
         "vendor/bhy2/bhy2.c" "vendor/bhy2/bhy2_hif.c"
         "vendor/bhy2/bhy2_parse.c"
    INCLUDE_DIRS "." "vendor/bhy2"
    PRIV_REQUIRES bt driver esp_driver_i2s esp_pm esp_hw_support nvs_flash fatfs sdmmc esp_wifi esp_event esp_netif json esp_http_client mbedtls
)

target_add_binary_data(${COMPONENT_LIB}
    "vendor/bhy2/firmware/BHI260AP.fw" BINARY)
