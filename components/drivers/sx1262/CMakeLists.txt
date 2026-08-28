idf_component_register(
    SRCS "sx1262.c" "meshtastic_radio.c" "meshtastic_data.c" "meshtastic_user.c" "meshtastic_crypto.c" "meshtastic_packet.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
        esp_common
        log
    PRIV_REQUIRES
        mbedtls
)
