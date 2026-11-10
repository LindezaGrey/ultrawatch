idf_component_register(
    SRCS
        "meshtastic_radio.c"
        "meshtastic_data.c"
        "meshtastic_user.c"
        "meshtastic_crypto.c"
        "meshtastic_packet.c"
    INCLUDE_DIRS "include"
    REQUIRES sx1262 esp_common log
    PRIV_REQUIRES mbedtls
)
