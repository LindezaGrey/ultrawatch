idf_component_register(
    SRCS "sx1262.c" "meshtastic_radio.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
        esp_common
        log
)
