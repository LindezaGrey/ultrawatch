idf_component_register(
    SRCS "m10q.c"
    INCLUDE_DIRS "include"
    REQUIRES
        ubxlib
        esp_common
        log
    PRIV_REQUIRES
        axp2101
        pcf85063a
        nvs_flash
)
