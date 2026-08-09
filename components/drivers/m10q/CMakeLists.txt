idf_component_register(
    SRCS "m10q.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_uart
        esp_common
        log
)
