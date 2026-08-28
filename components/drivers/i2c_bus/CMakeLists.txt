idf_component_register(
    SRCS "i2c_bus.c" "i2c_bus_frame.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_common
)
