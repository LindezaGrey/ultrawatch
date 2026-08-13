# ubxlib component (u-blox host library) - GNSS feature only.
# Vendored from https://github.com/u-blox/ubxlib v1.5.0.
get_filename_component(UBXLIB_BASE ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(PLATFORM_DIR ${UBXLIB_BASE}/port/platform/esp-idf ABSOLUTE)

set(UBXLIB_FEATURES gnss CACHE STRING "ubxlib features to build")

# Collect the ubxlib sources/includes based on the feature set.
include(${UBXLIB_BASE}/port/ubxlib.cmake)

list(APPEND UBXLIB_INC
    ${PLATFORM_DIR}
    ${PLATFORM_DIR}/mcu/esp32/cfg
    ${UBXLIB_BASE}/port/clib
)

list(APPEND UBXLIB_PRIVATE_INC
    ${PLATFORM_DIR}/src
    ${UBXLIB_PRIVATE_INC}
)

idf_component_register(
    SRCS
        ${PLATFORM_DIR}/src/u_port.c
        ${PLATFORM_DIR}/src/u_port_debug.c
        ${PLATFORM_DIR}/src/u_port_gpio.c
        ${PLATFORM_DIR}/src/u_port_os.c
        ${PLATFORM_DIR}/src/u_port_uart.c
        ${PLATFORM_DIR}/src/u_port_i2c.c
        ${PLATFORM_DIR}/src/u_port_spi.c
        ${PLATFORM_DIR}/src/u_port_ppp.c
        ${PLATFORM_DIR}/src/u_port_private.c
        ${UBXLIB_BASE}/port/clib/u_port_clib_mktime64.c
        ${UBXLIB_BASE}/port/u_port_timezone.c
        ${UBXLIB_SRC}
    INCLUDE_DIRS
        ${UBXLIB_INC}
    PRIV_INCLUDE_DIRS
        ${UBXLIB_PRIVATE_INC}
    REQUIRES
        driver
        esp_driver_uart
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_spi
        esp_netif
        esp_event
        esp_timer
        esp_system
        esp_common
        log
        freertos
)
