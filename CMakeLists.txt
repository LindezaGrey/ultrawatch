cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    components/drivers
    components/meshtastic
    components/spi2_power
    components/twatch_board
    components/ubxlib
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(UWatchHardwareFoundation)
