# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Discover the peripheral driver components nested under components/drivers/
set(EXTRA_COMPONENT_DIRS components/drivers)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(UWatch)
