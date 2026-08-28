# Host build for i2c_bus_frame's tests. No ESP-IDF, no idf.py - plain
# CMake, same convention as sim/CMakeLists.txt and pcf85063a/test/.
cmake_minimum_required(VERSION 3.16)
project(test_i2c_bus_frame C)

set(CMAKE_C_STANDARD 11)

add_executable(test_i2c_bus_frame
    test_i2c_bus_frame.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../i2c_bus_frame.c
)
target_include_directories(test_i2c_bus_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

enable_testing()
add_test(NAME i2c_bus_frame COMMAND test_i2c_bus_frame)
