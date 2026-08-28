# Host build for pcf85063a_time's tests. No ESP-IDF, no idf.py - plain
# CMake, same convention as sim/CMakeLists.txt for host-side testing.
cmake_minimum_required(VERSION 3.16)
project(test_pcf85063a_time C)

set(CMAKE_C_STANDARD 11)

add_executable(test_pcf85063a_time
    test_pcf85063a_time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../pcf85063a_time.c
)
target_include_directories(test_pcf85063a_time PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

enable_testing()
add_test(NAME pcf85063a_time COMMAND test_pcf85063a_time)
