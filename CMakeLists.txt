cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET esp32s3)
set(PROJECT_VER "0.5.6-row-aligned-flush")
execute_process(COMMAND git -C "$ENV{IDF_PATH}" rev-parse HEAD
    OUTPUT_VARIABLE IDF_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE IDF_REVISION_RESULT)
if(NOT IDF_REVISION_RESULT EQUAL 0 OR
   NOT IDF_REVISION STREQUAL "7101770dc6db2667b3c477cc31365dd1acd6db4e")
    message(FATAL_ERROR "Use the pinned ESP-IDF 6.0.2 checkout; see tools/sources.lock.json")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(twatch_ultra_phase1)
spiffs_create_partition_image(storage littlefs FLASH_IN_PROJECT)
