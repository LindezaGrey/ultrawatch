/*
 * portable_log.h - ESP_LOGx shim so files under main/screens/ compile
 * unchanged in both the ESP-IDF firmware build and the host sim build
 * (sim/CMakeLists.txt defines UWATCH_SIM).
 */
#pragma once

#ifdef UWATCH_SIM
#include <stdio.h>
#define ESP_LOGI(tag, fmt, ...) printf("I (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("W (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("E (%s) " fmt "\n", tag, ##__VA_ARGS__)
#else
#include "esp_log.h"
#endif
