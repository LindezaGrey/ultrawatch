/*
 * t3902.h - TDK T3902 PDM microphone.
 *
 * Captured through the Espressif I2S driver in PDM RX mode.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t t3902_init(void);
esp_err_t t3902_read(int16_t *samples, size_t n);

#ifdef __cplusplus
}
#endif
