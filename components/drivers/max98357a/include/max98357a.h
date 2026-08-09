/*
 * max98357a.h - Analog MAX98357A Class-D amp (I2S PCM output).
 *
 * I2S TX channel on dedicated pins (BCLK 9, WCLK 10, DOUT 11) using
 * the Espressif I2S standard-mode driver.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX98357A_PIN_BCLK 9
#define MAX98357A_PIN_WCLK 10
#define MAX98357A_PIN_DOUT 11

esp_err_t max98357a_init(void);
esp_err_t max98357a_write(const int16_t *data, size_t n);
esp_err_t max98357a_set_volume(float db);

#ifdef __cplusplus
}
#endif
