/*
 * max98357a.h - Analog MAX98357A Class-D amp (I2S PCM output).
 *
 * I2S TX channel on dedicated pins (BCLK 9, WCLK 10, DOUT 11) using
 * the Espressif I2S standard-mode driver. The channel handle is created by
 * twatch_board (shared I2S0 with the PDM mic RX channel) and passed in.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2s_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX98357A_PIN_BCLK 9
#define MAX98357A_PIN_WCLK 10
#define MAX98357A_PIN_DOUT 11

#define AUDIO_SAMPLE_RATE  16000   /* mono 16-bit sample rate */

/* Configure the given I2S0 TX channel for the amp (std mode, mono) and start
 * it. Safe to call once. */
esp_err_t max98357a_init(i2s_chan_handle_t tx_handle);

/* Write n mono 16-bit samples to the amp (blocking). */
esp_err_t max98357a_write(const int16_t *data, size_t n);

/* The MAX98357A has no software volume control (gain is fixed by a resistor);
 * this is a documented no-op returning ESP_OK. */
esp_err_t max98357a_set_volume(float db);

#ifdef __cplusplus
}
#endif
