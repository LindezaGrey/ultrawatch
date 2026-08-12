/*
 * t3902.h - TDK T3902 PDM microphone.
 *
 * Captured through the Espressif I2S driver in PDM RX mode (hardware
 * PDM->PCM decimation, 16-bit mono). The channel handle is created by
 * twatch_board (shared I2S0 with the amp TX channel) and passed in.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2s_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define T3902_PIN_CLK  17
#define T3902_PIN_DAT  18

#define AUDIO_SAMPLE_RATE  16000   /* mono 16-bit sample rate */

/* Configure the given I2S0 RX channel for PDM mode (clk=17, dat=18) and start
 * it. Safe to call once. */
esp_err_t t3902_init(i2s_chan_handle_t rx_handle);

/* Read up to n mono 16-bit PCM samples from the mic (blocking). */
esp_err_t t3902_read(int16_t *samples, size_t n);

#ifdef __cplusplus
}
#endif
