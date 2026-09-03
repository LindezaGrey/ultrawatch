#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* The audio task is the sole owner of MAX98357A/I2S TX and the speaker rail. */
esp_err_t audio_init(void);

/* Alarm audio has priority over ordinary PCM playback. */
esp_err_t audio_alarm_start(int16_t amplitude);
esp_err_t audio_alarm_stop(void);

/* Plays PCM synchronously. An alarm may preempt it. Samples are copied before
 * the request is queued, so the caller retains ownership of its buffer. */
esp_err_t audio_play_pcm(const int16_t *samples, size_t sample_count);
