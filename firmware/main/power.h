#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t power_sleep_init(void);

/* Called from the main loop each iteration. Enters light sleep when the UI is
 * idle for a while, waking on the minute boundary (timer) or touch (EXT1).
 * Returns true if it slept (caller should re-run the LVGL tick immediately). */
bool power_sleep_tick(void);

void power_set_sleep_enabled(bool enable);
bool power_get_sleep_enabled(void);

/* Notify the sleep engine that the user touched the screen (stays awake). */
void power_set_interactive(void);

/* Track the current (non-sleep) panel brightness so it can be restored. */
void power_set_user_brightness(uint8_t percent);

/* Diagnostics counters */
void power_get_stats(uint32_t *sleeps, uint32_t *wake_timer, uint32_t *wake_ext1);

#ifdef __cplusplus
}
#endif
