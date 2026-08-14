#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Panel brightness percentages. Day mode (casio dark) uses 30%; the
 * low-power red mode is kept on black with red-only subpixels. */
#define UI_DAY_MODE_BRIGHTNESS_PCT 30
#define UI_RED_MODE_BRIGHTNESS_PCT 30

esp_err_t ui_clock_init(void);

/* Cycle to the next display mode (short press / LVGL context) */
void ui_mode_cycle(void);

/* Select a display mode by index (0..2, LVGL context) */
void ui_mode_select(int index);

/* Console-safe variants: defer the LVGL work to the next UI tick */
void ui_mode_cycle_request(void);
void ui_mode_select_request(int index);

/* Trigger a screenshot capture and UART dump (console command `shot`) */
void ui_shot_trigger(void);

/* TE-sync statistics (for diagnostics) */
void ui_get_te_stats(uint32_t *calls, uint32_t *timeouts, uint32_t *max_us, uint32_t *last_us);

#ifdef __cplusplus
}
#endif
