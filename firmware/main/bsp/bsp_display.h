#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_display_init(void);
esp_lcd_panel_handle_t bsp_display_get_panel(void);
esp_err_t bsp_display_wait_flush_done(void);
esp_err_t bsp_display_set_brightness(uint8_t percent);
void bsp_display_show_test_pattern(void);
void bsp_display_xfer_begin(void);
void bsp_display_get_stats(uint32_t *starts, uint32_t *completions, uint32_t *overlaps);

#ifdef __cplusplus
}
#endif
