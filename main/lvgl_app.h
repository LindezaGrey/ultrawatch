/*
 * lvgl_app.h - LVGL integration via the Espressif esp_lvgl_adapter.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lvgl_app_start(void);

/* Invalidate the active LVGL screen so it repaints fully (used after wake,
 * when the panel GRAM was blanked during sleep). Safe to call from any task. */
void lvgl_force_redraw(void);

/* Capture the active LVGL screen and print it to the console as base64 RGB565
 * between ==SHOT:WxH== and ==ENDSHOT== markers (debug). Call from any task. */
esp_err_t lvgl_app_dump_screenshot(void);

#ifdef __cplusplus
}
#endif
