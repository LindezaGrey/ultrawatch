/*
 * cst9217.h - CST9217 capacitive touch panel (I2C 0x1A).
 *
 * Wraps the `waveshare/esp_lcd_touch_cst9217` driver and exposes the
 * esp_lcd_touch handle for the LVGL adapter. The T-Watch Ultra touch is at
 * I2C 0x1A (the Waveshare driver default is 0x5A, overridden here); its
 * reset is on the XL9555 expander (P8), driven by the board layer.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CST9217_I2C_ADDR 0x1A
#define CST9217_TOUCH_INT_GPIO 12

esp_err_t cst9217_init(i2c_master_bus_handle_t bus);

/* Native coordinate resolution. */
esp_err_t cst9217_get_resolution(uint16_t *x, uint16_t *y);

/* LVGL adapter custom-read callback; returns errors for invalid frames so the
 * adapter publishes a released state instead of stale coordinates. */
esp_err_t cst9217_read_points(esp_lcd_touch_handle_t touch,
                               esp_lcd_touch_point_data_t *points, uint8_t *count,
                               uint8_t max_count, void *user_ctx);

/* The esp_lcd_touch handle (NULL until init succeeds). */
esp_lcd_touch_handle_t cst9217_get_handle(void);

#ifdef __cplusplus
}
#endif
