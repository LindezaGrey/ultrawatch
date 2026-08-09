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

esp_err_t cst9217_init(i2c_master_bus_handle_t bus);

/* Native coordinate resolution read from the chip (0xD1F8). */
esp_err_t cst9217_get_resolution(uint16_t *x, uint16_t *y);

/* The esp_lcd_touch handle (NULL until init succeeds). */
esp_lcd_touch_handle_t cst9217_get_handle(void);

#ifdef __cplusplus
}
#endif
