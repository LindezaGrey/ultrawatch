/*
 * touch_controller.h - CST9217 lifecycle and touch wake boundary.
 *
 * LVGL owns input-device registration and gestures.  power_mgmt owns the
 * policy for whether touch may wake the watch; this controller performs the
 * corresponding GPIO wake configuration.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Reset and initialise the CST9217. Safe to call once during startup. */
esp_err_t touch_controller_init(void);

/* NULL until the CST9217 has initialised successfully. */
esp_lcd_touch_handle_t touch_controller_get_handle(void);

/* Native coordinate resolution reported by the CST9217. */
esp_err_t touch_controller_get_resolution(uint16_t *x, uint16_t *y);

/* Arm or disarm GPIO12 as an active-low light-sleep wake source. */
esp_err_t touch_controller_set_wake_enabled(bool enabled);

#ifdef __cplusplus
}
#endif
