#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SCREEN_TOUCH_DOWN = 1,
    SCREEN_TOUCH_MOVE = 2,
    SCREEN_TOUCH_UP = 3,
} screen_touch_event_t;

/* Set the AMOLED DCS brightness level using a percentage from 0 through 100. */
esp_err_t screen_set_brightness(uint8_t percentage);

/* Return the last brightness level successfully sent to the AMOLED. */
uint8_t screen_get_brightness(void);

/* Ask the display task to redraw time, battery, and BLE state immediately. */
void screen_request_refresh(void);

/* Wake the black screen and restart its touch-inactivity deadline. */
void screen_wake_from_touch(void);

/* Route a CST9217 coordinate event into the window manager. */
void screen_handle_touch(screen_touch_event_t event, uint16_t x, uint16_t y);
