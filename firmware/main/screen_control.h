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

/* Set the RGB color used for contours, rings, and highlighted text. */
esp_err_t screen_set_theme_color(uint8_t red, uint8_t green, uint8_t blue);

/* Return the current RGB theme color. */
void screen_get_theme_color(uint8_t *red, uint8_t *green, uint8_t *blue);

/* Ask the display task to redraw time, battery, and BLE state immediately. */
void screen_request_refresh(void);

/* Wake the display and open the messaging app for a received text. */
void screen_show_messages(void);

/* Wake the black screen and restart its touch-inactivity deadline. */
void screen_wake_from_touch(void);

/* Wake the display and show the alarm scene when the RTC alarm fires. */
void screen_alarm_ring_started(void);

/* Route a CST9217 coordinate event into the window manager. */
void screen_handle_touch(screen_touch_event_t event, uint16_t x, uint16_t y);
