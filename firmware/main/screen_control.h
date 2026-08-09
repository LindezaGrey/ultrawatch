#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Set the AMOLED DCS brightness level using a percentage from 0 through 100. */
esp_err_t screen_set_brightness(uint8_t percentage);

/* Return the last brightness level successfully sent to the AMOLED. */
uint8_t screen_get_brightness(void);

/* Ask the display task to redraw time, battery, and BLE state immediately. */
void screen_request_refresh(void);

/* Wake the black screen and restart its touch-inactivity deadline. */
void screen_wake_from_touch(void);
