#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Set the AMOLED DCS brightness level using a percentage from 0 through 100. */
esp_err_t screen_set_brightness(uint8_t percentage);

/* Return the last brightness level successfully sent to the AMOLED. */
uint8_t screen_get_brightness(void);
