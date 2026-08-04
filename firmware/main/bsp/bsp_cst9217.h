#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Reset + verify the CST9217 touch controller */
esp_err_t bsp_touch_init(void);

/* Poll for a touch point. Returns true when a finger is down and fills x/y. */
bool bsp_touch_get_point(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif
