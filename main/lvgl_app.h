/*
 * lvgl_app.h - LVGL integration via the Espressif esp_lvgl_adapter.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lvgl_app_start(void);

#ifdef __cplusplus
}
#endif
