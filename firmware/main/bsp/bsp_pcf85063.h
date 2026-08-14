#pragma once

#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the RTC (24h mode). Does not overwrite time. */
esp_err_t bsp_rtc_init(void);

/* Read time. Returns ESP_ERR_INVALID_STATE if the RTC lost time (OS flag set). */
esp_err_t bsp_rtc_get_time(struct tm *tm);

esp_err_t bsp_rtc_set_time(const struct tm *tm);

#ifdef __cplusplus
}
#endif
