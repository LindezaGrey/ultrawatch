/*
 * pcf85063a.h - NXP PCF85063A RTC (I2C).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t weekday;
    uint8_t month;
    uint16_t year;
} pcf85063a_time_t;

esp_err_t pcf85063a_init(i2c_master_dev_handle_t dev);
esp_err_t pcf85063a_get_time(i2c_master_dev_handle_t dev, pcf85063a_time_t *t);
esp_err_t pcf85063a_set_time(i2c_master_dev_handle_t dev, const pcf85063a_time_t *t);

#ifdef __cplusplus
}
#endif
