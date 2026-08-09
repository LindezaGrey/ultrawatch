/*
 * cst9217.h - CST9217 capacitive touch panel (I2C).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CST9217_MAX_POINTS 10

typedef struct {
    uint8_t  num_points;
    uint16_t x[CST9217_MAX_POINTS];
    uint16_t y[CST9217_MAX_POINTS];
    uint8_t  event[CST9217_MAX_POINTS];
} cst9217_data_t;

esp_err_t cst9217_init(i2c_master_dev_handle_t dev);
esp_err_t cst9217_get_touch(i2c_master_dev_handle_t dev, cst9217_data_t *data);

#ifdef __cplusplus
}
#endif
