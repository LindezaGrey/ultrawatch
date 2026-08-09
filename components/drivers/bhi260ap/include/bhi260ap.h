/*
 * bhi260ap.h - Bosch BHI260AP smart sensor / IMU (I2C, Bosch FSC protocol).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t acc_x_mg;
    int16_t acc_y_mg;
    int16_t acc_z_mg;
} bhi260ap_accel_t;

esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev);
esp_err_t bhi260ap_read_accel(i2c_master_dev_handle_t dev, bhi260ap_accel_t *accel);
esp_err_t bhi260ap_read_gyro(i2c_master_dev_handle_t dev, int16_t out[3]);

#ifdef __cplusplus
}
#endif
