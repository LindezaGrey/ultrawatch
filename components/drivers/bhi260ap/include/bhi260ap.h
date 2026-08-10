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

/* Bring up the BHI260AP: upload the RAM firmware from the SPIFFS assets
 * partition, boot it, and enable the step counter. The assets partition must
 * be mounted first. On failure logs and returns an error (never aborts). */
esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev);

/* Poll the FIFO once (delivers queued sensor events to registered callbacks). */
esp_err_t bhi260ap_process_fifo(void);

/* Latest step count reported by the on-chip step counter. */
esp_err_t bhi260ap_get_step_count(uint32_t *steps);

esp_err_t bhi260ap_read_accel(i2c_master_dev_handle_t dev, bhi260ap_accel_t *accel);
esp_err_t bhi260ap_read_gyro(i2c_master_dev_handle_t dev, int16_t out[3]);

#ifdef __cplusplus
}
#endif
