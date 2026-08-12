/*
 * drv2605.h - TI DRV2605 haptic driver (I2C).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV2605_MODE_INT_TRIGGER = 0,
    DRV2605_MODE_EXT_TRIGGER = 1,
    DRV2605_MODE_REAL_TIME   = 5,
} drv2605_mode_t;

esp_err_t drv2605_init(i2c_master_dev_handle_t dev);
esp_err_t drv2605_set_waveform(i2c_master_dev_handle_t dev, uint8_t slot, uint8_t wave);
esp_err_t drv2605_go(i2c_master_dev_handle_t dev);
esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave);

/* Run the on-chip auto-calibration (drives the motor ~1 s) and save the
 * results to NVS. The DRV2605 has no non-volatile memory, so this is needed
 * once after the first power-up. */
esp_err_t drv2605_auto_calibrate(i2c_master_dev_handle_t dev);

/* Restore a previously saved calibration from NVS. Returns ESP_ERR_NOT_FOUND
 * if none is stored. */
esp_err_t drv2605_calibrate_restore(i2c_master_dev_handle_t dev);

#ifdef __cplusplus
}
#endif
