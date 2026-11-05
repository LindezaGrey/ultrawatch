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

esp_err_t drv2605_init(i2c_master_dev_handle_t dev);
/* Store a library effect ID (0 terminates) in one sequencer slot 0..7. */
esp_err_t drv2605_set_waveform(i2c_master_dev_handle_t dev, uint8_t slot, uint8_t wave);
esp_err_t drv2605_go(i2c_master_dev_handle_t dev);
esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave);

/* Run on-chip auto-calibration (motor rail must be active; the motor runs
 * for about 1 s). Returns timeout, I2C, mode-restore, or NVS persistence
 * errors rather than reporting incomplete/unsaved calibration as success. */
esp_err_t drv2605_auto_calibrate(i2c_master_dev_handle_t dev);

/* Restore previously saved volatile calibration registers from NVS. Returns
 * ESP_ERR_NOT_FOUND if no saved calibration exists. */
esp_err_t drv2605_calibrate_restore(i2c_master_dev_handle_t dev);

#ifdef __cplusplus
}
#endif
