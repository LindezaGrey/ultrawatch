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
/* Store a library-1 effect ID in one sequencer slot 0..7 (0 terminates the
 * sequence; nonzero effect IDs must be in the library-1 range 1..123). */
esp_err_t drv2605_set_waveform(i2c_master_dev_handle_t dev, uint8_t slot, uint8_t wave);
/* Trigger the already-programmed waveform sequence; caller must ensure the
 * actuator rail and enable line are active. */
esp_err_t drv2605_go(i2c_master_dev_handle_t dev);
/* Configure and trigger one library-1 effect (1..123). */
esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave);

/* Run on-chip auto-calibration (motor rail must be active; motor runs for
 * about 1 s, with a 1.5 s maximum wait). Serializes the whole operation and
 * returns timeout, I2C, mode-restore, or NVS persistence errors. */
esp_err_t drv2605_auto_calibrate(i2c_master_dev_handle_t dev);

/* Restore previously saved volatile calibration registers from NVS. Returns
 * ESP_ERR_NOT_FOUND if no complete saved calibration pair exists. */
esp_err_t drv2605_calibrate_restore(i2c_master_dev_handle_t dev);

#ifdef __cplusplus
}
#endif
