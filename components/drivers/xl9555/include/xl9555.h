/*
 * xl9555.h - Xinluda XL9555 I2C GPIO expander (16-bit, 0x20).
 *
 * Register map (PCA9555-compatible, verified against LilyGO SensorLib):
 *   0x00/0x01 Input Port 0/1, 0x02/0x03 Output Port 0/1,
 *   0x06/0x07 Configuration 0/1 (1=input, 0=output).
 *   Port 0 = pins 0..7, Port 1 = pins 8..15.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XL9555_I2C_ADDR 0x20

esp_err_t xl9555_init(i2c_master_dev_handle_t dev);
esp_err_t xl9555_pin_mode(i2c_master_dev_handle_t dev, uint8_t pin, bool output);
esp_err_t xl9555_set_output(i2c_master_dev_handle_t dev, uint8_t pin, bool level);
esp_err_t xl9555_read_input(i2c_master_dev_handle_t dev, uint8_t pin, bool *level);
esp_err_t xl9555_read_port(i2c_master_dev_handle_t dev, uint16_t *state);

#ifdef __cplusplus
}
#endif
