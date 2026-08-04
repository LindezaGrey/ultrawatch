#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_i2c_init(void);

/* Read/write a register of a 7-bit addressed I2C device */
esp_err_t bsp_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
esp_err_t bsp_i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);

/* Raw transactions (no register byte), used by the CST9217 touch protocol */
esp_err_t bsp_i2c_read_raw(uint8_t addr, uint8_t *data, size_t len);
esp_err_t bsp_i2c_write_raw(uint8_t addr, const uint8_t *data, size_t len);

/* Write a command, then read the response (used by CST9217) */
esp_err_t bsp_i2c_transmit_receive(uint8_t addr, const uint8_t *cmd, size_t cmd_len,
                                   uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
