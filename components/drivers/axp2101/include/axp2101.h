/*
 * axp2101.h - X-Powers AXP2101 PMU (I2C 0x34).
 *
 * Register map verified against the XPowersLib AXP2101 reference:
 *   - chip ID:   0x03 = 0x4A
 *   - enables:   0x90 (ALDO1..DLDO2), 0x91 bit0 (CPUSLDO), 0x80 bit0 (DC1)
 *   - voltages:  0x92..0x99 = ALDO1..DLDO2, value (mV-500)/100
 *   - ADC enable:0x30 bit0 batt-V, bit2 VBUS, bit3 sys-V, bit4 temp
 *   - battery:   0x34 (hi5) + 0x35 (lo8) -> 13-bit mV; percent 0xA4
 *   - status:    0x00 bit3 battery present
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AXP2101_I2C_ADDR         0x34
#define AXP2101_CHIP_ID_EXPECTED 0x4A

typedef enum {
    AXP2101_ALDO1 = 0,
    AXP2101_ALDO2,
    AXP2101_ALDO3,
    AXP2101_ALDO4,
    AXP2101_BLDO1,
    AXP2101_BLDO2,
    AXP2101_DLDO1,
    AXP2101_RAIL_MAX,
} axp2101_rail_t;

esp_err_t axp2101_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val);
esp_err_t axp2101_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);
esp_err_t axp2101_read_chip_id(i2c_master_dev_handle_t dev, uint8_t *id);
esp_err_t axp2101_is_battery_present(i2c_master_dev_handle_t dev, bool *present);
esp_err_t axp2101_is_vbus_present(i2c_master_dev_handle_t dev, bool *present);

esp_err_t axp2101_init(i2c_master_dev_handle_t dev);
esp_err_t axp2101_set_rail(i2c_master_dev_handle_t dev, axp2101_rail_t rail, uint16_t mv);
esp_err_t axp2101_set_default_power(i2c_master_dev_handle_t dev);
esp_err_t axp2101_get_battery_mv(i2c_master_dev_handle_t dev, uint16_t *mv);
esp_err_t axp2101_get_battery_pct(i2c_master_dev_handle_t dev, uint8_t *pct);

/* Enable/disable a power rail (used for sleep). */
esp_err_t axp2101_enable_rail(i2c_master_dev_handle_t dev, axp2101_rail_t rail, bool enable);

/* Interrupt handling (PEK power key etc.). */
esp_err_t axp2101_enable_pek_irq(i2c_master_dev_handle_t dev);
esp_err_t axp2101_clear_irq(i2c_master_dev_handle_t dev);
/* 24-bit IRQ status: bits 0-7 = INTSTS1, 8-15 = INTSTS2, 16-23 = INTSTS3.
 * PEK: INTSTS2 bits 0=press edge, 1=release edge, 2=long, 3=short. */
esp_err_t axp2101_get_irq_status(i2c_master_dev_handle_t dev, uint32_t *status);

#ifdef __cplusplus
}
#endif
