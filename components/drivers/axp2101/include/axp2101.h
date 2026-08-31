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

/* Charging state (STATUS2 bits 0-2). */
typedef enum {
    AXP2101_CHG_TRI,      /* trickle charge */
    AXP2101_CHG_PRE,      /* pre-charge */
    AXP2101_CHG_CC,       /* constant current */
    AXP2101_CHG_CV,       /* constant voltage */
    AXP2101_CHG_DONE,     /* charge complete */
    AXP2101_CHG_STOP,     /* not charging */
} axp2101_charge_state_t;

esp_err_t axp2101_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val);
esp_err_t axp2101_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);
esp_err_t axp2101_read_chip_id(i2c_master_dev_handle_t dev, uint8_t *id);
esp_err_t axp2101_is_battery_present(i2c_master_dev_handle_t dev, bool *present);
esp_err_t axp2101_is_vbus_present(i2c_master_dev_handle_t dev, bool *present);

esp_err_t axp2101_init(i2c_master_dev_handle_t dev);
/* One-time bring-up: configure a rail's voltage and turn it on. Always
 * rewrites the voltage register, so it's not a cheap no-op if called again -
 * use axp2101_enable_rail() for runtime on/off (e.g. sleep gating). */
esp_err_t axp2101_init_rail(i2c_master_dev_handle_t dev, axp2101_rail_t rail, uint16_t mv);
esp_err_t axp2101_set_default_power(i2c_master_dev_handle_t dev);
esp_err_t axp2101_get_battery_mv(i2c_master_dev_handle_t dev, uint16_t *mv);
esp_err_t axp2101_get_battery_pct(i2c_master_dev_handle_t dev, uint8_t *pct);

/* Runtime on/off at whatever voltage was last configured via
 * axp2101_init_rail() - does not touch the voltage register. Used for
 * sleep/wake rail gating. */
esp_err_t axp2101_enable_rail(i2c_master_dev_handle_t dev, axp2101_rail_t rail, bool enable);

/* Current on/off state of a rail, read live from LDO_ONOFF0 (not cached). */
esp_err_t axp2101_is_rail_enabled(i2c_master_dev_handle_t dev, axp2101_rail_t rail, bool *enabled);

/* Interrupt handling (PEK power key etc.). */
esp_err_t axp2101_enable_pek_irq(i2c_master_dev_handle_t dev);
/* VBUS-insert IRQ (datasheet REG41 bit7, "vinsert_irq") - same enable
 * register as axp2101_enable_pek_irq(), a different bit, safe to call
 * alongside it. Lets USB-plug become a real wake source (e.g. for
 * Ultra-Sparmodus's deep-sleep exit, docs/application.md section 10.4) on
 * the same physical IRQ line PWRKEY already shares - see
 * AXP_IRQ_VBUS_INSERT for reading which one actually fired. */
esp_err_t axp2101_enable_vbus_irq(i2c_master_dev_handle_t dev);
esp_err_t axp2101_clear_irq(i2c_master_dev_handle_t dev);
/* 24-bit IRQ status: bits 0-7 = INTSTS1, 8-15 = INTSTS2, 16-23 = INTSTS3.
 * PEK: INTSTS2 bits 0=press edge, 1=release edge, 2=long, 3=short.
 * VBUS: INTSTS2 bit 7 = insert, bit 6 = remove (datasheet REG49). */
esp_err_t axp2101_get_irq_status(i2c_master_dev_handle_t dev, uint32_t *status);

/* PWRKEY long-press shutdown (datasheet section 6.5.4.3 "Power Off" + REG22H
 * PWROFF_EN + REG27H OFFLEVEL). By factory default the PMIC can cut all
 * rails on its own the instant a press crosses OFFLEVEL (REG22H bit1), with
 * no chance for software to react - that's what this disables. The PEK
 * "long press" IRQ (already unmasked by axp2101_enable_pek_irq(), INTSTS2
 * bit 2) still fires at the same OFFLEVEL threshold either way, so software
 * keeps its notification; it just also keeps the power. Call once at boot,
 * then handle the long-press IRQ by doing cleanup (e.g. sd_log_unmount())
 * and calling axp2101_soft_poweroff() when ready - see power_mgmt.c. */
esp_err_t axp2101_configure_pwrkey_shutdown(i2c_master_dev_handle_t dev);

/* Cleanly power off all rails now via the PMIC's own controlled sequence
 * (REG10H bit0, "Soft PWROFF"). Only call after any needed cleanup is
 * done - the watch loses power essentially immediately after this command
 * is acknowledged over I2C. */
esp_err_t axp2101_soft_poweroff(i2c_master_dev_handle_t dev);

/* Charging control / telemetry. */
esp_err_t axp2101_get_charge_status(i2c_master_dev_handle_t dev, axp2101_charge_state_t *state);
esp_err_t axp2101_set_charge_enabled(i2c_master_dev_handle_t dev, bool enable);
esp_err_t axp2101_is_charge_enabled(i2c_master_dev_handle_t dev, bool *enabled);
/* RTC backup coin cell (Seiko MS621FE-FL11E on VBACKUP) - separate
 * charge-enable bit from the main battery, see axp2101_set_default_power(). */
esp_err_t axp2101_is_button_batt_charge_enabled(i2c_master_dev_handle_t dev, bool *enabled);
esp_err_t axp2101_get_charge_current_ma(i2c_master_dev_handle_t dev, uint16_t *ma);
esp_err_t axp2101_set_charge_current_ma(i2c_master_dev_handle_t dev, uint16_t ma);
/* Battery temperature in 0.1 deg C (TS ADC). */
esp_err_t axp2101_get_battery_temp(i2c_master_dev_handle_t dev, int16_t *tenths_c);

#ifdef __cplusplus
}
#endif
