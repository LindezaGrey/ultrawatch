#include <string.h>
#include "axp2101.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "axp2101";

#define AXP_REG_IC_TYPE         0x03
#define AXP_REG_DC_ONOFF_DVM    0x80
#define AXP_REG_LDO_ONOFF0      0x90
#define AXP_REG_LDO_ONOFF1      0x91
#define AXP_REG_LDO_VOL0        0x92 /* ALDO1 */
#define AXP_REG_ADC_CHANNEL     0x30
#define AXP_REG_ADC_DATA0       0x34 /* batt volt hi */
#define AXP_REG_ADC_DATA1       0x35 /* batt volt lo */
#define AXP_REG_ADC_DATA_TEMP0  0x3C /* die/battery temp hi (H6L8) */
#define AXP_REG_ADC_DATA_TEMP1  0x3D
#define AXP_REG_CHG_GAUGE_WDT   0x18 /* charging + gauge + watchdog ctrl */
#define AXP_REG_ICC_CHG_SET     0x62 /* constant charge current */
#define AXP_REG_BTN_BATT_CHG_V  0x6A /* button battery charge termination voltage */
#define AXP_REG_BAT_PERCENT     0xA4
#define AXP_REG_STATUS1         0x00
#define AXP_REG_STATUS2         0x01
#define AXP_REG_INTEN2     0x41
#define AXP_REG_INTSTS1    0x48
#define AXP_REG_INTSTS2    0x49
#define AXP_REG_INTSTS3    0x4A

#define AXP_INTEN2_PEK    0x0F   /* bits 0-3: press/release edge, long, short */

/* 0x18 bit 1 = main (Cell) battery charging enabled (CHARGE_GAUGE_WDT_CTRL). */
#define AXP_CHG_CTRL_CHARGE  (1u << 1)
/* 0x18 bit 2 = Button Battery (RTC backup) charging enabled. Defaults
 * disabled and resets to disabled on every system reset. */
#define AXP_CHG_CTRL_BTN_BATT (1u << 2)

/* 0x6A bits 2:0 = button battery charge termination voltage, 2.6-3.3V in
 * 100mV steps; 011b = 2.9V is also the chip's own reset default. */
#define AXP_BTN_BATT_CHG_V_MASK  0x07
#define AXP_BTN_BATT_CHG_V_2V9   0x03

/* 0x62 charge current code (XPowers AXP2101 CHG_CUR encoding):
 * 0=0mA 4=100mA 5=125 6=150 7=175 8=200 9=300 10=400 11=500 ... 16=1000. */
#define AXP_ICC_CHG_MASK     0x1F

#define AXP_BATT_VOLT_MSB_BITS  5

#define AXP_LDO_VOLT_MIN_MV     500
#define AXP_LDO_VOLT_STEP_MV    100
#define AXP_LDO_VOLT_MASK       0x1F

/* 0x90 enable bits */
#define AXP_EN_ALDO1   (1u << 0)
#define AXP_EN_ALDO2   (1u << 1)
#define AXP_EN_ALDO3   (1u << 2)
#define AXP_EN_ALDO4   (1u << 3)
#define AXP_EN_BLDO1   (1u << 4)
#define AXP_EN_BLDO2   (1u << 5)
#define AXP_EN_DLDO1   (1u << 6)
#define AXP_EN_DLDO2   (1u << 7)

/* 0x91 enable bits */
#define AXP_EN_CPUSLDO (1u << 0)

/* 0x30 ADC channel enable bits */
#define AXP_ADC_BATT   (1u << 0)
#define AXP_ADC_VBUS   (1u << 2)
#define AXP_ADC_SYS    (1u << 3)
#define AXP_ADC_TEMP   (1u << 4)

typedef struct {
    uint8_t reg;
    uint8_t en_mask;
    uint16_t min_mv;
    uint16_t step_mv;
    uint8_t vol_mask;
} axp_rail_map_t;

static const axp_rail_map_t s_rail_map[AXP2101_RAIL_MAX] = {
    [AXP2101_ALDO1] = { AXP_REG_LDO_VOL0 + 0, AXP_EN_ALDO1, 500, 100, 0x1F },
    [AXP2101_ALDO2] = { AXP_REG_LDO_VOL0 + 1, AXP_EN_ALDO2, 500, 100, 0x1F },
    [AXP2101_ALDO3] = { AXP_REG_LDO_VOL0 + 2, AXP_EN_ALDO3, 500, 100, 0x1F },
    [AXP2101_ALDO4] = { AXP_REG_LDO_VOL0 + 3, AXP_EN_ALDO4, 500, 100, 0x1F },
    [AXP2101_BLDO1] = { AXP_REG_LDO_VOL0 + 4, AXP_EN_BLDO1, 500, 100, 0x1F },
    [AXP2101_BLDO2] = { AXP_REG_LDO_VOL0 + 5, AXP_EN_BLDO2, 500, 100, 0x1F },
    [AXP2101_DLDO1] = { AXP_REG_LDO_VOL0 + 6, AXP_EN_DLDO1, 500, 100, 0x1F },
};

esp_err_t axp2101_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100);
}

esp_err_t axp2101_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t axp2101_set_bit(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t mask, bool set)
{
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, reg, &val), TAG, "read reg 0x%02x", reg);
    if (set) {
        val |= mask;
    } else {
        val &= (uint8_t)~mask;
    }
    return axp2101_write_reg(dev, reg, val);
}

esp_err_t axp2101_read_chip_id(i2c_master_dev_handle_t dev, uint8_t *id)
{
    return axp2101_read_reg(dev, AXP_REG_IC_TYPE, id);
}

esp_err_t axp2101_is_battery_present(i2c_master_dev_handle_t dev, bool *present)
{
    uint8_t st = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_STATUS1, &st), TAG, "read status1");
    *present = (st & (1u << 3)) != 0;
    return ESP_OK;
}

esp_err_t axp2101_is_vbus_present(i2c_master_dev_handle_t dev, bool *present)
{
    uint8_t st1 = 0, st2 = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_STATUS1, &st1), TAG, "read status1");
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_STATUS2, &st2), TAG, "read status2");
    *present = ((st1 & (1u << 5)) != 0) && ((st2 & (1u << 3)) == 0);
    return ESP_OK;
}

esp_err_t axp2101_init(i2c_master_dev_handle_t dev)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_chip_id(dev, &id), TAG, "axp2101 read chip id failed");
    if (id != AXP2101_CHIP_ID_EXPECTED) {
        ESP_LOGW(TAG, "unexpected chip id 0x%02x (expected 0x%02x), continuing", id, AXP2101_CHIP_ID_EXPECTED);
    } else {
        ESP_LOGI(TAG, "AXP2101 found, chip id 0x%02x", id);
    }

    /* Clear PMU interrupt status by reading the status registers. */
    uint8_t tmp[3] = { 0 };
    for (int i = 0; i < 3; i++) {
        axp2101_read_reg(dev, AXP_REG_INTSTS1 + i, &tmp[i]);
    }

    /* Enable ADC channels: battery voltage, vbus, system, temperature. */
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_ADC_CHANNEL, AXP_ADC_BATT, true), TAG, "enable batt adc");
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_ADC_CHANNEL, AXP_ADC_VBUS, true), TAG, "enable vbus adc");
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_ADC_CHANNEL, AXP_ADC_SYS, true), TAG, "enable sys adc");
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_ADC_CHANNEL, AXP_ADC_TEMP, true), TAG, "enable temp adc");

    return ESP_OK;
}

esp_err_t axp2101_init_rail(i2c_master_dev_handle_t dev, axp2101_rail_t rail, uint16_t mv)
{
    if (rail >= AXP2101_RAIL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    const axp_rail_map_t *m = &s_rail_map[rail];
    if (mv < m->min_mv || (mv - m->min_mv) % m->step_mv != 0) {
        ESP_LOGE(TAG, "invalid rail voltage %umV (min %umV, step %umV)", mv, m->min_mv, m->step_mv);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vol = (mv - m->min_mv) / m->step_mv;
    uint8_t cur = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, m->reg, &cur), TAG, "read vol reg 0x%02x", m->reg);
    cur = (cur & ~m->vol_mask) | (vol & m->vol_mask);
    ESP_RETURN_ON_ERROR(axp2101_write_reg(dev, m->reg, cur), TAG, "write vol reg 0x%02x", m->reg);

    return axp2101_set_bit(dev, AXP_REG_LDO_ONOFF0, m->en_mask, true);
}

esp_err_t axp2101_set_default_power(i2c_master_dev_handle_t dev)
{
    /* Power tree per LilyGO T-Watch Ultra (LilyGoWatchUltra::initPMU). */
    esp_err_t ret = axp2101_init_rail(dev, AXP2101_ALDO1, 3300); /* SD card */
    ESP_RETURN_ON_ERROR(ret, TAG, "aldo1");
    ret = axp2101_init_rail(dev, AXP2101_ALDO2, 3300);           /* display */
    ESP_RETURN_ON_ERROR(ret, TAG, "aldo2");
    ret = axp2101_init_rail(dev, AXP2101_ALDO3, 3300);           /* LoRa */
    ESP_RETURN_ON_ERROR(ret, TAG, "aldo3");
    ret = axp2101_init_rail(dev, AXP2101_ALDO4, 1800);           /* sensor */
    ESP_RETURN_ON_ERROR(ret, TAG, "aldo4");
    ret = axp2101_init_rail(dev, AXP2101_BLDO1, 3300);           /* GNSS */
    ESP_RETURN_ON_ERROR(ret, TAG, "bldo1");
    ret = axp2101_init_rail(dev, AXP2101_BLDO2, 3300);           /* speaker */
    ESP_RETURN_ON_ERROR(ret, TAG, "bldo2");
    ret = axp2101_init_rail(dev, AXP2101_DLDO1, 3300);           /* NFC */
    ESP_RETURN_ON_ERROR(ret, TAG, "dldo1");

    /* Unused channels: DC2-DC5, CPUSLDO. */
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_DC_ONOFF_DVM, 0x1E, false), TAG, "disable dc2-5");
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_LDO_ONOFF1, AXP_EN_CPUSLDO, false), TAG, "disable cpusldo");

    /* RTC backup battery (Seiko MS621FE-FL11E, rechargeable, on VBACKUP per
     * schematic): its charge-enable bit defaults disabled and resets to
     * disabled on every system reset, so without this the coin cell that
     * keeps the RTC/BHI260AP VBACKUP time alive across power loss is never
     * charged. */
    ESP_RETURN_ON_ERROR(axp2101_set_bit(dev, AXP_REG_CHG_GAUGE_WDT, AXP_CHG_CTRL_BTN_BATT, true),
                         TAG, "enable button battery charge");
    uint8_t btn_v = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_BTN_BATT_CHG_V, &btn_v), TAG, "read button batt chg v");
    btn_v = (btn_v & ~AXP_BTN_BATT_CHG_V_MASK) | AXP_BTN_BATT_CHG_V_2V9;
    ESP_RETURN_ON_ERROR(axp2101_write_reg(dev, AXP_REG_BTN_BATT_CHG_V, btn_v), TAG, "set button batt chg v");

    ESP_LOGI(TAG, "AXP2101 power tree configured");
    return ESP_OK;
}

esp_err_t axp2101_get_battery_mv(i2c_master_dev_handle_t dev, uint16_t *mv)
{
    bool present = false;
    ESP_RETURN_ON_ERROR(axp2101_is_battery_present(dev, &present), TAG, "battery present check");
    if (!present) {
        *mv = 0;
        return ESP_OK;
    }
    uint8_t hi = 0, lo = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_ADC_DATA0, &hi), TAG, "read batt hi");
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_ADC_DATA1, &lo), TAG, "read batt lo");
    /* 13-bit ADC, value in millivolts. */
    *mv = (uint16_t)((hi & ((1u << AXP_BATT_VOLT_MSB_BITS) - 1)) << 8) | lo;
    return ESP_OK;
}

esp_err_t axp2101_get_battery_pct(i2c_master_dev_handle_t dev, uint8_t *pct)
{
    bool present = false;
    ESP_RETURN_ON_ERROR(axp2101_is_battery_present(dev, &present), TAG, "battery present check");
    if (!present) {
        *pct = 0;
        return ESP_OK;
    }
    return axp2101_read_reg(dev, AXP_REG_BAT_PERCENT, pct);
}

esp_err_t axp2101_enable_rail(i2c_master_dev_handle_t dev, axp2101_rail_t rail, bool enable)
{
    if (rail >= AXP2101_RAIL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    return axp2101_set_bit(dev, AXP_REG_LDO_ONOFF0, s_rail_map[rail].en_mask, enable);
}

esp_err_t axp2101_enable_pek_irq(i2c_master_dev_handle_t dev)
{
    /* Enable PEK interrupts (press/release edge, long/short press) in INTEN2. */
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_INTEN2, &val), TAG, "read inten2");
    return axp2101_write_reg(dev, AXP_REG_INTEN2, (uint8_t)(val | AXP_INTEN2_PEK));
}

esp_err_t axp2101_clear_irq(i2c_master_dev_handle_t dev)
{
    /* Write 0xFF to the status registers to clear all pending interrupts. */
    for (int i = 0; i < 3; i++) {
        esp_err_t ret = axp2101_write_reg(dev, AXP_REG_INTSTS1 + i, 0xFF);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t axp2101_get_irq_status(i2c_master_dev_handle_t dev, uint32_t *status)
{
    uint8_t s[3] = { 0, 0, 0 };
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_INTSTS1, &s[0]), TAG, "intsts1");
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_INTSTS2, &s[1]), TAG, "intsts2");
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_INTSTS3, &s[2]), TAG, "intsts3");
    *status = ((uint32_t)s[2] << 16) | ((uint32_t)s[1] << 8) | s[0];
    return ESP_OK;
}

/* Charging state: STATUS2 bits 0-2. Values per XPowers AXP2101:
 * 0=tri 1=pre 2=CC 3=CV 4=done 5=stop. */
esp_err_t axp2101_get_charge_status(i2c_master_dev_handle_t dev, axp2101_charge_state_t *state)
{
    uint8_t st = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_STATUS2, &st), TAG, "read status2");
    *state = (axp2101_charge_state_t)(st & 0x07);
    return ESP_OK;
}

esp_err_t axp2101_set_charge_enabled(i2c_master_dev_handle_t dev, bool enable)
{
    return axp2101_set_bit(dev, AXP_REG_CHG_GAUGE_WDT, AXP_CHG_CTRL_CHARGE, enable);
}

esp_err_t axp2101_is_charge_enabled(i2c_master_dev_handle_t dev, bool *enabled)
{
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_CHG_GAUGE_WDT, &val), TAG, "read chg ctrl");
    *enabled = (val & AXP_CHG_CTRL_CHARGE) != 0;
    return ESP_OK;
}

/* Map a charge-current code (0x62 low 5 bits) to mA. */
static uint16_t charge_code_to_ma(uint8_t code)
{
    switch (code) {
    case 0:  return 0;
    case 4:  return 100;
    case 5:  return 125;
    case 6:  return 150;
    case 7:  return 175;
    case 8:  return 200;
    case 9:  return 300;
    case 10: return 400;
    case 11: return 500;
    case 12: return 600;
    case 13: return 700;
    case 14: return 800;
    case 15: return 900;
    case 16: return 1000;
    default: return 0;
    }
}

/* Map mA to the nearest charge-current code. */
static uint8_t charge_ma_to_code(uint16_t ma)
{
    if (ma < 100) return 0;
    if (ma < 125) return 4;
    if (ma < 150) return 5;
    if (ma < 175) return 6;
    if (ma < 200) return 7;
    if (ma < 300) return 8;
    if (ma < 400) return 9;
    if (ma < 500) return 10;
    if (ma < 600) return 11;
    if (ma < 700) return 12;
    if (ma < 800) return 13;
    if (ma < 900) return 14;
    if (ma < 1000) return 15;
    return 16;
}

esp_err_t axp2101_get_charge_current_ma(i2c_master_dev_handle_t dev, uint16_t *ma)
{
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_ICC_CHG_SET, &val), TAG, "read chg current");
    *ma = charge_code_to_ma(val & AXP_ICC_CHG_MASK);
    return ESP_OK;
}

esp_err_t axp2101_set_charge_current_ma(i2c_master_dev_handle_t dev, uint16_t ma)
{
    uint8_t code = charge_ma_to_code(ma);
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_ICC_CHG_SET, &val), TAG, "read chg current");
    val = (uint8_t)((val & ~AXP_ICC_CHG_MASK) | (code & AXP_ICC_CHG_MASK));
    return axp2101_write_reg(dev, AXP_REG_ICC_CHG_SET, val);
}

/* Battery/die temperature in 0.1 deg C from the TS ADC (H6L8 @ 0x3C/0x3D).
 * XPowers conversion: T(C) = 22.0 + (7274 - raw) / 20.0. */
esp_err_t axp2101_get_battery_temp(i2c_master_dev_handle_t dev, int16_t *tenths_c)
{
    uint8_t hi = 0, lo = 0;
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_ADC_DATA_TEMP0, &hi), TAG, "read temp hi");
    ESP_RETURN_ON_ERROR(axp2101_read_reg(dev, AXP_REG_ADC_DATA_TEMP1, &lo), TAG, "read temp lo");
    uint16_t raw = (uint16_t)(((hi & 0x3F) << 8) | lo);
    if (raw > 16383) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* 22.0 + (7274 - raw)/20.0  ->  fixed point 0.1 deg C */
    int32_t v = 220 + ((int32_t)7274 - (int32_t)raw) / 2;
    *tenths_c = (int16_t)v;
    return ESP_OK;
}
