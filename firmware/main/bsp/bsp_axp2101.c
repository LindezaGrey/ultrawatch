#include "bsp_axp2101.h"
#include "bsp_i2c.h"
#include "bsp_twatch_ultra.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_axp2101";

/* AXP2101 registers (see XPowersLib AXP2101Constants.h / datasheet) */
#define AXP_REG_STATUS1             0x00
#define AXP_REG_IC_TYPE             0x03
#define AXP_REG_ADC_CHANNEL_CTRL    0x30
#define AXP_REG_ADC_DATA_RELUST0    0x34
#define AXP_REG_ADC_DATA_RELUST1    0x35
#define AXP_REG_BAT_DET_CTRL        0x68
#define AXP_REG_LDO_ONOFF_CTRL0     0x90
#define AXP_REG_ALDO1_VOL_CTRL      0x92
#define AXP_REG_LDO_VOL1_CTRL       0x93

#define AXP_CHIP_ID                 0x4A

#define AXP_STATUS1_BAT_CONNECT_BIT (3)
#define AXP_ADC_CTRL_BATT_VOL_BIT   (0)
#define AXP_BAT_DET_EN_BIT          (0)
#define AXP_LDO_CTRL_ALDO1_EN_BIT   (0)
#define AXP_LDO_CTRL_ALDO2_EN_BIT   (1)

#define AXP_ALDO1_MV                3300
#define AXP_ALDO2_MV                3300
#define AXP_ALDO_VOL_STEP_MV        100
#define AXP_ALDO_VOL_MIN_MV         500

#define AXP_BATT_ADC_LSB_MV         10  /* 1 mV per LSB (datasheet 6.10, Table 6-5), scaled by 10 */

esp_err_t bsp_axp2101_init(void)
{
    uint8_t val = 0;

    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_IC_TYPE, &val, 1),
                        TAG, "read chip id failed");
    if (val != AXP_CHIP_ID) {
        ESP_LOGE(TAG, "unexpected chip id 0x%02X (expected 0x%02X)", val, AXP_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }

    /* ALDO1 = 3.3V, SD card power rail (also set before enabling) */
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_ALDO1_VOL_CTRL, &val, 1),
                        TAG, "read aldo1 vol failed");
    val &= 0xE0;
    val |= (AXP_ALDO1_MV - AXP_ALDO_VOL_MIN_MV) / AXP_ALDO_VOL_STEP_MV;
    ESP_RETURN_ON_ERROR(bsp_i2c_write_reg(AXP2101_I2C_ADDR, AXP_REG_ALDO1_VOL_CTRL, &val, 1),
                        TAG, "write aldo1 vol failed");

    /* ALDO2 = 3.3V, display power rail */
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_LDO_VOL1_CTRL, &val, 1),
                        TAG, "read ldo vol failed");
    val &= 0xE0;
    val |= (AXP_ALDO2_MV - AXP_ALDO_VOL_MIN_MV) / AXP_ALDO_VOL_STEP_MV;
    ESP_RETURN_ON_ERROR(bsp_i2c_write_reg(AXP2101_I2C_ADDR, AXP_REG_LDO_VOL1_CTRL, &val, 1),
                        TAG, "write ldo vol failed");

    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_LDO_ONOFF_CTRL0, &val, 1),
                        TAG, "read ldo onoff failed");
    val |= (1 << AXP_LDO_CTRL_ALDO2_EN_BIT);
    ESP_RETURN_ON_ERROR(bsp_i2c_write_reg(AXP2101_I2C_ADDR, AXP_REG_LDO_ONOFF_CTRL0, &val, 1),
                        TAG, "write ldo onoff failed");

    /* Enable battery detection and battery voltage ADC */
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_ADC_CHANNEL_CTRL, &val, 1),
                        TAG, "read adc ctrl failed");
    val |= (1 << AXP_ADC_CTRL_BATT_VOL_BIT);
    ESP_RETURN_ON_ERROR(bsp_i2c_write_reg(AXP2101_I2C_ADDR, AXP_REG_ADC_CHANNEL_CTRL, &val, 1),
                        TAG, "write adc ctrl failed");

    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_BAT_DET_CTRL, &val, 1),
                        TAG, "read bat det failed");
    val |= (1 << AXP_BAT_DET_EN_BIT);
    ESP_RETURN_ON_ERROR(bsp_i2c_write_reg(AXP2101_I2C_ADDR, AXP_REG_BAT_DET_CTRL, &val, 1),
                        TAG, "write bat det failed");

    ESP_LOGI(TAG, "AXP2101 ready");
    return ESP_OK;
}

bool bsp_axp2101_battery_connected(void)
{
    uint8_t val = 0;
    if (bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_STATUS1, &val, 1) != ESP_OK) {
        return false;
    }
    return (val >> AXP_STATUS1_BAT_CONNECT_BIT) & 0x01;
}

uint16_t bsp_axp2101_battery_voltage_mv(void)
{
    uint8_t data[2] = { 0 };
    if (!bsp_axp2101_battery_connected()) {
        return 0;
    }
    if (bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_ADC_DATA_RELUST0, data, 2) != ESP_OK) {
        return 0;
    }
    /* 14-bit ADC, high 6 bits in reg 0x34 then low 8 bits in reg 0x35 */
    uint16_t adc = ((data[0] & 0x3F) << 8) | data[1];
    return (uint16_t)((adc * AXP_BATT_ADC_LSB_MV) / 10u);
}

uint8_t bsp_axp2101_battery_percent(void)
{
    if (!bsp_axp2101_battery_connected()) {
        return 0xFF;
    }
    uint16_t mv = bsp_axp2101_battery_voltage_mv();
    const uint16_t v_empty = 3200;
    const uint16_t v_full = 4200;

    if (mv >= v_full) {
        return 100;
    }
    if (mv <= v_empty) {
        return 0;
    }
    return (uint8_t)(((uint32_t)(mv - v_empty) * 100) / (v_full - v_empty));
}

esp_err_t bsp_axp2101_set_aldo1(bool enable)
{
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(AXP2101_I2C_ADDR, AXP_REG_LDO_ONOFF_CTRL0, &val, 1),
                        TAG, "read ldo onoff failed");
    if (enable) {
        val |= (1 << AXP_LDO_CTRL_ALDO1_EN_BIT);
    } else {
        val &= ~(1 << AXP_LDO_CTRL_ALDO1_EN_BIT);
    }
    return bsp_i2c_write_reg(AXP2101_I2C_ADDR, AXP_REG_LDO_ONOFF_CTRL0, &val, 1);
}
