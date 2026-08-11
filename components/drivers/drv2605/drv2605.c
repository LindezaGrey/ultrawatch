/*
 * drv2605.c - TI DRV2605 haptic driver (I2C).
 *
 * DRV2605 register map (0x5A):
 *   0x01 Mode (MOD 2:0, RTPIN 6, ERM/LRA 5, STANDBY 7)
 *   0x02 Real-time playback control (input analog/PWM)
 *   0x03 Library selection
 *   0x04-0x0B Waveform sequencer (1..8)
 *   0x0C Go bit
 *   0x1D Data (real-time amplitude)
 *
 * The board drives M_EN on the XL9555 (haptic enable); set it high before
 * use. Default actuator type is ERM with library 1 (the T-Watch uses an ERM
 * motor).
 */
#include "drv2605.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "drv2605";

#define DRV2605_MODE_REG      0x01
#define DRV2605_LIBRARY_REG   0x03
#define DRV2605_WAVEFORM0     0x04
#define DRV2605_GO_REG        0x0C
#define DRV2605_DATA_REG      0x1D

#define DRV2605_MODE_STANDBY  (1u << 7)
#define DRV2605_MODE_RTPIN    (1u << 6)
#define DRV2605_MODE_ERM_LRA  (1u << 5)   /* 1=LRA, 0=ERM */

static esp_err_t drv2605_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t drv2605_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100);
}

esp_err_t drv2605_init(i2c_master_dev_handle_t dev)
{
    /* Take out of standby, internal trigger, ERM (default library 1). */
    ESP_RETURN_ON_ERROR(drv2605_write_reg(dev, DRV2605_MODE_REG,
                                          DRV2605_MODE_INT_TRIGGER),
                        TAG, "mode");
    ESP_RETURN_ON_ERROR(drv2605_write_reg(dev, DRV2605_LIBRARY_REG, 1), TAG, "library");
    /* Read back the mode register to confirm the chip is present. */
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(drv2605_read_reg(dev, DRV2605_MODE_REG, &val), TAG, "readback");
    ESP_LOGI(TAG, "DRV2605 initialized (mode=0x%02X, ERM, library 1)", val);
    return ESP_OK;
}

esp_err_t drv2605_set_waveform(i2c_master_dev_handle_t dev, uint8_t slot, uint8_t wave)
{
    if (slot > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    return drv2605_write_reg(dev, DRV2605_WAVEFORM0 + slot, wave);
}

esp_err_t drv2605_go(i2c_master_dev_handle_t dev)
{
    return drv2605_write_reg(dev, DRV2605_GO_REG, 1);
}

esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave)
{
    ESP_RETURN_ON_ERROR(drv2605_set_waveform(dev, 0, wave), TAG, "set_waveform failed");
    ESP_RETURN_ON_ERROR(drv2605_set_waveform(dev, 1, 0), TAG, "stop slot");
    return drv2605_go(dev);
}
