#include "bsp_xl9555.h"
#include "bsp_i2c.h"
#include "bsp_twatch_ultra.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_xl9555";

/* XL9555 register map (PCA9555 compatible) */
#define XL_REG_INPUT0    0x00
#define XL_REG_INPUT1    0x01
#define XL_REG_OUTPUT0   0x02
#define XL_REG_OUTPUT1   0x03
#define XL_REG_CONFIG0   0x06
#define XL_REG_CONFIG1   0x07

static uint8_t xl9555_reg(uint8_t base, uint8_t pin)
{
    return (pin < 8) ? base : base + 1;
}

static uint8_t xl9555_bit(uint8_t pin)
{
    return pin % 8;
}

static esp_err_t xl9555_read(uint8_t reg, uint8_t *val)
{
    return bsp_i2c_read_reg(XL9555_I2C_ADDR, reg, val, 1);
}

static esp_err_t xl9555_write(uint8_t reg, uint8_t val)
{
    return bsp_i2c_write_reg(XL9555_I2C_ADDR, reg, &val, 1);
}

esp_err_t bsp_xl9555_pin_mode(uint8_t pin, bool output)
{
    uint8_t reg = xl9555_reg(XL_REG_CONFIG0, pin);
    uint8_t bit = xl9555_bit(pin);
    uint8_t val = 0;

    ESP_RETURN_ON_ERROR(xl9555_read(reg, &val), TAG, "read config failed");
    if (output) {
        val &= ~(1 << bit);  /* 0 = output */
    } else {
        val |= (1 << bit);   /* 1 = input */
    }
    return xl9555_write(reg, val);
}

esp_err_t bsp_xl9555_write_pin(uint8_t pin, bool level)
{
    uint8_t reg = xl9555_reg(XL_REG_OUTPUT0, pin);
    uint8_t bit = xl9555_bit(pin);
    uint8_t val = 0;

    ESP_RETURN_ON_ERROR(xl9555_read(reg, &val), TAG, "read output failed");
    if (level) {
        val |= (1 << bit);
    } else {
        val &= ~(1 << bit);
    }
    return xl9555_write(reg, val);
}

esp_err_t bsp_xl9555_read_pin(uint8_t pin, bool *level)
{
    uint8_t reg = xl9555_reg(XL_REG_INPUT0, pin);
    uint8_t bit = xl9555_bit(pin);
    uint8_t val = 0;

    ESP_RETURN_ON_ERROR(xl9555_read(reg, &val), TAG, "read input failed");
    *level = (val >> bit) & 0x01;
    return ESP_OK;
}

esp_err_t bsp_xl9555_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_xl9555_pin_mode(XL9555_PIN_DRV_EN, true), TAG, "drv_en mode failed");
    ESP_RETURN_ON_ERROR(bsp_xl9555_pin_mode(XL9555_PIN_DISP_EN, true), TAG, "disp_en mode failed");
    ESP_RETURN_ON_ERROR(bsp_xl9555_pin_mode(XL9555_PIN_TOUCH_RST, true), TAG, "touch_rst mode failed");
    ESP_RETURN_ON_ERROR(bsp_xl9555_pin_mode(BSP_SD_DET_PIN, false), TAG, "sd_det mode failed");

    ESP_RETURN_ON_ERROR(bsp_xl9555_write_pin(XL9555_PIN_DRV_EN, true), TAG, "drv_en write failed");
    ESP_RETURN_ON_ERROR(bsp_xl9555_write_pin(XL9555_PIN_DISP_EN, true), TAG, "disp_en write failed");
    ESP_RETURN_ON_ERROR(bsp_xl9555_write_pin(XL9555_PIN_TOUCH_RST, true), TAG, "touch_rst write failed");

    ESP_LOGI(TAG, "XL9555 ready");
    return ESP_OK;
}

bool bsp_xl9555_sd_detect(void)
{
    bool lvl = false;
    if (bsp_xl9555_read_pin(BSP_SD_DET_PIN, &lvl) != ESP_OK) {
        return false;
    }
    return !lvl; /* active low */
}
