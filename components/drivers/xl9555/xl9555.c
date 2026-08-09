#include "xl9555.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "xl9555";

#define XL_REG_INP0   0x00
#define XL_REG_INP1   0x01
#define XL_REG_OUTP0  0x02
#define XL_REG_OUTP1  0x03
#define XL_REG_CFG0   0x06
#define XL_REG_CFG1   0x07

static esp_err_t xl9555_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100);
}

static esp_err_t xl9555_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

/* 1 = input, 0 = output (config register semantics). */
static uint8_t xl9555_port_and_bit(uint8_t pin, uint8_t *reg)
{
    *reg = (pin < 8) ? XL_REG_CFG0 : XL_REG_CFG1;
    return (uint8_t)(1u << (pin % 8));
}

esp_err_t xl9555_init(i2c_master_dev_handle_t dev)
{
    uint8_t cfg = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, XL_REG_CFG0, &cfg), TAG, "probe failed (reg 0x06)");
    ESP_LOGI(TAG, "XL9555 found, config0=0x%02x", cfg);
    return ESP_OK;
}

esp_err_t xl9555_pin_mode(i2c_master_dev_handle_t dev, uint8_t pin, bool output)
{
    uint8_t reg = 0, bit = 0;
    bit = xl9555_port_and_bit(pin, &reg);

    uint8_t cfg = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, reg, &cfg), TAG, "read cfg 0x%02x", reg);
    if (output) {
        cfg &= (uint8_t)~bit; /* 0 = output */
    } else {
        cfg |= bit;           /* 1 = input */
    }
    return xl9555_write_reg(dev, reg, cfg);
}

esp_err_t xl9555_set_output(i2c_master_dev_handle_t dev, uint8_t pin, bool level)
{
    uint8_t reg = (pin < 8) ? XL_REG_OUTP0 : XL_REG_OUTP1;
    uint8_t bit = (uint8_t)(1u << (pin % 8));

    uint8_t out = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, reg, &out), TAG, "read out 0x%02x", reg);
    if (level) {
        out |= bit;
    } else {
        out &= (uint8_t)~bit;
    }
    return xl9555_write_reg(dev, reg, out);
}

esp_err_t xl9555_read_input(i2c_master_dev_handle_t dev, uint8_t pin, bool *level)
{
    uint16_t state = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_port(dev, &state), TAG, "read port");
    *level = (state & (1u << pin)) != 0;
    return ESP_OK;
}

esp_err_t xl9555_read_port(i2c_master_dev_handle_t dev, uint16_t *state)
{
    uint8_t p0 = 0, p1 = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, XL_REG_INP0, &p0), TAG, "read inp0");
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, XL_REG_INP1, &p1), TAG, "read inp1");
    *state = (uint16_t)p0 | ((uint16_t)p1 << 8);
    return ESP_OK;
}
