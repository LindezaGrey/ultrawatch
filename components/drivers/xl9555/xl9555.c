#include "xl9555.h"
#include "esp_check.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "xl9555_logic.h"

static const char *TAG = "xl9555";

#define XL9555_I2C_TIMEOUT_MS 100

#define XL_REG_INP0   0x00
#define XL_REG_INP1   0x01
#define XL_REG_CFG0   0x06

static esp_err_t xl9555_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_bus_read(dev, reg, val, 1, XL9555_I2C_TIMEOUT_MS);
}

static esp_err_t xl9555_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    return i2c_bus_write(dev, reg, &val, 1, XL9555_I2C_TIMEOUT_MS);
}

/* 1 = input, 0 = output (config register semantics). */
static bool xl9555_port_and_bit(uint8_t pin, uint8_t *reg, uint8_t *out_reg, uint8_t *bit)
{
    return xl9555_pin_registers(pin, reg, out_reg, bit);
}

static SemaphoreHandle_t s_port_mutex;
static portMUX_TYPE s_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t xl9555_get_mutex(void)
{
    portENTER_CRITICAL(&s_mutex_init_lock);
    SemaphoreHandle_t mutex = s_port_mutex;
    portEXIT_CRITICAL(&s_mutex_init_lock);
    return mutex;
}

/* Caller holds s_port_mutex across the complete read-modify-write sequence. */
static esp_err_t xl9555_update_bit_locked(i2c_master_dev_handle_t dev,
                                           uint8_t reg, uint8_t bit, bool value)
{
    uint8_t port = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, reg, &port), TAG, "read port 0x%02x", reg);
    if (value) {
        port |= bit;
    } else {
        port &= (uint8_t)~bit;
    }
    return xl9555_write_reg(dev, reg, port);
}

esp_err_t xl9555_init(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!xl9555_get_mutex()) {
        SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
        if (!mutex) {
            return ESP_ERR_NO_MEM;
        }
        portENTER_CRITICAL(&s_mutex_init_lock);
        if (!s_port_mutex) {
            s_port_mutex = mutex;
            mutex = NULL;
        }
        portEXIT_CRITICAL(&s_mutex_init_lock);
        if (mutex) {
            vSemaphoreDelete(mutex);
        }
    }
    uint8_t cfg = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, 0x06, &cfg), TAG, "probe failed (reg 0x06)");
    ESP_LOGI(TAG, "XL9555 found, config0=0x%02x", cfg);
    return ESP_OK;
}

esp_err_t xl9555_pin_mode(i2c_master_dev_handle_t dev, uint8_t pin, bool output)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    SemaphoreHandle_t mutex = xl9555_get_mutex();
    if (!mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t reg = 0, out_reg = 0, bit = 0;
    if (!xl9555_port_and_bit(pin, &reg, &out_reg, &bit)) {
        return ESP_ERR_INVALID_ARG;
    }
    (void)out_reg;

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t cfg = 0;
    esp_err_t err = xl9555_read_reg(dev, reg, &cfg);
    if (err == ESP_OK) {
        if (output) cfg &= (uint8_t)~bit; /* 0 = output */
        else cfg |= bit;                  /* 1 = input */
        err = xl9555_write_reg(dev, reg, cfg);
    }
    xSemaphoreGive(mutex);
    return err;
}

esp_err_t xl9555_config_output(i2c_master_dev_handle_t dev, uint8_t pin, bool initial_level)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    SemaphoreHandle_t mutex = xl9555_get_mutex();
    if (!mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t cfg_reg = 0, out_reg = 0, bit = 0;
    if (!xl9555_pin_registers(pin, &cfg_reg, &out_reg, &bit)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = xl9555_update_bit_locked(dev, out_reg, bit, initial_level);
    if (err == ESP_OK) {
        uint8_t cfg = 0;
        err = xl9555_read_reg(dev, cfg_reg, &cfg);
        if (err == ESP_OK) {
            err = xl9555_write_reg(dev, cfg_reg, (uint8_t)(cfg & (uint8_t)~bit));
        }
    }
    xSemaphoreGive(mutex);
    return err;
}

esp_err_t xl9555_set_output(i2c_master_dev_handle_t dev, uint8_t pin, bool level)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    SemaphoreHandle_t mutex = xl9555_get_mutex();
    if (!mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t cfg_reg = 0, out_reg = 0, bit = 0;
    if (!xl9555_pin_registers(pin, &cfg_reg, &out_reg, &bit)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t cfg = 0;
    esp_err_t err = xl9555_read_reg(dev, cfg_reg, &cfg);
    if (err == ESP_OK && (cfg & bit)) {
        err = ESP_ERR_INVALID_STATE;
    }
    if (err == ESP_OK) {
        err = xl9555_update_bit_locked(dev, out_reg, bit, level);
    }
    xSemaphoreGive(mutex);
    return err;
}

esp_err_t xl9555_read_input(i2c_master_dev_handle_t dev, uint8_t pin, bool *level)
{
    if (!dev || !level) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pin >= 16) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t state = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_port(dev, &state), TAG, "read port");
    *level = (state & (1u << pin)) != 0;
    return ESP_OK;
}

esp_err_t xl9555_read_port(i2c_master_dev_handle_t dev, uint16_t *state)
{
    if (!dev || !state) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t p0 = 0, p1 = 0;
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, XL_REG_INP0, &p0), TAG, "read inp0");
    ESP_RETURN_ON_ERROR(xl9555_read_reg(dev, XL_REG_INP1, &p1), TAG, "read inp1");
    *state = (uint16_t)p0 | ((uint16_t)p1 << 8);
    return ESP_OK;
}
