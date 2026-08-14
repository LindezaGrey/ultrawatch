#include "bsp_i2c.h"
#include "bsp_twatch_ultra.h"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "bsp_i2c";

/* Finite transaction timeout: a stalled/stuck slave (e.g. a flaky touch chip
 * holding the bus) must never block the UI loop indefinitely. */
#define BSP_I2C_TIMEOUT_MS    100
#define BSP_I2C_SLOW_LOG_US   20000

static esp_err_t i2c_op(const char *op, uint8_t addr, esp_err_t err, uint32_t t0)
{
    uint32_t dt = (uint32_t)esp_timer_get_time() - t0;
    if (dt > BSP_I2C_SLOW_LOG_US) {
        ESP_LOGW(TAG, "%s 0x%02X slow: %lu.%03lu ms -> %s", op, addr,
                 (unsigned long)(dt / 1000), (unsigned long)(dt % 1000),
                 esp_err_to_name(err));
    }
    return err;
}

i2c_master_bus_handle_t bsp_i2c_bus_handle = NULL;

esp_err_t bsp_i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BSP_I2C_SDA_PIN,
        .scl_io_num = BSP_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bsp_i2c_bus_handle), TAG, "new master bus failed");

    return ESP_OK;
}

static i2c_master_dev_handle_t get_dev(uint8_t addr)
{
    static i2c_master_dev_handle_t s_devs[8];
    static uint8_t s_addrs[8] = { 0 };
    static int s_count = 0;

    if (addr >= 0x08 && addr < 0x78) {
        for (int i = 0; i < s_count; i++) {
            if (s_addrs[i] == addr) {
                return s_devs[i];
            }
        }
        if (s_count < 8) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = BSP_I2C_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bsp_i2c_bus_handle, &dev_cfg, &s_devs[s_count]) == ESP_OK) {
                s_addrs[s_count] = addr;
                return s_devs[s_count++];
            }
        }
    }
    return NULL;
}

esp_err_t bsp_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev = get_dev(addr);
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, data, len, BSP_I2C_TIMEOUT_MS);
    return i2c_op("read_reg", addr, err, t0);
}

esp_err_t bsp_i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev = get_dev(addr);
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[16];
    if (len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = reg;
    for (size_t i = 0; i < len; i++) {
        buf[1 + i] = data[i];
    }
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    esp_err_t err = i2c_master_transmit(dev, buf, len + 1, BSP_I2C_TIMEOUT_MS);
    return i2c_op("write_reg", addr, err, t0);
}

esp_err_t bsp_i2c_write_raw(uint8_t addr, const uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev = get_dev(addr);
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    esp_err_t err = i2c_master_transmit(dev, data, len, BSP_I2C_TIMEOUT_MS);
    return i2c_op("write_raw", addr, err, t0);
}

esp_err_t bsp_i2c_read_raw(uint8_t addr, uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev = get_dev(addr);
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    esp_err_t err = i2c_master_receive(dev, data, len, BSP_I2C_TIMEOUT_MS);
    return i2c_op("read_raw", addr, err, t0);
}

esp_err_t bsp_i2c_transmit_receive(uint8_t addr, const uint8_t *cmd, size_t cmd_len,
                                   uint8_t *data, size_t len)
{
    i2c_master_dev_handle_t dev = get_dev(addr);
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    esp_err_t err = i2c_master_transmit_receive(dev, cmd, cmd_len, data, len, BSP_I2C_TIMEOUT_MS);
    return i2c_op("transmit_receive", addr, err, t0);
}
