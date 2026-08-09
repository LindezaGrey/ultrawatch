#include "cst9217.h"
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_cst9217.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cst9217";

#define CST9217_CMD_MODE_REG   0xD101
#define CST9217_RESOLUTION_REG 0xD1F8

static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_touch_handle_t s_touch = NULL;
static uint16_t s_res_x = 0;
static uint16_t s_res_y = 0;

static esp_err_t cst9217_read_resolution(void)
{
    uint8_t hi = CST9217_CMD_MODE_REG >> 8;
    uint8_t lo = CST9217_CMD_MODE_REG & 0xff;
    uint8_t val = 0x01;   /* enter command mode */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(s_io, hi, &lo, 1), TAG, "cmd-mode addr");
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(s_io, val, NULL, 0), TAG, "cmd-mode val");
    vTaskDelay(pdMS_TO_TICKS(2));

    hi = CST9217_RESOLUTION_REG >> 8;
    lo = CST9217_RESOLUTION_REG & 0xff;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(s_io, hi, &lo, 1), TAG, "res addr");
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t data[4] = { 0 };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(s_io, -1, data, sizeof(data)), TAG, "res read");
    s_res_x = (uint16_t)((data[1] << 8) | data[0]);
    s_res_y = (uint16_t)((data[3] << 8) | data[2]);
    ESP_LOGI(TAG, "CST9217 native resolution: %ux%u", s_res_x, s_res_y);
    return ESP_OK;
}

esp_err_t cst9217_init(i2c_master_bus_handle_t bus)
{
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
    io_cfg.dev_addr = CST9217_I2C_ADDR;   /* T-Watch Ultra touch is at 0x1A */
    io_cfg.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &s_io), TAG, "touch io init");

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 410,   /* display width; adapter scales native coords */
        .y_max = 502,
        .rst_gpio_num = -1,   /* reset is on XL9555 P8 (board layer) */
        .int_gpio_num = -1,   /* polling via the LVGL adapter */
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_cst9217(s_io, &tp_cfg, &s_touch), TAG, "cst9217 new");

    /* Native resolution (best-effort; used for coordinate scaling). */
    cst9217_read_resolution();

    return ESP_OK;
}

esp_err_t cst9217_get_resolution(uint16_t *x, uint16_t *y)
{
    if (x) *x = s_res_x;
    if (y) *y = s_res_y;
    return (s_res_x && s_res_y) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_lcd_touch_handle_t cst9217_get_handle(void)
{
    return s_touch;
}
