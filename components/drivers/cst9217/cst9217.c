#include "cst9217.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_cst9217.h"
#include "cst9217_logic.h"
static const char *TAG = "cst9217";

static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_touch_handle_t s_touch = NULL;
static uint16_t s_res_x = 0;
static uint16_t s_res_y = 0;
static bool s_resolution_valid;

esp_err_t cst9217_init(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_io || s_touch) {
        return ESP_ERR_INVALID_STATE;
    }
    s_res_x = 0;
    s_res_y = 0;
    s_resolution_valid = false;
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
    io_cfg.dev_addr = CST9217_I2C_ADDR;   /* T-Watch Ultra touch is at 0x1A */
    io_cfg.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &s_io), TAG, "touch io init");

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 410,
        .y_max = 502,
        .rst_gpio_num = -1,   /* reset is on XL9555 P8 (board layer) */
        .int_gpio_num = CST9217_TOUCH_INT_GPIO,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };
    esp_err_t err = esp_lcd_touch_new_i2c_cst9217(s_io, &tp_cfg, &s_touch);
    if (err != ESP_OK) {
        esp_lcd_panel_io_del(s_io);
        s_io = NULL;
        return err;
    }

    /* The vendor constructor reads and logs the controller's resolution.
     * Export the board geometry for application-level coordinate handling. */
    s_res_x = 410;
    s_res_y = 502;
    s_resolution_valid = cst9217_resolution_is_valid(s_res_x, s_res_y);
    ESP_LOGI(TAG, "CST9217 configured resolution: %ux%u", s_res_x, s_res_y);
    return ESP_OK;
}

esp_err_t cst9217_get_resolution(uint16_t *x, uint16_t *y)
{
    if (!x || !y) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_res_x || !s_res_y) {
        return ESP_ERR_INVALID_STATE;
    }
    *x = s_res_x;
    *y = s_res_y;
    return ESP_OK;
}

esp_lcd_touch_handle_t cst9217_get_handle(void)
{
    return s_touch;
}
