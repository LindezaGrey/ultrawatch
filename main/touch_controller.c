/*
 * touch_controller.c - CST9217 lifecycle and touch wake boundary.
 */
#include "touch_controller.h"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "twatch_board.h"
#include "cst9217.h"

static const char *TAG = "touch_controller";
static bool s_initialized;

esp_err_t touch_controller_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(twatch_board_reset_touch(), TAG, "touch reset");
    ESP_RETURN_ON_ERROR(cst9217_init(twatch_i2c_bus), TAG, "CST9217 init");
    s_initialized = true;
    ESP_LOGI(TAG, "CST9217 ready");
    return ESP_OK;
}

esp_lcd_touch_handle_t touch_controller_get_handle(void)
{
    return s_initialized ? cst9217_get_handle() : NULL;
}

esp_err_t touch_controller_get_resolution(uint16_t *x, uint16_t *y)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG,
                        "CST9217 not initialized");
    return cst9217_get_resolution(x, y);
}

esp_err_t touch_controller_set_wake_enabled(bool enabled)
{
    if (enabled) {
        return gpio_wakeup_enable(TWATCH_PIN_TOUCH_INT, GPIO_INTR_LOW_LEVEL);
    }
    return gpio_wakeup_disable(TWATCH_PIN_TOUCH_INT);
}
