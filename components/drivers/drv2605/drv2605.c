#include "drv2605.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "drv2605";

esp_err_t drv2605_init(i2c_master_dev_handle_t dev)
{
    (void)dev;
    ESP_LOGW(TAG, "drv2605_init: not implemented");
    return ESP_OK;
}

esp_err_t drv2605_set_waveform(i2c_master_dev_handle_t dev, uint8_t slot, uint8_t wave)
{
    (void)dev;
    (void)slot;
    (void)wave;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t drv2605_go(i2c_master_dev_handle_t dev)
{
    (void)dev;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave)
{
    ESP_RETURN_ON_ERROR(drv2605_set_waveform(dev, 0, wave), TAG, "set_waveform failed");
    return drv2605_go(dev);
}
