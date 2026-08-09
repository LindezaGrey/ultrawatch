#include "max98357a.h"
#include "esp_log.h"

static const char *TAG = "max98357a";

esp_err_t max98357a_init(void)
{
    ESP_LOGW(TAG, "max98357a_init: not implemented (esp_driver_i2s, I2S_STD_MODE)");
    return ESP_OK;
}

esp_err_t max98357a_write(const int16_t *data, size_t n)
{
    (void)data;
    (void)n;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t max98357a_set_volume(float db)
{
    (void)db;
    return ESP_ERR_NOT_SUPPORTED;
}
