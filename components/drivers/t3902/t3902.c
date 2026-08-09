#include "t3902.h"
#include "esp_log.h"

static const char *TAG = "t3902";

esp_err_t t3902_init(void)
{
    ESP_LOGW(TAG, "t3902_init: not implemented (esp_driver_i2s, I2S_PDM_RX_MODE)");
    return ESP_OK;
}

esp_err_t t3902_read(int16_t *samples, size_t n)
{
    (void)samples;
    (void)n;
    return ESP_ERR_NOT_SUPPORTED;
}
