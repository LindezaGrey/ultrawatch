#include "pcf85063a.h"
#include "esp_log.h"

static const char *TAG = "pcf85063a";

esp_err_t pcf85063a_init(i2c_master_dev_handle_t dev)
{
    (void)dev;
    ESP_LOGW(TAG, "pcf85063a_init: not implemented");
    return ESP_OK;
}

esp_err_t pcf85063a_get_time(i2c_master_dev_handle_t dev, pcf85063a_time_t *t)
{
    (void)dev;
    (void)t;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t pcf85063a_set_time(i2c_master_dev_handle_t dev, const pcf85063a_time_t *t)
{
    (void)dev;
    (void)t;
    return ESP_ERR_NOT_SUPPORTED;
}
