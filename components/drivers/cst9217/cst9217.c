#include "cst9217.h"
#include "esp_log.h"

static const char *TAG = "cst9217";

esp_err_t cst9217_init(i2c_master_dev_handle_t dev)
{
    (void)dev;
    ESP_LOGW(TAG, "cst9217_init: not implemented");
    return ESP_OK;
}

esp_err_t cst9217_get_touch(i2c_master_dev_handle_t dev, cst9217_data_t *data)
{
    (void)dev;
    (void)data;
    return ESP_ERR_NOT_SUPPORTED;
}
