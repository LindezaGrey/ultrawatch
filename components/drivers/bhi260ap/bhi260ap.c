#include "bhi260ap.h"
#include "esp_log.h"

static const char *TAG = "bhi260ap";

esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev)
{
    (void)dev;
    ESP_LOGW(TAG, "bhi260ap_init: not implemented");
    return ESP_OK;
}

esp_err_t bhi260ap_read_accel(i2c_master_dev_handle_t dev, bhi260ap_accel_t *accel)
{
    (void)dev;
    (void)accel;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bhi260ap_read_gyro(i2c_master_dev_handle_t dev, int16_t out[3])
{
    (void)dev;
    (void)out;
    return ESP_ERR_NOT_SUPPORTED;
}
