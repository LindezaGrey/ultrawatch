#include "st25r3916.h"
#include "esp_log.h"

static const char *TAG = "st25r3916";

esp_err_t st25r3916_init(spi_device_handle_t spi)
{
    (void)spi;
    ESP_LOGW(TAG, "st25r3916_init: not implemented");
    return ESP_OK;
}

esp_err_t st25r3916_poll(st25r3916_tag_t *tag, int timeout_ms)
{
    (void)tag;
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}
