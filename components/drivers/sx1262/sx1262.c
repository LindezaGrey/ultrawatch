#include "sx1262.h"
#include "esp_log.h"

static const char *TAG = "sx1262";

esp_err_t sx1262_init(spi_device_handle_t spi)
{
    (void)spi;
    ESP_LOGW(TAG, "sx1262_init: not implemented");
    return ESP_OK;
}

esp_err_t sx1262_set_frequency(uint32_t freq_hz)
{
    (void)freq_hz;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sx1262_send(const uint8_t *buf, size_t len, int timeout_ms)
{
    (void)buf;
    (void)len;
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sx1262_recv(uint8_t *buf, size_t len, int timeout_ms)
{
    (void)buf;
    (void)len;
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}
