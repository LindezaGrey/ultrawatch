#include "m10q.h"
#include "esp_log.h"

static const char *TAG = "m10q";

esp_err_t m10q_init(void)
{
    ESP_LOGW(TAG, "m10q_init: not implemented (esp_driver_uart + NMEA parser)");
    return ESP_OK;
}

esp_err_t m10q_get_fix(m10q_fix_t *fix)
{
    (void)fix;
    return ESP_ERR_NOT_SUPPORTED;
}
