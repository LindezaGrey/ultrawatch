#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include "twatch_board.h"
#include "pcf85063a.h"

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "hardware_foundation";

static void sync_system_time_from_rtc(void)
{
    pcf85063a_time_t rtc_time;
    esp_err_t err = pcf85063a_get_time(twatch_rtc_dev, &rtc_time);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RTC time unavailable: %s", esp_err_to_name(err));
        return;
    }
    struct timeval tv = {
        .tv_sec = pcf85063a_time_to_epoch(&rtc_time),
        .tv_usec = 0,
    };
    err = settimeofday(&tv, NULL);
    if (err != 0) {
        ESP_LOGW(TAG, "system clock sync failed (%d)", errno);
    } else {
        ESP_LOGI(TAG, "system clock initialized from RTC (UTC)");
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return;
    }

    err = twatch_board_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "T-Watch Ultra board init failed: %s", esp_err_to_name(err));
        return;
    }

    err = pcf85063a_init(twatch_rtc_dev);
    if (err == ESP_OK) {
        sync_system_time_from_rtc();
    } else {
        ESP_LOGW(TAG, "RTC init failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "hardware foundation ready; application/UI not included");
}
