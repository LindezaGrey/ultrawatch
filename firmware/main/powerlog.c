#include "powerlog.h"

#include "bsp_twatch_ultra.h"
#include "bsp_sdcard.h"
#include "bsp_pcf85063.h"
#include "bsp_axp2101.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "powerlog";

#define POWERLOG_INTERVAL_MS   (5 * 60 * 1000)
#define POWERLOG_RETRY_MS      30000

static bool s_initialized;
static uint32_t s_last_log_ms;

static void log_line(void)
{
    struct tm tm;
    if (bsp_rtc_get_time(&tm) != ESP_OK) {
        ESP_LOGW(TAG, "rtc unavailable, skipping");
        return;
    }
    uint8_t pct = bsp_axp2101_battery_percent();
    uint16_t mv = bsp_axp2101_battery_voltage_mv();

    char path[64];
    snprintf(path, sizeof(path), BSP_SD_MOUNT_POINT "/power.log");

    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGW(TAG, "cannot open %s", path);
        return;
    }
    if (ftell(f) == 0) {
        fprintf(f, "datetime,capacity_pct,voltage_mv\n");
    }
    fprintf(f, "%04d-%02d-%02d %02d:%02d:%02d,%d,%d\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, pct, mv);
    fflush(f);
    fclose(f);

    ESP_LOGI(TAG, "%04d-%02d-%02d %02d:%02d:%02d pct=%d mv=%d",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec, pct, mv);
}

esp_err_t powerlog_init(void)
{
    s_last_log_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_initialized = true;
    ESP_LOGI(TAG, "power logger ready (every %lu s)", (unsigned long)(POWERLOG_INTERVAL_MS / 1000));
    return ESP_OK;
}

void powerlog_tick(void)
{
    if (!s_initialized) {
        return;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (now_ms - s_last_log_ms < POWERLOG_INTERVAL_MS) {
        return;
    }

    if (!bsp_sd_mounted() && bsp_sd_init() != ESP_OK) {
        /* Card absent or mount failed: retry in a bit, don't hammer the bus */
        s_last_log_ms = now_ms - POWERLOG_INTERVAL_MS + POWERLOG_RETRY_MS;
        return;
    }

    log_line();
    s_last_log_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}
