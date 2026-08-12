/*
 * sensor_cache.c - background telemetry cache for slow I2C peripherals.
 *
 * A low-priority task polls the AXP2101 PMU and the PCF85063A RTC once per
 * second into a RAM struct. The LVGL/UI task reads this snapshot instead of
 * blocking on I2C, so rendering and swipe handling never stall on the bus.
 * Sensor data that is already cached in its own driver (BHI260AP, M10Q) is
 * deliberately not duplicated here.
 */
#include "sensor_cache.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "twatch_board.h"

static const char *TAG = "sensor_cache";

#define CACHE_PERIOD_MS   1000
#define CACHE_TASK_STACK  2048

static sensor_cache_t s_cache;
static SemaphoreHandle_t s_mux;
static TaskHandle_t s_task;

static void cache_task(void *arg)
{
    (void)arg;
    for (;;) {
        sensor_cache_t c = { 0 };
        c.rtc_valid = (pcf85063a_get_time(twatch_rtc_dev, &c.rtc) == ESP_OK);

        bool ok = true;
        if (axp2101_get_battery_pct(twatch_pmu_dev, &c.batt_pct) != ESP_OK) {
            ok = false;
        }
        if (axp2101_get_battery_mv(twatch_pmu_dev, &c.batt_mv) != ESP_OK) {
            ok = false;
        }
        if (axp2101_get_charge_status(twatch_pmu_dev, &c.chg_state) != ESP_OK) {
            ok = false;
        }
        if (axp2101_is_charge_enabled(twatch_pmu_dev, &c.chg_enabled) != ESP_OK) {
            ok = false;
        }
        if (axp2101_get_charge_current_ma(twatch_pmu_dev, &c.chg_ma) != ESP_OK) {
            ok = false;
        }
        if (axp2101_get_battery_temp(twatch_pmu_dev, &c.batt_temp_c10) != ESP_OK) {
            ok = false;
        }
        c.valid = ok;

        if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_cache = c;
            xSemaphoreGive(s_mux);
        }
        vTaskDelay(pdMS_TO_TICKS(CACHE_PERIOD_MS));
    }
}

void sensor_cache_init(void)
{
    if (s_task) {
        return;
    }
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
    xTaskCreate(cache_task, "sensor_cache", CACHE_TASK_STACK, NULL, 3, &s_task);
    ESP_LOGI(TAG, "telemetry cache task started (1 s)");
}

void sensor_cache_get(sensor_cache_t *out)
{
    if (!out) {
        return;
    }
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_cache;
        xSemaphoreGive(s_mux);
    }
}

bool sensor_cache_get_rtc(pcf85063a_time_t *out)
{
    if (!out) {
        return false;
    }
    bool valid = false;
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_cache.rtc;
        valid = s_cache.rtc_valid;
        xSemaphoreGive(s_mux);
    }
    return valid;
}
