/*
 * services/sensor_cache.c - background telemetry cache for slow I2C peripherals.
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
#include <stdlib.h>

static const char *TAG = "sensor_cache";

#define CACHE_PERIOD_MS   1000
#define CACHE_TASK_STACK  2048

static sensor_cache_t s_cache;
static SemaphoreHandle_t s_mux;
static TaskHandle_t s_task;
static uint32_t s_sync_system_count;   /* periodic RTC -> system clock re-sync */

/* ---- Battery gauge rate tracker ----
 * Samples batt_pct each poll and derives a drain/charge rate over a rolling
 * window of active (awake) time. The AXP % register is 1%-resolution, so a
 * window of ~5 minutes is needed for a stable rate. */
#define GAUGE_WINDOW_MS   (5 * 60 * 1000)   /* 5 min rolling window */
#define GAUGE_MIN_DELTA   3                  /* need >=3% movement to estimate */
#define GAUGE_MIN_SAMPLES 2

typedef struct {
    uint32_t first_ms;   /* uptime of the window's first sample */
    uint8_t  first_pct;
    uint32_t last_ms;    /* uptime of the latest sample */
    uint8_t  last_pct;
    bool     first_set;
} gauge_window_t;

static gauge_window_t s_gauge;
static battery_estimate_t s_est;
static uint8_t s_gauge_prev_chg;   /* last charge state, to detect flips */

static void gauge_reset(void)
{
    s_gauge.first_set = false;
    s_gauge.first_ms = 0;
    s_gauge.first_pct = 0;
    s_gauge.last_ms = 0;
    s_gauge.last_pct = 0;
    s_est.estimate_valid = false;
    s_est.pct_per_hour = 0;
    s_est.runtime_h = 0;
    s_est.charge_h = 0;
}

/* Feed a battery % sample (with uptime ms) into the rolling tracker. */
static void gauge_feed(uint32_t now_ms, uint8_t pct, uint8_t chg_state)
{
    if (s_gauge_prev_chg != chg_state) {
        gauge_reset();            /* plug/unplug or charge toggle */
        s_gauge_prev_chg = chg_state;
    }
    if (!s_gauge.first_set) {
        s_gauge.first_ms = now_ms;
        s_gauge.first_pct = pct;
        s_gauge.first_set = true;
    }
    s_gauge.last_ms = now_ms;
    s_gauge.last_pct = pct;

    /* Drop old first sample when the window is full. */
    if (s_gauge.first_set && (now_ms - s_gauge.first_ms) > GAUGE_WINDOW_MS) {
        if (s_gauge.last_ms <= s_gauge.first_ms) {
            gauge_reset();
            s_gauge.first_set = false;
        }
        /* slide: re-base the first sample to the start of the next window */
        s_gauge.first_ms = now_ms - GAUGE_WINDOW_MS + 1000;
    }

    int32_t dpct = (int32_t)s_gauge.last_pct - s_gauge.first_pct;
    uint32_t dms = s_gauge.last_ms - s_gauge.first_ms;
    if (dms == 0 || abs((int)dpct) < GAUGE_MIN_DELTA) {
        s_est.estimate_valid = false;
        return;
    }
    float hours = dms / 3600000.0f;
    float pct_per_hour = (float)dpct / hours;

    s_est.estimate_valid = true;
    s_est.pct_per_hour = pct_per_hour;
    s_est.runtime_h = 0;
    s_est.charge_h = 0;
    if (pct_per_hour < 0) {
        s_est.runtime_h = s_gauge.last_pct / -pct_per_hour;
    } else if (pct_per_hour > 0) {
        s_est.charge_h = (100 - s_gauge.last_pct) / pct_per_hour;
    }
}

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
            /* Track battery % rate in the foreground/background (sampled while holding
 * the cache mutex; rate estimate reflects awake time at the time of polling). */
            gauge_feed((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                       c.batt_pct, (uint8_t)c.chg_state);
            xSemaphoreGive(s_mux);
        }

        /* The RTC is the authoritative clock; re-push it into the ESP32 system
         * clock every minute so time()/mktime/crash timestamps never drift. */
        if (++s_sync_system_count >= (60u * 1000u) / CACHE_PERIOD_MS) {
            s_sync_system_count = 0;
            twatch_board_sync_system_time();
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

void battery_estimate_get(battery_estimate_t *out)
{
    if (!out) {
        return;
    }
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_est;
        xSemaphoreGive(s_mux);
    }
}
