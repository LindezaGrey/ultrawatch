/*
 * daily_log.c - per-minute steps + activity logging to the SD card.
 *
 * See daily_log.h for the public interface.
 */
#include "daily_log.h"
#include "sensor_cache.h"
#include "bhi260ap.h"
#include "sd_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "daily_log";

#define DAILY_DIR      "/sdcard/log"
#define STEPS_CSV      "/sdcard/log/steps.csv"
#define ACTIVITY_CSV   "/sdcard/log/activity.csv"
#define LOG_INTERVAL_MS 60000   /* 1 min */

static SemaphoreHandle_t s_mux;
static uint32_t s_steps;                 /* last sampled daily steps */
static uint16_t s_act_min[DAILY_ACT_COUNT];
static bool s_log_ready;                 /* CSV header written this boot */

static daily_activity_t activity_class(uint8_t a)
{
    switch (a) {
    case BHI260AP_ACTIVITY_STILL:      return DAILY_ACT_STILL;
    case BHI260AP_ACTIVITY_WALKING:    return DAILY_ACT_WALKING;
    case BHI260AP_ACTIVITY_RUNNING:    return DAILY_ACT_RUNNING;
    case BHI260AP_ACTIVITY_ON_BICYCLE: return DAILY_ACT_CYCLING;
    case BHI260AP_ACTIVITY_IN_VEHICLE: return DAILY_ACT_VEHICLE;
    case BHI260AP_ACTIVITY_TILTING:    return DAILY_ACT_TILTING;
    default:                           return DAILY_ACT_UNKNOWN;
    }
}

static const char *class_name(daily_activity_t c)
{
    switch (c) {
    case DAILY_ACT_STILL:   return "still";
    case DAILY_ACT_WALKING: return "walking";
    case DAILY_ACT_RUNNING: return "running";
    case DAILY_ACT_CYCLING: return "cycling";
    case DAILY_ACT_VEHICLE: return "in_vehicle";
    case DAILY_ACT_TILTING: return "tilting";
    default:                return "unknown";
    }
}

/* Write the CSV header if the file is new/empty on this boot. */
static void ensure_header(void)
{
    if (!sd_log_available()) {
        return;
    }
    FILE *s = fopen(STEPS_CSV, "a");
    if (s) {
        if (ftell(s) == 0) {
            fprintf(s, "date,time,day_steps,lifetime_steps\n");
        }
        fclose(s);
    }
    FILE *a = fopen(ACTIVITY_CSV, "a");
    if (a) {
        if (ftell(a) == 0) {
            fprintf(a, "date,time,activity\n");
        }
        fclose(a);
    }
}

static void append_lines(const pcf85063a_time_t *rtc, uint32_t lifetime)
{
    if (!sd_log_available()) {
        return;
    }
    char stemp[24], stem[24];
    snprintf(stemp, sizeof(stemp), "%04u-%02u-%02u", (unsigned)rtc->year,
             (unsigned)rtc->month, (unsigned)rtc->day);
    snprintf(stem, sizeof(stem), "%02u:%02u", (unsigned)rtc->hour, (unsigned)rtc->min);

    FILE *s = fopen(STEPS_CSV, "a");
    if (s) {
        fprintf(s, "%s %s,%lu,%lu\n", stemp, stem, (unsigned long)s_steps,
                (unsigned long)lifetime);
        fclose(s);
    }
    FILE *a = fopen(ACTIVITY_CSV, "a");
    if (a) {
        /* Determine the class that was current at this sample. */
        uint8_t act = BHI260AP_ACTIVITY_UNKNOWN;
        bhi260ap_get_activity(&act);
        fprintf(a, "%s %s,%s\n", stemp, stem, class_name(activity_class(act)));
        fclose(a);
    }
}

static void daily_log_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));   /* let boot settle */

    bool have_last = false;
    uint32_t last_ymd = 0;

    for (;;) {
        pcf85063a_time_t rtc;
        if (sensor_cache_get_rtc(&rtc)) {
            uint32_t ymd = (uint32_t)rtc.year * 10000u + (uint32_t)rtc.month * 100
                           + (uint32_t)rtc.day;

            /* Get the lifetime total first: it drives the daily counter and
             * the CSV row alike. */
            uint32_t lifetime = 0;
            bhi260ap_get_step_count(&lifetime);

            if (have_last && ymd != last_ymd) {
                /* New calendar day: reset activity tallies + CSV separator.
                 * The daily step counter resets itself inside daily_sample. */
                ESP_LOGI(TAG, "new day %lu, activity tallies reset",
                         (unsigned long)ymd);
                xSemaphoreTake(s_mux, pdMS_TO_TICKS(100));
                for (int i = 0; i < DAILY_ACT_COUNT; i++) {
                    s_act_min[i] = 0;
                }
                xSemaphoreGive(s_mux);
                if (sd_log_available()) {
                    FILE *s = fopen(STEPS_CSV, "a");
                    if (s) {
                        fprintf(s, "# day %04u-%02u-%02u\n", (unsigned)rtc.year,
                                (unsigned)rtc.month, (unsigned)rtc.day);
                        fclose(s);
                    }
                }
            }
            last_ymd = ymd;
            have_last = true;

            /* Feed lifetime + day id into the daily counter (reboot-safe),
             * then cache today's total for display/logging. */
            bhi260ap_daily_sample(lifetime, ymd);
            bhi260ap_get_daily_steps(&s_steps);

            /* Credit one minute to the current activity class. */
            uint8_t act = BHI260AP_ACTIVITY_UNKNOWN;
            bhi260ap_get_activity(&act);
            daily_activity_t c = activity_class(act);
            xSemaphoreTake(s_mux, pdMS_TO_TICKS(100));
            if (s_act_min[c] != 0xFFFF) {
                s_act_min[c]++;
            }
            xSemaphoreGive(s_mux);

            if (!s_log_ready) {
                ensure_header();
                s_log_ready = true;
            }
            append_lines(&rtc, lifetime);
        }
        vTaskDelay(pdMS_TO_TICKS(LOG_INTERVAL_MS));
    }
}

esp_err_t daily_log_get_steps(uint32_t *steps)
{
    if (!steps) {
        return ESP_ERR_INVALID_ARG;
    }
    *steps = s_steps;
    return ESP_OK;
}

esp_err_t daily_log_get_activity_minutes(const uint16_t *out[DAILY_ACT_COUNT])
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mux, pdMS_TO_TICKS(100));
    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        out[i] = &s_act_min[i];
    }
    xSemaphoreGive(s_mux);
    return ESP_OK;
}

void daily_log_flush(void)
{
    /* No buffered ring for CSV: lines are written directly per minute. */
}

void daily_log_init(void)
{
    if (s_mux == NULL) {
        s_mux = xSemaphoreCreateMutex();
    }
    if (s_mux == NULL) {
        return;
    }
    xTaskCreate(daily_log_task, "daily_log", 3072, NULL, 2, NULL);
}
