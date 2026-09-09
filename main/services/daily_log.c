/*
 * services/daily_log.c - per-minute steps + activity logging to the SD card.
 *
 * See daily_log.h for the public interface.
 */
#include "daily_log.h"
#include "daily_archive.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <sys/stat.h>
#include "sensor_cache.h"
#include "bhi260ap.h"
#include "pcf85063a.h"
#include "sd_log.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "daily_log";

#define DAILY_DIR      "/sdcard/log"
#define STEPS_CSV      "/sdcard/log/steps.csv"
#define ACTIVITY_CSV   "/sdcard/log/activity.csv"
/* Battery telemetry, same per-minute cadence as the other two. Added after an
 * overnight run burned ~45% with no way to see where it went: steps.csv and
 * activity.csv record what the user did, nothing recorded what the power did.
 * Every field comes from sensor_cache (already sampled in the background), so
 * this costs one more fopen/fprintf per minute inside the SD session that is
 * being opened anyway - no extra I2C and no extra card mount. */
#define BATTERY_CSV    "/sdcard/log/battery.csv"
#define LOG_INTERVAL_MS 60000   /* 1 min */

static SemaphoreHandle_t s_log_mutex;
static uint32_t s_steps;
#define TOTALS_CSV "/sdcard/log/activity_totals.csv"
#define SUMMARY_DIR "/sdcard/log/daily"
void daily_log_init(void) { if (!s_log_mutex) s_log_mutex = xSemaphoreCreateMutex(); }
static bool s_log_ready;                 /* CSV header written this boot */

/* Short, greppable names for the AXP2101 charge state, so the CSV is readable
 * without a decoder ring. */
static const char *charge_state_name(axp2101_charge_state_t st)
{
    switch (st) {
    case AXP2101_CHG_TRI:  return "trickle";
    case AXP2101_CHG_PRE:  return "pre";
    case AXP2101_CHG_CC:   return "cc";
    case AXP2101_CHG_CV:   return "cv";
    case AXP2101_CHG_DONE: return "done";
    case AXP2101_CHG_STOP: return "stop";
    default:               return "?";
    }
}

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

/* Write the CSV header if the file is new/empty on this boot. Called only
 * from within a sd_log_session_begin()/end() bracket (see daily_log_task). */
static void ensure_header(void)
{
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
    FILE *b = fopen(BATTERY_CSV, "a");
    if (b) {
        if (ftell(b) == 0) {
            fprintf(b, "date,time,uptime_s,pct,mv,charge_state,chg_enabled,chg_ma,temp_c10\n");
        }
        fclose(b);
    }
}

/* Called only from within a sd_log_session_begin()/end() bracket (see
 * daily_log_task). */
static void append_lines(const struct tm *lt, uint32_t lifetime)
{
    char stemp[24], stem[24];
    snprintf(stemp, sizeof(stemp), "%04u-%02u-%02u", (unsigned)(lt->tm_year + 1900),
             (unsigned)(lt->tm_mon + 1), (unsigned)lt->tm_mday);
    snprintf(stem, sizeof(stem), "%02u:%02u", (unsigned)lt->tm_hour, (unsigned)lt->tm_min);

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
    /* Battery sample. Written even when the cache is stale (valid==false) -
     * a row of zeros with the timestamp still shows the gap, which is more
     * useful when reconstructing an overnight drain than a missing line. */
    FILE *b = fopen(BATTERY_CSV, "a");
    if (b) {
        sensor_cache_t c;
        sensor_cache_get(&c);
        /* date and time as two real columns, matching the header. The other
         * two CSVs join them with a space into a single field despite their
         * headers claiming otherwise; not changed there, because those files
         * already hold history in that shape and re-splitting mid-file would
         * break the existing series. */
        /* uptime comes from the kernel, not from sensor_cache like the
         * timestamp and the battery figures do. On 2026-09-02 the cache task
         * stopped updating for ~19 minutes: housekeeping kept writing a row a
         * minute, but every row carried the same stale time (07:26) and the
         * same stale 3844 mV, so the file looked like a 19-minute gap when in
         * fact it was 20 frozen samples. With uptime alongside, the two are
         * immediately distinguishable - uptime climbing while the timestamp
         * stands still means the cache is stale, both standing still means the
         * logger itself stopped. */
        fprintf(b, "%s,%s,%lu,%u,%u,%s,%u,%u,%d\n", stemp, stem,
                (unsigned long)(esp_timer_get_time() / 1000000),
                (unsigned)c.batt_pct, (unsigned)c.batt_mv,
                c.valid ? charge_state_name(c.chg_state) : "invalid",
                (unsigned)(c.chg_enabled ? 1 : 0), (unsigned)c.chg_ma,
                (int)c.batt_temp_c10);
        fclose(b);
    }
}

static bool local_date(struct tm *lt, uint32_t *ymd)
{
    pcf85063a_time_t rtc;
    if (!sensor_cache_get_rtc(&rtc) || rtc.year < 2020 || rtc.year > 2099 ||
        rtc.month < 1 || rtc.month > 12 || rtc.day < 1 || rtc.day > 31) return false;
    time_t epoch = pcf85063a_time_to_epoch(&rtc);
    if (!localtime_r(&epoch, lt)) return false;
    *ymd = (uint32_t)(lt->tm_year + 1900) * 10000u + (lt->tm_mon + 1) * 100u + lt->tm_mday;
    return true;
}

/* Reconcile once per second from housekeeping, with no SD access. */
void daily_log_reconcile(void)
{
    struct tm lt;
    uint32_t ymd;
    if (local_date(&lt, &ymd)) bhi260ap_daily_update(ymd, false);
}

void daily_log_tick(void)
{
    if (!s_log_mutex) return;
    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    struct tm lt;
    uint32_t ymd;
    if (!local_date(&lt, &ymd)) goto done;
    /* Checkpointing works without an SD card. Never invent zero sensor readings. */
    esp_err_t update = bhi260ap_daily_update(ymd, true);
    if (sd_log_session_begin() != ESP_OK) {
        sd_log_session_end();
        goto done;
    }
    mkdir(SUMMARY_DIR, 0755);
    activity_day_t pending;
    while (bhi260ap_daily_pending(&pending) == ESP_OK) {
        if (!daily_archive_write(SUMMARY_DIR, &pending)) {
            ESP_LOGW(TAG, "daily summary %lu deferred", (unsigned long)pending.ymd);
            break;
        }
        if (bhi260ap_daily_ack(pending.ymd) != ESP_OK) break;
    }
    /* A full pending queue may have just been drained. */
    if (update != ESP_OK) update = bhi260ap_daily_update(ymd, true);
    activity_day_t day;
    if (update == ESP_OK && bhi260ap_daily_snapshot(&day) == ESP_OK && day.ymd == ymd) {
        s_steps = day.steps;
        if (!s_log_ready) { ensure_header(); s_log_ready = true; }
        append_lines(&lt, day.lifetime);
        FILE *file = fopen(TOTALS_CSV, "a");
        if (file) {
            char clock[12], row[256];
            strftime(clock, sizeof(clock), "%H:%M:%S", &lt);
            daily_totals_format(row, sizeof(row), &day, clock);
            bool ok = true;
            if (ftell(file) == 0 && fputs(DAILY_TOTALS_HEADER, file) < 0) ok = false;
            if (fputs(row, file) < 0) ok = false;
            if (fclose(file) != 0) ok = false;
            if (!ok) ESP_LOGW(TAG, "activity totals append failed");
        }
    }
    sd_log_session_end();
done:
    xSemaphoreGive(s_log_mutex);
}

esp_err_t daily_log_get_steps(uint32_t *steps)
{
    return steps ? bhi260ap_get_daily_steps(steps) : ESP_ERR_INVALID_ARG;
}

esp_err_t daily_log_get_activity_seconds(uint32_t out[DAILY_ACT_COUNT])
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, DAILY_ACT_COUNT * sizeof(*out));
    uint32_t ms[BHI260AP_ACTIVITY_COUNT] = {0};
    esp_err_t err = bhi260ap_get_activity_ms(ms);
    if (err != ESP_OK) return err;
    for (int i = 0; i < BHI260AP_ACTIVITY_COUNT; i++) out[activity_class((uint8_t)i)] += ms[i] / 1000;
    return ESP_OK;
}

void daily_log_flush(void)
{
    daily_log_tick();
    /* Also save a live checkpoint if RTC/SD logging was unavailable. */
    bhi260ap_daily_checkpoint();
}
