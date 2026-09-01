/*
 * daily_log.c - per-minute steps + activity logging to the SD card.
 *
 * See daily_log.h for the public interface.
 */
#include "daily_log.h"
#include "sensor_cache.h"
#include "bhi260ap.h"
#include "pcf85063a.h"
#include "sd_log.h"
#include "esp_log.h"
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

static uint32_t s_steps;                 /* last sampled daily steps */
/* Backing storage for daily_log_get_activity_seconds()'s returned pointers,
 * refreshed fresh from bhi260ap_get_activity_ms() on each call - not a
 * periodically-sampled tally anymore, see that function. */
static uint32_t s_act_sec[DAILY_ACT_COUNT];
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
            fprintf(b, "date,time,pct,mv,charge_state,chg_enabled,chg_ma,temp_c10\n");
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
        fprintf(b, "%s,%s,%u,%u,%s,%u,%u,%d\n", stemp, stem,
                (unsigned)c.batt_pct, (unsigned)c.batt_mv,
                c.valid ? charge_state_name(c.chg_state) : "invalid",
                (unsigned)(c.chg_enabled ? 1 : 0), (unsigned)c.chg_ma,
                (int)c.batt_temp_c10);
        fclose(b);
    }
}

static bool s_have_last;
static uint32_t s_last_ymd;

/* One sample: called every DAILY_LOG_INTERVAL_MS from the shared
 * housekeeping task (main/housekeeping.c) - this used to be its own
 * always-running task's loop body; consolidated to reclaim a whole task
 * stack (see docs/application.md section 12). */
void daily_log_tick(void)
{
    pcf85063a_time_t rtc;
    if (!sensor_cache_get_rtc(&rtc)) {
        return;
    }
    /* The RTC stores UTC; "day" for the daily counters/CSV means the
     * wearer's LOCAL calendar day, so convert before computing ymd. */
    time_t epoch = pcf85063a_time_to_epoch(&rtc);
    struct tm lt;
    localtime_r(&epoch, &lt);
    uint32_t ymd = (uint32_t)(lt.tm_year + 1900) * 10000u
                   + (uint32_t)(lt.tm_mon + 1) * 100u + (uint32_t)lt.tm_mday;

    /* Get the lifetime total first: it drives the daily counter and
     * the CSV row alike. */
    uint32_t lifetime = 0;
    bhi260ap_get_step_count(&lifetime);

    /* One mounted session covers every SD touch this tick (day-roll
     * marker, header-on-first-run, the steps/activity rows) rather
     * than mounting/unmounting separately for each - they always
     * happen together, once a minute. */
    bool sd_ok = (sd_log_session_begin() == ESP_OK);

    if (s_have_last && ymd != s_last_ymd) {
        ESP_LOGI(TAG, "new day %lu", (unsigned long)ymd);
        if (sd_ok) {
            FILE *s = fopen(STEPS_CSV, "a");
            if (s) {
                fprintf(s, "# day %04u-%02u-%02u\n",
                        (unsigned)(lt.tm_year + 1900),
                        (unsigned)(lt.tm_mon + 1), (unsigned)lt.tm_mday);
                fclose(s);
            }
        }
    }
    s_last_ymd = ymd;
    s_have_last = true;

    /* Feed lifetime + day id into the daily counter (reboot-safe),
     * then cache today's total for display/logging. Same call also
     * resets the BHI260AP driver's activity-duration tally on a day
     * rollover (bhi260ap_daily_sample()), so there's one place
     * deciding "is it a new day" for both day-scoped counters. */
    bhi260ap_daily_sample(lifetime, ymd);
    bhi260ap_get_daily_steps(&s_steps);

    if (sd_ok) {
        if (!s_log_ready) {
            ensure_header();
            s_log_ready = true;
        }
        append_lines(&lt, lifetime);
    }
    sd_log_session_end();
}

esp_err_t daily_log_get_steps(uint32_t *steps)
{
    if (!steps) {
        return ESP_ERR_INVALID_ARG;
    }
    *steps = s_steps;
    return ESP_OK;
}

esp_err_t daily_log_get_activity_seconds(const uint32_t *out[DAILY_ACT_COUNT])
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Zero-filled default: bhi260ap_get_activity_ms() fails (returns
     * ESP_ERR_INVALID_STATE, leaving ms[] untouched) until the sensor
     * finishes booting, a normal condition in the first few seconds after
     * power-on - out[] must still always point at valid, defined data on
     * return, matching this function's original never-fails contract.
     * Callers (e.g. the dailylog console command) don't check the return
     * value, matching that original contract. */
    uint32_t ms[BHI260AP_ACTIVITY_COUNT] = {0};
    bhi260ap_get_activity_ms(ms);
    /* activity_class() maps each bhi260ap_activity_t 1:1 onto a
     * daily_activity_t (see the switch above), so this just re-buckets the
     * driver's per-class ms totals into daily_log's own enum and converts
     * to seconds. */
    memset(s_act_sec, 0, sizeof(s_act_sec));
    for (int i = 0; i < BHI260AP_ACTIVITY_COUNT; i++) {
        daily_activity_t c = activity_class((uint8_t)i);
        uint32_t add_s = ms[i] / 1000;
        if (s_act_sec[c] <= UINT32_MAX - add_s) {
            s_act_sec[c] += add_s;
        }
    }
    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        out[i] = &s_act_sec[i];
    }
    return ESP_OK;
}

void daily_log_flush(void)
{
    /* No buffered ring for CSV: lines are written directly per minute. */
}

