/*
 * gpx_log.c - GPX track logging to the SD card (see gpx_log.h for the API).
 *
 * Implementation notes:
 *  - Directory/filename follow crash_dump.c's timestamped-file idiom:
 *    /sdcard/log/gpx/track_<YYYYMMDD_HHMMSS>.gpx, one file per session.
 *  - Interval logging follows daily_log.c's dedicated-task idiom: one
 *    always-running FreeRTOS task, woken every 30 s, a no-op unless a
 *    session is active. All SD I/O happens on this task, never the LVGL
 *    task.
 *  - Append-only, fopen(path, "a") per operation, no persistent handle -
 *    same as daily_log.c/crash_dump.c. Accepted tradeoff: if the watch
 *    loses power mid-session (not a clean gpx_log_stop()), the file is
 *    left without its closing </trkseg></trk></gpx> tags - technically
 *    invalid XML, though most GPX readers tolerate it. Not worth a more
 *    complex rewrite-whole-file-every-tick scheme for a personal device.
 *  - Each trackpoint's <time> uses the RTC (UTC-native, no TZ conversion
 *    needed - GPX times must be UTC/Zulu); lat/lon/altitude come from
 *    m10q_get_fix(), skipped (gap in the track, not a bad point) if the
 *    fix isn't valid at that tick.
 */
#include "gpx_log.h"
#include "sensor_cache.h"
#include "pcf85063a.h"
#include "m10q.h"
#include "sd_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static const char *TAG = "gpx_log";

#define GPX_DIR          "/sdcard/log/gpx"
#define GPX_LOG_INTERVAL_MS 30000   /* 30 s, per docs/application.md section 6 */

static volatile bool s_active;
static uint32_t s_point_count;
static char s_path[64];

static void make_timestamp(char *buf, size_t len)
{
    /* System clock is synced from the RTC at boot; TZ is set in app_main -
     * same as crash_dump.c's make_timestamp(), just for the filename. */
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    strftime(buf, len, "%Y%m%d_%H%M%S", &tmv);
}

esp_err_t gpx_log_start(void)
{
    if (s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sd_log_available()) {
        return ESP_ERR_NOT_FOUND;
    }
    mkdir(GPX_DIR, 0755);

    char ts[24];
    make_timestamp(ts, sizeof(ts));
    snprintf(s_path, sizeof(s_path), "%s/track_%s.gpx", GPX_DIR, ts);

    FILE *f = fopen(s_path, "a");
    if (!f) {
        ESP_LOGW(TAG, "start: fopen %s failed", s_path);
        return ESP_FAIL;
    }
    fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
               "<gpx version=\"1.1\" creator=\"UWatch\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
               "<trk><name>%s</name><trkseg>\n", ts);
    fclose(f);

    s_point_count = 0;
    s_active = true;
    ESP_LOGI(TAG, "started: %s", s_path);
    return ESP_OK;
}

esp_err_t gpx_log_stop(void)
{
    if (!s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    s_active = false;
    if (sd_log_available()) {
        FILE *f = fopen(s_path, "a");
        if (f) {
            fprintf(f, "</trkseg></trk></gpx>\n");
            fclose(f);
        }
    }
    ESP_LOGI(TAG, "stopped: %s (%lu points)", s_path, (unsigned long)s_point_count);
    return ESP_OK;
}

bool gpx_log_is_active(void)
{
    return s_active;
}

uint32_t gpx_log_point_count(void)
{
    return s_point_count;
}

const char *gpx_log_current_path(void)
{
    return s_path;
}

static void append_trackpoint(void)
{
    if (!sd_log_available()) {
        return;
    }
    m10q_fix_t fix;
    if (m10q_get_fix(&fix) != ESP_OK || !fix.valid) {
        ESP_LOGD(TAG, "tick: no valid fix, skipping point");
        return;
    }
    pcf85063a_time_t rtc;
    if (!sensor_cache_get_rtc(&rtc)) {
        ESP_LOGD(TAG, "tick: RTC unavailable, skipping point");
        return;
    }
    time_t epoch = pcf85063a_time_to_epoch(&rtc);
    struct tm utc;
    gmtime_r(&epoch, &utc);
    char ts[24];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &utc);

    FILE *f = fopen(s_path, "a");
    if (!f) {
        ESP_LOGW(TAG, "tick: fopen %s failed", s_path);
        return;
    }
    fprintf(f, "<trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%.1f</ele><time>%s</time></trkpt>\n",
            fix.lat, fix.lon, fix.alt_m, ts);
    fclose(f);
    s_point_count++;
}

static void gpx_log_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(GPX_LOG_INTERVAL_MS));
        if (s_active) {
            append_trackpoint();
        }
    }
}

void gpx_log_init(void)
{
    xTaskCreate(gpx_log_task, "gpx_log", 3072, NULL, 2, NULL);
}
