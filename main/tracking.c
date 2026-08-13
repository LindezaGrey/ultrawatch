/*
 * tracking.c - step-gated distance tracking.
 *
 * A tracking session keeps the watch awake (display stays on; the watch face
 * shows a red dot) and uses the always-on BHI260AP step counter as the
 * trigger: every TRACK_STEPS_PER_FIX steps the GNSS receiver is pulsed once
 * for a position fix, the haversine distance since the previous fix is
 * accumulated, and the receiver is powered back off. Lifetime distance + steps
 * are persisted in NVS ("track" namespace) so the average step length can be
 * computed (dist_cm / steps).
 *
 * The session is driven by a background task that watches the step counter and
 * flags "fix due" to the GNSS control task in lvgl_app, which does the actual
 * (blocking) m10q power on/fix/off on its own task.
 */
#include "tracking.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bhi260ap.h"
#include "m10q.h"

static const char *TAG = "tracking";

#define TRACK_NVS_NS       "track"
#define NVS_KEY_DIST_CM    "dist_cm"
#define NVS_KEY_STEPS      "steps"

/* Pulse GNSS once per this many steps (~35-40 m when walking). */
#define TRACK_STEPS_PER_FIX    50
#define TRACK_POLL_MS          1000

/* Activity classes that count as "active" for GNSS pulsing + distance. */
static bool track_activity_active(void)
{
    uint8_t act = BHI260AP_ACTIVITY_UNKNOWN;
    if (bhi260ap_get_activity(&act) != ESP_OK) {
        return true;   /* fail-open: don't stall tracking on a sensor glitch */
    }
    return (act == BHI260AP_ACTIVITY_WALKING || act == BHI260AP_ACTIVITY_RUNNING);
}

/* ---- Persisted lifetime totals ---- */
static tracking_totals_t s_totals;

/* ---- Session state ---- */
static bool s_active;
static uint32_t s_base_steps;        /* step counter at session start */
static uint32_t s_last_fix_steps;    /* steps at the last GNSS fix */
static uint32_t s_session_dist_cm;   /* distance accumulated this session */
static bool s_session_has_pos;       /* a fix has been recorded this session */
static double s_prev_lat, s_prev_lon;
static uint16_t s_prev_course_deg;   /* last GNSS course at the last fix */
static bool s_fix_due;

static TaskHandle_t s_task;

static void totals_load(void)
{
    nvs_handle_t h;
    if (nvs_open(TRACK_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_DIST_CM, &s_totals.dist_cm);
        nvs_get_u32(h, NVS_KEY_STEPS, &s_totals.steps);
        nvs_close(h);
    }
}

static void totals_save(void)
{
    nvs_handle_t h;
    if (nvs_open(TRACK_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_DIST_CM, s_totals.dist_cm);
        nvs_set_u32(h, NVS_KEY_STEPS, s_totals.steps);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void tracking_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TRACK_POLL_MS));
        if (!s_active) {
            continue;
        }
        /* Keep the watch awake (no auto-sleep) so the BHI step counter keeps
         * running and the GNSS rail isn't cut mid-session. */
        esp_lv_adapter_report_activity();
        uint32_t steps = 0;
        if (bhi260ap_get_step_count(&steps) != ESP_OK) {
            continue;
        }
        /* Gate on activity: only pulse GNSS while walking/running. When the
         * user is in a vehicle/on a bike/stationary, reset the step baseline so
         * the next walk starts a fresh 50-step window (no fix burst after a
         * drive). */
        if (!track_activity_active()) {
            s_last_fix_steps = steps;
            s_fix_due = false;
            continue;
        }
        /* Every TRACK_STEPS_PER_FIX new steps -> ask for a GNSS fix. */
        if ((steps - s_last_fix_steps) >= TRACK_STEPS_PER_FIX) {
            s_fix_due = true;
            ESP_LOGI(TAG, "track: %lu steps since last fix, GNSS fix due",
                     (unsigned long)(steps - s_last_fix_steps));
        }
    }
}

/* ---- Public API ---- */

void tracking_init(void)
{
#if TRACKING_ENABLED
    totals_load();
    if (s_task == NULL) {
        xTaskCreate(tracking_task, "track", 2048, NULL, 2, &s_task);
    }
    ESP_LOGI(TAG, "tracking ready: dist %.0f m, steps %lu",
             s_totals.dist_cm / 100.0, (unsigned long)s_totals.steps);
#else
    ESP_LOGI(TAG, "tracking disabled (TRACKING_ENABLED=0)");
#endif
}

esp_err_t tracking_start(void)
{
#if TRACKING_ENABLED
    if (s_active) {
        return ESP_OK;
    }
    uint32_t steps = 0;
    if (bhi260ap_get_step_count(&steps) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    s_active = true;
    s_base_steps = steps;
    s_last_fix_steps = steps;
    s_session_dist_cm = 0;
    s_session_has_pos = false;
    s_fix_due = false;
    ESP_LOGI(TAG, "tracking started (base steps %lu)", (unsigned long)steps);
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t tracking_stop(void)
{
#if TRACKING_ENABLED
    if (!s_active) {
        return ESP_OK;
    }
    uint32_t steps = 0;
    if (bhi260ap_get_step_count(&steps) == ESP_OK && steps >= s_base_steps) {
        s_totals.steps += (steps - s_base_steps);
    }
    s_totals.dist_cm += s_session_dist_cm;
    totals_save();
    s_active = false;
    ESP_LOGI(TAG, "tracking stopped: +%lu steps, +%.0f m (total %.2f km / %lu steps)",
             (unsigned long)(steps >= s_base_steps ? steps - s_base_steps : 0),
             s_session_dist_cm / 100.0,
             s_totals.dist_cm / 100000.0, (unsigned long)s_totals.steps);
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool tracking_is_active(void)
{
    return s_active;
}

bool tracking_is_gated_active(void)
{
    return s_active && track_activity_active();
}

bool tracking_get_estimated_position(double *lat, double *lon)
{
    if (!lat || !lon || !s_session_has_pos) {
        return false;
    }
    uint32_t steps = 0;
    if (bhi260ap_get_step_count(&steps) != ESP_OK || steps <= s_last_fix_steps) {
        *lat = s_prev_lat;
        *lon = s_prev_lon;
        return true;
    }
    /* Distance moved since the last fix: steps x average step length. */
    uint32_t avg_cm = tracking_get_avg_step_cm();
    double stride = (avg_cm > 0) ? avg_cm / 100.0 : 0.70;   /* default 0.7 m */
    double dist_m = (double)(steps - s_last_fix_steps) * stride;

    /* Project along the last known GNSS course (magnetic north on the M10 is
     * true north; no compass on the BHI, so this is the heading of travel). */
    double course = s_prev_course_deg * 3.14159265358979 / 180.0;
    double dlat = dist_m * cos(course) / 111320.0;
    double cos_lat = cos(s_prev_lat * 3.14159265358979 / 180.0);
    if (cos_lat < 0.01) {
        cos_lat = 0.01;   /* avoid division by zero at extreme latitudes */
    }
    double dlon = dist_m * sin(course) / (111320.0 * cos_lat);
    *lat = s_prev_lat + dlat;
    *lon = s_prev_lon + dlon;
    return true;
}

void tracking_on_fix(double lat, double lon)
{
    if (!s_active) {
        return;
    }
    if (s_session_has_pos) {
        double d = m10q_distance_m(s_prev_lat, s_prev_lon, lat, lon);
        if (d >= 1.0) {   /* ignore sub-metre jitter */
            s_session_dist_cm += (uint32_t)(d * 100.0);
        }
    }
    s_prev_lat = lat;
    s_prev_lon = lon;
    s_session_has_pos = true;

    /* Remember the course at this fix for the next position estimate. */
    m10q_fix_t f;
    if (m10q_get_fix(&f) == ESP_OK) {
        s_prev_course_deg = f.course_deg;
    }

    uint32_t steps = 0;
    bhi260ap_get_step_count(&steps);
    s_last_fix_steps = steps;
    s_fix_due = false;
    ESP_LOGI(TAG, "track: fix at (%.5f, %.5f), session dist %.0f m",
             lat, lon, s_session_dist_cm / 100.0);
}

bool tracking_fix_due(void)
{
#if TRACKING_ENABLED
    return s_active && s_fix_due;
#else
    return false;
#endif
}

void tracking_fix_clear(void)
{
    s_fix_due = false;
    uint32_t steps = 0;
    bhi260ap_get_step_count(&steps);
    s_last_fix_steps = steps;
}

void tracking_get_totals(tracking_totals_t *totals)
{
    if (totals) {
        /* Report lifetime totals including the current session. */
        uint32_t session_steps = tracking_get_session_steps();
        uint32_t total_dist = s_totals.dist_cm + s_session_dist_cm;
        uint32_t total_steps = s_totals.steps + session_steps;
        totals->dist_cm = total_dist;
        totals->steps = total_steps;
    }
}

uint32_t tracking_get_session_steps(void)
{
    if (!s_active) {
        return 0;
    }
    uint32_t steps = 0;
    if (bhi260ap_get_step_count(&steps) != ESP_OK) {
        return 0;
    }
    return (steps >= s_base_steps) ? (steps - s_base_steps) : 0;
}

uint32_t tracking_get_avg_step_cm(void)
{
    tracking_totals_t t;
    tracking_get_totals(&t);
    if (t.steps == 0) {
        return 0;
    }
    return t.dist_cm / t.steps;
}
