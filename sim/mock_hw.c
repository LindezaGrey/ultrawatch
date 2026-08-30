/*
 * mock_hw.c - fake driver data for the host simulator.
 *
 * Wall-clock time comes from the host's real clock (via localtime), so the
 * watch face shows real time exactly like a synced RTC would. Battery,
 * steps, GPS, BHI orientation, tracking and alarm state all come from
 * synthetic generators or simple static-variable-backed setters/getters (the
 * same style as the original GNSS-state-by-time mock) so every screen has
 * something to render and, where the real screen has a control (switch,
 * button), touching it visibly changes what's mocked.
 */
#include "mock_hw.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Monotonic seconds with sub-second resolution, used to animate the BHI cube
 * smoothly (time(NULL) alone only ticks once a second) and to timestamp
 * synthetic mesh message/node ages. Built on lv_tick_get() (ms since
 * SDL_Init, driven by SDL_GetTicks() - see main.c) rather than
 * CLOCK_MONOTONIC (machine uptime, unrelated and much larger): UI code that
 * computes an age via "lv_tick_get() - some_mock_timestamp" needs both
 * sides on the same clock, or the subtraction underflows into a huge
 * wrapped uint32_t (confirmed live: the Node-Overview screen showed
 * "4293901188s ago" before this fix). */
static double mock_now_s(void)
{
    return (double)lv_tick_get() / 1000.0;
}

/* ---- RTC / watch face (unchanged) ---- */

/* Charge enable/current, set by the power screen's switch + current buttons
 * (axp2101_set_charge_enabled/axp2101_set_charge_current_ma below), read
 * back by sensor_cache_get() so the power screen reflects what was set. */
static bool s_mock_chg_enabled = true;
static uint16_t s_mock_chg_ma = 100;

void sensor_cache_get(sensor_cache_t *out)
{
    /* Battery drains from 100% to 20% over 5 simulated minutes, for real
     * (host) elapsed time - long enough to watch the bar move, short
     * enough to actually see it move during a dev session. */
    double elapsed_s = (double)time(NULL) - (double)0;
    int pct = 100 - (int)(((long)elapsed_s % 300) * 80 / 300);
    out->batt_pct = (uint8_t)pct;
    /* Rough mV curve over the same 3.3-4.2 V range as the percentage. */
    out->batt_mv = (uint16_t)(3300 + pct * 9);

    out->chg_enabled = s_mock_chg_enabled;
    out->chg_ma = s_mock_chg_ma;
    if (!out->chg_enabled) {
        out->chg_state = AXP2101_CHG_STOP;
    } else {
        /* Cycle through a plausible charge-state sequence over a 40 s loop
         * so the power screen's charge-state text visibly changes. */
        switch (((long)elapsed_s / 8) % 5) {
        case 0:  out->chg_state = AXP2101_CHG_TRI; break;
        case 1:  out->chg_state = AXP2101_CHG_PRE; break;
        case 2:  out->chg_state = AXP2101_CHG_CC; break;
        case 3:  out->chg_state = AXP2101_CHG_CV; break;
        default: out->chg_state = AXP2101_CHG_DONE; break;
        }
    }
    out->batt_temp_c10 = (int16_t)(250 + ((long)elapsed_s % 50));   /* ~25.0-29.9 C */
    out->valid = true;
}

bool sensor_cache_get_rtc(pcf85063a_time_t *out)
{
    /* Mirrors the real RTC: fields are UTC, not local. */
    time_t now = time(NULL);
    struct tm gt;
    gmtime_r(&now, &gt);
    out->sec = (uint8_t)gt.tm_sec;
    out->min = (uint8_t)gt.tm_min;
    out->hour = (uint8_t)gt.tm_hour;
    out->day = (uint8_t)gt.tm_mday;
    out->weekday = (uint8_t)(gt.tm_wday + 1);  /* pcf85063a: 1=Sun..7=Sat */
    out->month = (uint8_t)(gt.tm_mon + 1);
    out->year = (uint16_t)(gt.tm_year + 1900);
    return true;
}

/* Same civil<->days algorithm as the real driver (Howard Hinnant's
 * date_algorithms.html), so this is TZ-independent exactly like the real
 * pcf85063a_time_to_epoch() the ported watch-face code expects. */
time_t pcf85063a_time_to_epoch(const pcf85063a_time_t *t)
{
    int y = t->year;
    unsigned m = t->month, d = t->day;
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = era * 146097 + (int64_t)doe - 719468;
    return (time_t)(days * 86400 + t->hour * 3600 + t->min * 60 + t->sec);
}

void battery_estimate_get(battery_estimate_t *out)
{
    out->estimate_valid = true;
    out->pct_per_hour = -4.5f;
    out->runtime_h = 4.75f;
    out->charge_h = 1.5f;
}

/* ---- axp2101 (power screen controls) ---- */

i2c_master_dev_handle_t twatch_pmu_dev = (void *)0x1;   /* dummy non-NULL handle */

esp_err_t axp2101_set_charge_enabled(i2c_master_dev_handle_t dev, bool enable)
{
    (void)dev;
    s_mock_chg_enabled = enable;
    return ESP_OK;
}

esp_err_t axp2101_set_charge_current_ma(i2c_master_dev_handle_t dev, uint16_t ma)
{
    (void)dev;
    s_mock_chg_ma = ma;
    return ESP_OK;
}

/* ---- power_mgmt (power screen switches) ---- */

static bool s_pm_night_mode_auto = true;
static bool s_pm_skip_sleep_on_usb = true;

bool power_mgmt_get_night_mode_auto(void) { return s_pm_night_mode_auto; }
void power_mgmt_set_night_mode_auto(bool on) { s_pm_night_mode_auto = on; }
bool power_mgmt_get_skip_sleep_on_usb(void) { return s_pm_skip_sleep_on_usb; }
void power_mgmt_set_skip_sleep_on_usb(bool yes) { s_pm_skip_sleep_on_usb = yes; }

static uint32_t s_pm_display_timeout_s = 5;
static uint8_t s_pm_brightness = 0x80;

uint32_t power_mgmt_get_display_timeout_s(void) { return s_pm_display_timeout_s; }
void power_mgmt_set_display_timeout_s(uint32_t seconds) { s_pm_display_timeout_s = seconds; }
uint8_t power_mgmt_get_brightness(void) { return s_pm_brightness; }
void power_mgmt_set_brightness(uint8_t level) { s_pm_brightness = level; }

static bool s_pm_sparmodus_active = false;
bool power_mgmt_get_sparmodus_active(void) { return s_pm_sparmodus_active; }
void power_mgmt_set_sparmodus_active(bool on) { s_pm_sparmodus_active = on; }

/* ---- m10q / GPS screen ----
 * GNSS defaults OFF (matches the firmware's default persisted setting) and is
 * only "powered" via mock_gnss_set_enabled(), driven by the GPS screen's
 * power switch. Once enabled it reports ACQUIRING for a few seconds, then
 * FIXED with a fabricated position + skyplot, exactly like a warm GNSS
 * start. */
static bool s_gnss_enabled = false;
static time_t s_gnss_on_since = 0;

void mock_gnss_set_enabled(bool on)
{
    if (on && !s_gnss_enabled) {
        s_gnss_on_since = time(NULL);
    }
    s_gnss_enabled = on;
}

void lvgl_gps_set_enabled(bool on)
{
    mock_gnss_set_enabled(on);
}

m10q_state_t m10q_get_state(void)
{
    if (!s_gnss_enabled) {
        return M10Q_STATE_OFF;
    }
    return ((time(NULL) - s_gnss_on_since) < 5) ? M10Q_STATE_ACQUIRING : M10Q_STATE_FIXED;
}

esp_err_t m10q_get_fix(m10q_fix_t *fix)
{
    memset(fix, 0, sizeof(*fix));
    m10q_state_t st = m10q_get_state();
    fix->valid = (st == M10Q_STATE_FIXED);
    fix->fix_3d = fix->valid;
    if (!fix->valid) {
        return ESP_OK;
    }

    double t = mock_now_s();
    fix->lat = 52.5200 + 0.0004 * sin(t / 47.0);    /* Berlin, slowly drifting */
    fix->lon = 13.4050 + 0.0004 * cos(t / 53.0);
    fix->alt_m = 34.0 + 2.0 * sin(t / 30.0);
    fix->speed_kmh = (uint16_t)(3 + 2 * fabs(sin(t / 20.0)));
    fix->course_deg = (uint16_t)fmod(t * 4.0, 360.0);
    fix->hdop = 12;
    fix->hacc_m = 5;

    time_t now = time(NULL);
    struct tm utc;
    gmtime_r(&now, &utc);
    fix->hour = (uint8_t)utc.tm_hour;
    fix->minute = (uint8_t)utc.tm_min;
    fix->second = (uint8_t)utc.tm_sec;
    fix->rtc_offset_s = 0;

    /* Eight fabricated satellites scattered across the sky, varying SNR so
     * the skyplot's color-coded dots and Used/tracked ring are all visibly
     * exercised. */
    static const struct { uint8_t prn; int16_t el; int16_t az; int16_t snr; bool used; } fake[] = {
        { 2,  62,  45, 41, true },
        { 5,  38, 120, 33, true },
        { 9,  71, 260, 45, true },
        { 12, 20, 300, 22, false },
        { 15, 55, 200, 38, true },
        { 18, 12,  10, 18, false },
        { 21, 44, 340, 30, true },
        { 24, 28, 165, 25, true },
    };
    int n = (int)(sizeof(fake) / sizeof(fake[0]));
    fix->sat_in_view = (uint16_t)n;
    fix->sat_count = 0;
    for (int i = 0; i < n; i++) {
        fix->sats[i].prn = fake[i].prn;
        fix->sats[i].elevation_deg = fake[i].el;
        fix->sats[i].azimuth_deg = fake[i].az;
        fix->sats[i].snr_db = fake[i].snr;
        fix->sats[i].used = fake[i].used;
        if (fake[i].used) {
            fix->sat_count++;
        }
    }
    return ESP_OK;
}

void m10q_get_dbg(uint32_t *rx_bytes, uint32_t *nmea_lines)
{
    uint32_t up = (uint32_t)time(NULL);
    if (rx_bytes) *rx_bytes = s_gnss_enabled ? up * 37u : 0;
    if (nmea_lines) *nmea_lines = s_gnss_enabled ? up % 5000u : 0;
}

esp_err_t m10q_get_stats(m10q_stats_t *stats)
{
    stats->total_fixes = 42;
    stats->fixes_today = 3;
    stats->ttf_avg_ms = 4200;
    stats->ttf_best_ms = 2100;
    return ESP_OK;
}

uint32_t m10q_get_gsv_count(void)
{
    return s_gnss_enabled ? (uint32_t)time(NULL) % 200u : 0;
}

/* ---- bhi260ap / BHI status screen ---- */

esp_err_t bhi260ap_get_daily_steps(uint32_t *steps)
{
    /* One "step" every 2s, purely so the counter visibly moves. */
    *steps = (uint32_t)(time(NULL) % 100000) / 2;
    return ESP_OK;
}

esp_err_t bhi260ap_get_status(bool *ready, uint32_t *steps)
{
    if (ready) *ready = true;
    if (steps) bhi260ap_get_daily_steps(steps);
    return ESP_OK;
}

esp_err_t bhi260ap_get_rotation(int16_t *x, int16_t *y, int16_t *z, int16_t *w,
                                uint16_t *accuracy)
{
    /* Continuous yaw spin about Y so the cube is visibly rotating - the main
     * visual payoff of the BHI screen. */
    double angle = fmod(mock_now_s() * 0.6, 2.0 * M_PI);
    double qy = sin(angle / 2.0);
    double qw = cos(angle / 2.0);
    if (x) *x = 0;
    if (y) *y = (int16_t)(qy * 16384.0);
    if (z) *z = 0;
    if (w) *w = (int16_t)(qw * 16384.0);
    if (accuracy) *accuracy = 3;
    return ESP_OK;
}

esp_err_t bhi260ap_get_activity(uint8_t *activity)
{
    /* Cycle through the activity classes every 6 s so activity_name() text
     * is exercised. */
    static const uint8_t seq[] = {
        BHI260AP_ACTIVITY_STILL, BHI260AP_ACTIVITY_WALKING, BHI260AP_ACTIVITY_RUNNING,
        BHI260AP_ACTIVITY_ON_BICYCLE, BHI260AP_ACTIVITY_IN_VEHICLE, BHI260AP_ACTIVITY_TILTING,
    };
    *activity = seq[(time(NULL) / 6) % (sizeof(seq) / sizeof(seq[0]))];
    return ESP_OK;
}

esp_err_t bhi260ap_consume_gestures(bool *wrist_tilt, bool *wake_gesture, bool *glance,
                                    bool *pickup, bool *tilt)
{
    /* Pulse a different gesture true once per 15 s window (fires for the
     * duration of that one second) so the "Gesture:" line is exercised
     * without needing real motion input. */
    static const int period = 15;
    long phase = time(NULL) % period;
    bool fire = (phase == 0);
    int which = (int)((time(NULL) / period) % 5);
    if (wrist_tilt) *wrist_tilt = fire && which == 0;
    if (wake_gesture) *wake_gesture = fire && which == 1;
    if (glance) *glance = fire && which == 2;
    if (pickup) *pickup = fire && which == 3;
    if (tilt) *tilt = fire && which == 4;
    return ESP_OK;
}

/* ---- daily_log / BHI screen per-activity minutes ---- */

/* Old per-minute values (320,74,12,6,18,4,2) x60, except Walk: that one
 * grows with real elapsed seconds (wrapping every 5 min) instead of sitting
 * at a fixed value, so the seconds-precision fix (vs. the firmware's old
 * minute-only tally) is actually visible in the sim without waiting a full
 * minute for the number to move. */
esp_err_t daily_log_get_activity_seconds(const uint32_t *out[DAILY_ACT_COUNT])
{
    static uint32_t vals[DAILY_ACT_COUNT];
    vals[DAILY_ACT_STILL] = 19200;
    vals[DAILY_ACT_WALKING] = (uint32_t)time(NULL) % 300;
    vals[DAILY_ACT_RUNNING] = 720;
    vals[DAILY_ACT_CYCLING] = 360;
    vals[DAILY_ACT_VEHICLE] = 1080;
    vals[DAILY_ACT_TILTING] = 240;
    vals[DAILY_ACT_UNKNOWN] = 120;
    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        out[i] = &vals[i];
    }
    return ESP_OK;
}

/* ---- tracking / GPS screen Start-Stop button ----
 * Session totals accumulate while active (~1.5 m/s, ~2 steps/s), same as a
 * plausible walking pace, so distance/steps visibly grow once tracking is
 * started. */
static bool s_tracking_active = false;
static time_t s_tracking_started_at = 0;
static uint32_t s_tracking_base_dist_cm = 0;
static uint32_t s_tracking_base_steps = 0;

bool tracking_is_active(void) { return s_tracking_active; }

void tracking_get_totals(tracking_totals_t *totals)
{
    uint32_t dist = s_tracking_base_dist_cm;
    uint32_t steps = s_tracking_base_steps;
    if (s_tracking_active) {
        long elapsed = (long)(time(NULL) - s_tracking_started_at);
        dist += (uint32_t)(elapsed * 150);
        steps += (uint32_t)(elapsed * 2);
    }
    totals->dist_cm = dist;
    totals->steps = steps;
}

void lvgl_tracking_start(void)
{
    if (!s_tracking_active) {
        s_tracking_active = true;
        s_tracking_started_at = time(NULL);
    }
}

/* ---- GPX logging (gps_track_btn_cb on the GPS screen) ----
 * Point count derived from elapsed real time / the firmware's 30 s
 * interval, rather than an actual ticking timer, so the count visibly
 * grows during a sim session without needing a background task here. */
static bool s_gpx_active;
static time_t s_gpx_started_at;
static uint32_t s_gpx_point_count;

esp_err_t gpx_log_start(void)
{
    s_gpx_active = true;
    s_gpx_started_at = time(NULL);
    s_gpx_point_count = 0;
    return ESP_OK;
}

esp_err_t gpx_log_stop(void)
{
    s_gpx_active = false;
    return ESP_OK;
}

bool gpx_log_is_active(void)
{
    return s_gpx_active;
}

uint32_t gpx_log_point_count(void)
{
    if (s_gpx_active) {
        s_gpx_point_count = (uint32_t)(time(NULL) - s_gpx_started_at) / 30;
    }
    return s_gpx_point_count;
}

void lvgl_tracking_stop(void)
{
    if (s_tracking_active) {
        tracking_totals_t t;
        tracking_get_totals(&t);
        s_tracking_base_dist_cm = t.dist_cm;
        s_tracking_base_steps = t.steps;
        s_tracking_active = false;
    }
}

/* ---- mesh screen ----
 * Static synthetic entries (newest first, like the real ring buffer's
 * mesh_log_get_recent() order), one of each kind: a decoded text message
 * (one long enough to exercise the row's CLIP-mode horizontal truncation),
 * a decoded NodeInfo, a decoded non-NodeInfo portnum, and an unknown-channel
 * entry - covering all three mesh_msg_kind_t row stylings. */

static bool s_mesh_notify_enabled = true;
bool mesh_log_get_notify_enabled(void) { return s_mesh_notify_enabled; }
void mesh_log_set_notify_enabled(bool enabled) { s_mesh_notify_enabled = enabled; }

size_t mesh_log_get_recent(mesh_msg_t *out, size_t max)
{
    static const struct { uint32_t from; const char *text; uint8_t channel_hash; mesh_msg_kind_t kind; double age_s; } msgs[] = {
        { 0x55c80e38, "Hello from the trailhead, see you at the summit around noon", 0x08, MESH_MSG_TEXT,    12.0 },
        { 0x1a2b3c4d, "Zest",                                                       0x40, MESH_MSG_TEXT,    340.0 },
        { 0x2ee119ab, "NodeInfo: Kevin Hester",                                     0x08, MESH_MSG_OTHER,   610.0 },
        { 0x7c3d9a11, "",                                                           0x93, MESH_MSG_UNKNOWN, 900.0 },
        { 0x9f8e7d6c, "Good morning",                                               0x08, MESH_MSG_TEXT,    1820.0 },
    };
    size_t n = sizeof(msgs) / sizeof(msgs[0]);
    if (n > max) {
        n = max;
    }
    int64_t now_us = (int64_t)(mock_now_s() * 1e6);
    for (size_t i = 0; i < n; i++) {
        out[i].from = msgs[i].from;
        strncpy(out[i].text, msgs[i].text, MESH_LOG_TEXT_MAX);
        out[i].text[MESH_LOG_TEXT_MAX] = '\0';
        out[i].channel_hash = msgs[i].channel_hash;
        out[i].kind = msgs[i].kind;
        out[i].rssi_dbm = -97;
        out[i].snr_db = 6;
        out[i].received_at_us = now_us - (int64_t)(msgs[i].age_s * 1e6);
    }
    return n;
}

/* ---- mesh node table ----
 * Static synthetic nodes, matching some of mesh_log_get_recent()'s fake
 * senders above (0x2ee119ab has a name from its NodeInfo message; the
 * others don't, to exercise both the named and hex-ID row stylings on the
 * Node-Overview screen). */
size_t mesh_log_get_nodes(mesh_node_t *out, size_t max)
{
    static const struct { uint32_t node_id; const char *name; int16_t rssi; int8_t snr; double age_s; } nodes[] = {
        { 0x2ee119ab, "Kevin Hester", -82, 5,  610.0 },
        { 0x55c80e38, "",             -97, 6,  12.0 },
        { 0x1a2b3c4d, "",             -103, 2, 340.0 },
        { 0x9f8e7d6c, "",             -91, 8,  1820.0 },
    };
    size_t n = sizeof(nodes) / sizeof(nodes[0]);
    if (n > max) {
        n = max;
    }
    int64_t now_us = (int64_t)(mock_now_s() * 1e6);
    for (size_t i = 0; i < n; i++) {
        out[i].node_id = nodes[i].node_id;
        snprintf(out[i].name, sizeof(out[i].name), "%s", nodes[i].name);
        out[i].last_rssi_dbm = nodes[i].rssi;
        out[i].last_snr_db = nodes[i].snr;
        out[i].last_seen_us = now_us - (int64_t)(nodes[i].age_s * 1e6);
    }
    return n;
}

size_t mesh_log_node_count(void)
{
    mesh_node_t tmp[MESH_NODE_TABLE_MAX];
    return mesh_log_get_nodes(tmp, MESH_NODE_TABLE_MAX);
}

void mesh_log_node_name(uint32_t node_id, char *out, size_t outlen)
{
    if (outlen > 0) {
        out[0] = '\0';
    }
    mesh_node_t nodes[MESH_NODE_TABLE_MAX];
    size_t n = mesh_log_get_nodes(nodes, MESH_NODE_TABLE_MAX);
    for (size_t i = 0; i < n; i++) {
        if (nodes[i].node_id == node_id) {
            snprintf(out, outlen, "%s", nodes[i].name);
            break;
        }
    }
}

/* ---- alarm / alarms-timers screens ----
 * In-RAM only (no NVS on the host); seeded with two example alarms so the
 * list screen has something to render without needing to add one by hand
 * first. Ring/dismiss/snooze state is tracked but nothing ever actually
 * *fires* here (no RTC alarm interrupt on the host) - the ring screen is
 * only reachable via main.c's 'R'/'T' dev-preview keys, same as before this
 * overhaul; see ring_screen.c. */
static alarm_entry_t s_alarms[ALARM_MAX_COUNT] = {
    { .in_use = true, .enabled = true, .hour = 7, .min = 0,
      .ring_mode = ALARM_RING_BEEP, .weekday_mask = 0x3E /* Mon-Fri */ },
    { .in_use = true, .enabled = false, .hour = 9, .min = 30,
      .ring_mode = ALARM_RING_BOTH, .weekday_mask = ALARM_WEEKDAY_ALL },
};
static bool s_alarm_ringing;
static bool s_alarm_snoozing;
static int s_alarm_ringing_idx = 0;

esp_err_t alarm_check(void)
{
    return ESP_OK;
}

bool alarm_is_ringing(void)
{
    return s_alarm_ringing;
}

static bool s_alarm_sound_enabled = true;
bool alarm_get_sound_enabled(void) { return s_alarm_sound_enabled; }
void alarm_set_sound_enabled(bool enabled) { s_alarm_sound_enabled = enabled; }

bool alarm_is_snoozing(void)
{
    return s_alarm_snoozing;
}

int alarm_add(uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask)
{
    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        if (!s_alarms[i].in_use) {
            s_alarms[i] = (alarm_entry_t){ .in_use = true, .enabled = true, .hour = hour,
                                           .min = min, .ring_mode = ring_mode,
                                           .weekday_mask = weekday_mask ? weekday_mask : ALARM_WEEKDAY_ALL };
            return i;
        }
    }
    return -1;
}

esp_err_t alarm_update(int idx, uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask)
{
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_alarms[idx].in_use) {
        return ESP_ERR_INVALID_ARG;
    }
    s_alarms[idx].hour = hour;
    s_alarms[idx].min = min;
    s_alarms[idx].ring_mode = ring_mode;
    s_alarms[idx].weekday_mask = weekday_mask ? weekday_mask : ALARM_WEEKDAY_ALL;
    return ESP_OK;
}

esp_err_t alarm_remove(int idx)
{
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_alarms[idx].in_use) {
        return ESP_ERR_INVALID_ARG;
    }
    s_alarms[idx] = (alarm_entry_t){ 0 };
    return ESP_OK;
}

esp_err_t alarm_set_enabled(int idx, bool enabled)
{
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_alarms[idx].in_use) {
        return ESP_ERR_INVALID_ARG;
    }
    s_alarms[idx].enabled = enabled;
    return ESP_OK;
}

size_t alarm_get_all(alarm_entry_t *out, size_t max)
{
    size_t n = (max < ALARM_MAX_COUNT) ? max : ALARM_MAX_COUNT;
    memcpy(out, s_alarms, n * sizeof(alarm_entry_t));
    return n;
}

int alarm_get_ringing_index(void)
{
    return s_alarm_ringing_idx;
}

esp_err_t alarm_dismiss(void)
{
    s_alarm_snoozing = false;
    s_alarm_ringing = false;
    return ESP_OK;
}

esp_err_t alarm_snooze(void)
{
    s_alarm_snoozing = true;
    s_alarm_ringing = false;
    return ESP_OK;
}

/* ---- cd_timer / active countdown on the alarms/timers list screen ---- */

static bool s_timer_active;
static uint32_t s_timer_remaining_s;

esp_err_t cdtimer_start(uint32_t seconds)
{
    s_timer_active = true;
    s_timer_remaining_s = seconds;
    return ESP_OK;
}

void cdtimer_cancel(void)
{
    s_timer_active = false;
    s_timer_remaining_s = 0;
}

bool cdtimer_is_active(void)
{
    return s_timer_active;
}

uint32_t cdtimer_remaining_seconds(void)
{
    if (!s_timer_active) {
        return 0;
    }
    /* Counts down for real, same 1 Hz cadence as the real cd_timer.c, so
     * the list screen's countdown display can be eyeballed - driven from
     * watch_face.c's existing 1 s timer via cdtimer_check() below rather
     * than a mock-only ticker, so the countdown only advances while a
     * screen with a periodic refresh is actually showing it (matches the
     * firmware, which only ticks via the watch face's timer too). */
    return s_timer_remaining_s;
}

void cdtimer_check(void)
{
    if (s_timer_active && s_timer_remaining_s > 0) {
        s_timer_remaining_s--;
        if (s_timer_remaining_s == 0) {
            s_timer_active = false;
        }
    }
}

/* ---- SD / BLE status bar sources ----
 * sd_log_available() cycles so the SD icon's red/orange/grey states are all
 * visible without needing a real card; the other two are static-true, since
 * neither has a meaningful sim analogue (no SD socket, no BLE stack here). */
bool sd_log_available(void)
{
    return ((long)time(NULL) / 10) % 2 == 0;
}

bool twatch_sd_card_seated(void)
{
    return true;
}

bool ble_debug_is_connected(void)
{
    return ((long)time(NULL) / 6) % 2 == 0;
}

static bool s_ble_advertising = true;
bool ble_debug_is_advertising(void) { return s_ble_advertising; }
void ble_debug_set_advertising(bool on) { s_ble_advertising = on; }

esp_err_t sd_log_get_space(uint64_t *total_bytes, uint64_t *free_bytes)
{
    if (total_bytes) { *total_bytes = 8ULL * 1024 * 1024 * 1024; }
    if (free_bytes) { *free_bytes = 3ULL * 1024 * 1024 * 1024; }
    return ESP_OK;
}
