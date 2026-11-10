/*
 * debug/debug_gnss.c - GNSS/tracking debug console commands.
 *
 * Moved out of uwatch_main.c's debug_process_cmd() dispatch (mechanical
 * refactor, no behavior change).
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "m10q.h"
#include "lvgl_app.h"
#include "sensor_cache.h"
#include "tracking.h"
#include "bhi260ap.h"
#include "debug_gnss.h"

void debug_gnss_on(const char *args)
{
    (void)args;
    lvgl_gps_set_enabled(true);
    printf("gnss: powered on\n");
}

void debug_gnss_off(const char *args)
{
    (void)args;
    lvgl_gps_set_enabled(false);
    printf("gnss: powered off\n");
}

void debug_gnss_ver(const char *args)
{
    (void)args;
    /* Dump UBX-MON-VER to identify the module (genuine u-blox vs clone). */
    uGnssVersionType_t ver;
    memset(&ver, 0, sizeof(ver));
    if (m10q_get_versions(&ver) == ESP_OK) {
        printf("gnssver: sw=%s\n", ver.ver);
        printf("gnssver: hw=%s\n", ver.hw);
        printf("gnssver: mod=%s\n", ver.mod);
        printf("gnssver: fw=%s\n", ver.fw);
        printf("gnssver: prot=%s\n", ver.prot);
    } else {
        printf("gnssver: no cached version; enable GPS first\n");
    }
}

void debug_gnss_rtccal(const char *args)
{
    (void)args;
    /* Re-run the PPS-based RTC drift calibration (needs a GNSS fix). */
    m10q_rtc_calibrate();
    printf("rtccal: triggered (PPS window ~120 s, needs fix)\n");
}

void debug_gnss_status(const char *args)
{
    (void)args;
    /* Dump GNSS state, fix, and satellites. */
    m10q_fix_t fix;
    m10q_get_fix(&fix);
    printf("gnss: state=%d valid=%d gsv=%lu\n",
           (int)m10q_get_state(), (int)fix.valid,
           (unsigned long)m10q_get_gsv_count());
    if (fix.valid) {
        printf("gnss: pos %.5f %.5f alt %.0fm\n", fix.lat, fix.lon, fix.alt_m);
        printf("gnss: speed %u km/h course %u sats %u hdop %.1f hAcc %um\n",
               (unsigned)fix.speed_kmh, (unsigned)fix.course_deg,
               (unsigned)fix.sat_count, fix.hdop / 10.0,
               (unsigned)fix.hacc_m);
    }
    printf("gnss: in view %u\n", (unsigned)fix.sat_in_view);
    m10q_nav_status_t ns;
    if (m10q_get_nav_status(&ns) == ESP_OK && ns.updated) {
        printf("gnss: nav fix=%u fixOk=%d wknsSet=%d towSet=%d ttff=%lu ms\n",
               (unsigned)ns.gps_fix, (int)ns.gps_fix_ok, (int)ns.wkns_set,
               (int)ns.tow_set, (unsigned long)ns.ttff_ms);
    }
    for (int i = 0; i < (int)fix.sat_in_view && i < M10Q_MAX_SATS; i++) {
        printf("gnss: sat %2u el %3d az %3d snr %d used %d\n",
               (unsigned)fix.sats[i].prn, (int)fix.sats[i].elevation_deg,
               (int)fix.sats[i].azimuth_deg, (int)fix.sats[i].snr_db,
               (int)fix.sats[i].used);
    }
    m10q_stats_t st;
    if (m10q_get_stats(&st) == ESP_OK) {
        printf("gnss: fixes=%lu today=%lu ttf_avg=%lu ms best=%lu ms\n",
               (unsigned long)st.total_fixes, (unsigned long)st.fixes_today,
               (unsigned long)st.ttf_avg_ms, (unsigned long)st.ttf_best_ms);
    }
    printf("gnss: AGC unavailable (MIA-M10Q does not support MON-RF)\n");
}

void debug_gnss_check(const char *args)
{
    (void)args;
    /* Start a fresh background acquisition window. */
    printf("gpscheck: queued (signal-aware GPS auto-off)\n");
    lvgl_gps_refresh();
}

void debug_gnss_seed(const char *args)
{
    /* Seed the receiver with an approximate position to speed acquisition.
     * Usage: "gnssseed <lat> <lon>". */
    char *end, *tail;
    double lat = strtod(args, &end);
    double lon = strtod(end, &tail);
    bool parsed = end != args && tail != end;
    while (isspace((unsigned char)*tail)) tail++;
    if (parsed && !*tail && isfinite(lat) && isfinite(lon) && fabs(lat) <= 90 && fabs(lon) <= 180) {
        lvgl_gps_seed_position(lat, lon);
        printf("gnssseed: queued %.5f, %.5f (requires GPS on)\n", lat, lon);
    } else {
        printf("gnssseed: usage gnssseed <lat> <lon>\n");
    }
}

void debug_gnss_lpk(const char *args)
{
    (void)args;
    double lat = 0, lon = 0;
    m10q_get_last_position(&lat, &lon);
    if (lat == 0 && lon == 0) {
        printf("lpk: none stored yet\n");
    } else {
        printf("lpk: %.5f, %.5f\n", lat, lon);
    }
}

void debug_gnss_raw(const char *args)
{
    (void)args;
    puts("gnssraw: unavailable; ubxlib owns UART RX");
}

void debug_gnss_probe(const char *args)
{
    (void)args;
    int8_t r = bhi260ap_gnss_inject_probe();
    printf("gnssprobe: inject_driver=%s rslt=%d\n",
           (r == 0) ? "active" : "not active / no GPS request", (int)r);
}

void debug_gnss_track(const char *args)
{
    if (args[0] == '\0') {
#if TRACKING_ENABLED
        lvgl_tracking_start();
        printf("track: started (GNSS pulses every 50 steps)\n");
#else
        printf("track: disabled (TRACKING_ENABLED=0)\n");
#endif
    } else if (strcmp(args, "stop") == 0) {
        lvgl_tracking_stop();
        printf("track: stopped\n");
    } else {
        printf("unknown command: track %s\n", args);
    }
}

void debug_gnss_cachedump(const char *args)
{
    (void)args;
    sensor_cache_t c;
    sensor_cache_get(&c);
    printf("cache: valid=%d rtc=%04u-%02u-%02u %02u:%02u:%02u wd=%u\n",
           (int)c.valid, (unsigned)c.rtc.year, (unsigned)c.rtc.month,
           (unsigned)c.rtc.day, (unsigned)c.rtc.hour, (unsigned)c.rtc.min,
           (unsigned)c.rtc.sec, (unsigned)c.rtc.weekday);
    printf("cache: batt=%u%% %umV chg=%d en=%d ma=%u pmu_temp=%d.%dC\n",
           (unsigned)c.batt_pct, (unsigned)c.batt_mv, (int)c.chg_state,
           (int)c.chg_enabled, (unsigned)c.chg_ma,
           c.pmu_die_temp_c10 / 10, abs(c.pmu_die_temp_c10 % 10));
    battery_estimate_t e;
    battery_estimate_get(&e);
    printf("batt est: valid=%d pct_per_hour=%.2f runtime_h=%.1f charge_h=%.1f\n",
           (int)e.estimate_valid, (double)e.pct_per_hour,
           (double)e.runtime_h, (double)e.charge_h);
}

void debug_gnss_trackstat(const char *args)
{
    (void)args;
    tracking_totals_t t;
    tracking_get_totals(&t);
    printf("track: active=%d dist=%.2f km steps=%lu avg=%.2f m session_steps=%lu\n",
           (int)tracking_is_active(),
           t.dist_cm / 100000.0, (unsigned long)t.steps,
           tracking_get_avg_step_cm() / 100.0,
           (unsigned long)tracking_get_session_steps());
}
