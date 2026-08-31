/*
 * gps_screen.c - GPS screen (skyplot + fix info). Shared between the
 * firmware and the host sim - see watch_face.c's header comment for the
 * mechanism (real header names, intercepted by stubs under sim/compat/).
 *
 * GNSS is powered on demand: the BLDO1 rail is enabled when this screen is
 * opened and disabled when it is left. The always-on VRTC backup rail keeps
 * the receiver's ephemeris/RTC alive, so each power-up is a warm/hot start.
 *
 * The background power-transition worker (gps_ctrl_task, a real FreeRTOS
 * task) stays firmware-only in main/lvgl_app.c - it has no sim equivalent
 * (the sim's GNSS mock has no power-on latency to simulate). Both it and
 * this file's gps_pwr_switch_cb() go through gps_screen_set_powered()/
 * gps_screen_is_powered() below instead of touching s_gps_powered directly,
 * since ownership of that flag is now split across two files (and, via
 * lvgl_app.c's settings_periph_refresh(), a third screen that mirrors the
 * same on/off state).
 */
#include "screens.h"
#include <stdio.h>
#include <math.h>
#include "cascadia_fonts.h"
#include "m10q.h"
#include "gpx_log.h"
#include "lvgl_app.h"
#include "esp_lv_adapter.h"
#include "portable_log.h"

static const lv_font_t *s_font_sec  = &cascadia_36;
static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_micro = &cascadia_18;

#define GPS_SKY_RADIUS    100
#define GPS_SKY_CX        205
#define GPS_SKY_CY        190

lv_obj_t *s_gps_screen;

static lv_obj_t *s_gps_status_label;
static lv_obj_t *s_gps_pos_label;
static lv_obj_t *s_gps_speed_label;
static lv_obj_t *s_gps_sats_label;
static lv_obj_t *s_gps_diag_label;           /* GNSS diagnostics (state/offset/ttff/rx) */
static lv_obj_t *s_gps_dots[M10Q_MAX_SATS];   /* satellite dots (in view order) */
static lv_obj_t *s_gps_track_label;            /* tracking stats (distance/steps/avg) */
static lv_obj_t *s_gps_track_btn;              /* Start/Stop tracking button */
static lv_obj_t *s_gps_pwr_switch;             /* GNSS on/off switch */

/* Owns the "is GNSS powered" flag and its power-on timestamp - see the
 * file header comment for why this needs a getter/setter instead of a
 * plain file-static now that gps_ctrl_task (main/lvgl_app.c) and the
 * Settings/Peripherie screen (also lvgl_app.c) both touch it. */
static volatile bool s_gps_powered;
static uint32_t s_gps_acq_start_ms;            /* power-on timestamp */

bool gps_screen_is_powered(void)
{
    return s_gps_powered;
}

void gps_screen_set_powered(bool on)
{
    s_gps_powered = on;
    if (on) {
        s_gps_acq_start_ms = lv_tick_get();
    }
    ESP_LOGI("gps_screen", "set_powered(%d) acq_start_ms=%lu", (int)on,
             (unsigned long)s_gps_acq_start_ms);
}

static lv_obj_t *gps_ring(int radius)
{
    lv_obj_t *arc = lv_arc_create(s_gps_screen);
    lv_obj_set_size(arc, radius * 2, radius * 2);
    lv_obj_set_pos(arc, GPS_SKY_CX - radius, GPS_SKY_CY - radius);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_rotation(arc, 0);
    lv_arc_set_value(arc, 100);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x2A5A2A), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x2A5A2A), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_INDICATOR);
    /* Hide the arc knob: with a full 0-360 range and value 100, LVGL draws the
     * default (blue) knob at the 360 deg point = 3 o'clock. Without this,
     * each ring contributes a blue dot in a horizontal line on the east. */
    lv_obj_set_style_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    return arc;
}

/* Convert satellite azimuth/elevation to skyplot pixel coords. */
static void gps_sat_xy(int az, int el, int *x, int *y)
{
    double r = (double)GPS_SKY_RADIUS * (90.0 - el) / 90.0;
    double a = (double)az * M_PI / 180.0;
    *x = GPS_SKY_CX + (int)(r * sin(a) + 0.5);
    *y = GPS_SKY_CY - (int)(r * cos(a) + 0.5);
}

static lv_color_t gps_snr_color(int snr)
{
    if (snr < 25) return lv_color_hex(0x888888);
    if (snr < 35) return lv_color_hex(0xFFD54D);
    return lv_color_hex(0x3DD68A);
}

/* Decimal degrees -> DMS. The doc's example uses "48°07'24"N", but the
 * baked bitmap fonts here (cascadia_*.c) only cover ASCII 0x20-0x7E - no
 * degree sign (U+00B0) - and the source TTF isn't in the repo to
 * regenerate them with a wider range (see cascadia_22.c's header comment
 * for the original `lv_font_conv` invocation). Using the plain-ASCII
 * "D M S" letter notation instead (e.g. "48d07m24sN"), a common fallback
 * for the same reason on other text-only GPS displays. */
static void format_dms(double deg, bool is_lat, char *out, size_t outlen)
{
    char dir = is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
    deg = fabs(deg);
    int d = (int)deg;
    double m_full = (deg - d) * 60.0;
    int m = (int)m_full;
    int s = (int)((m_full - m) * 60.0 + 0.5);
    snprintf(out, outlen, "%dd%02dm%02ds%c", d, m, s, dir);
}

void gps_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_gps_status_label) {
        return;
    }
    /* Skip all work (fix reads + widget updates) when the GPS screen is not
     * the active screen. The timer fires every second forever. */
    if (lv_screen_active() != s_gps_screen) {
        return;
    }
    m10q_fix_t fix;
    m10q_get_fix(&fix);
    m10q_state_t st = m10q_get_state();
    char buf[96];

    /* While acquiring, keep the watch awake (no auto-sleep) so the GNSS rail
     * stays powered. Once a fix is obtained, stop reporting activity: the
     * adapter's idle timeout then auto-sleeps the watch ~5 s after the fix,
     * powering BLDO1 off (VRTC backup keeps ephemeris for the next session). */
    if (st != M10Q_STATE_FIXED || !fix.valid) {
        esp_lv_adapter_report_activity();
    }
    if (st == M10Q_STATE_FIXED && fix.valid) {
        snprintf(buf, sizeof(buf), "Fix: %d sats  acc %um  HDOP %.1f",
                 (int)fix.sat_count, (unsigned)fix.hacc_m, fix.hdop / 10.0);
        lv_label_set_text(s_gps_status_label, buf);

        char dms_lat[16], dms_lon[16];
        format_dms(fix.lat, true, dms_lat, sizeof(dms_lat));
        format_dms(fix.lon, false, dms_lon, sizeof(dms_lon));
        snprintf(buf, sizeof(buf), "%.5f, %.5f  %.0f m\n%s  %s",
                 fix.lat, fix.lon, fix.alt_m, dms_lat, dms_lon);
        lv_label_set_text(s_gps_pos_label, buf);

        snprintf(buf, sizeof(buf), "Speed: %u km/h  course %u deg",
                 (unsigned)fix.speed_kmh, (unsigned)fix.course_deg);
        lv_label_set_text(s_gps_speed_label, buf);

        snprintf(buf, sizeof(buf), "%02u:%02u:%02u UTC  (%u in view)",
                 (unsigned)fix.hour, (unsigned)fix.minute, (unsigned)fix.second,
                 (unsigned)fix.sat_in_view);
        lv_label_set_text(s_gps_sats_label, buf);
    } else if (st == M10Q_STATE_ACQUIRING) {
        uint32_t now = lv_tick_get();
        uint32_t elapsed = (now - s_gps_acq_start_ms) / 1000;
        snprintf(buf, sizeof(buf), "Acquiring... (%lus)", (unsigned long)elapsed);
        lv_label_set_text(s_gps_status_label, buf);
        lv_label_set_text(s_gps_pos_label, "");
        lv_label_set_text(s_gps_speed_label, "");
        lv_label_set_text(s_gps_sats_label, "");
    } else {
        lv_label_set_text(s_gps_status_label, "GNSS off");
        lv_label_set_text(s_gps_pos_label, "");
        lv_label_set_text(s_gps_speed_label, "");
        lv_label_set_text(s_gps_sats_label, "");
    }

    /* Diagnostics line: receiver state, RX traffic, TTFF. */
    if (s_gps_diag_label) {
        uint32_t rx = 0, lines = 0;
        m10q_get_dbg(&rx, &lines);
        (void)lines;
        m10q_stats_t stats;
        if (m10q_get_stats(&stats) == ESP_OK) {
            snprintf(buf, sizeof(buf),
                     "st=%d rx=%lu gsv=%lu fix=%lu ttf=%lums",
                     (int)m10q_get_state(), (unsigned long)rx,
                     (unsigned long)m10q_get_gsv_count(),
                     (unsigned long)stats.total_fixes,
                     (unsigned long)stats.ttf_avg_ms);
        } else {
            snprintf(buf, sizeof(buf),
                     "st=%d rx=%lu gsv=%lu",
                     (int)m10q_get_state(), (unsigned long)rx,
                     (unsigned long)m10q_get_gsv_count());
        }
        lv_label_set_text(s_gps_diag_label, buf);
    }

    /* Satellite dots. Satellites with no elevation/azimuth (receiver not yet
     * resolved) or no SNR are hidden; otherwise they'd cluster at the skyplot
     * centre as a static blob. */
    for (int i = 0; i < M10Q_MAX_SATS; i++) {
        if (!s_gps_dots[i]) {
            break;
        }
        if (i < (int)fix.sat_in_view && fix.sats[i].snr_db >= 0 &&
                (fix.sats[i].elevation_deg > 0 || fix.sats[i].azimuth_deg > 0)) {
            m10q_sat_t *s = &fix.sats[i];
            int x, y;
            gps_sat_xy(s->azimuth_deg, s->elevation_deg, &x, &y);
            lv_obj_set_pos(s_gps_dots[i], x - 7, y - 7);
            lv_obj_set_style_bg_color(s_gps_dots[i], gps_snr_color(s->snr_db), 0);
            lv_obj_set_style_border_width(s_gps_dots[i], s->used ? 0 : 2, 0);
            lv_obj_set_style_border_color(s_gps_dots[i], lv_color_hex(0xAAAAAA), 0);
            lv_obj_clear_flag(s_gps_dots[i], LV_OBJ_FLAG_HIDDEN);
            char prn[8];
            snprintf(prn, sizeof(prn), "%u", (unsigned)s->prn);
            lv_label_set_text_fmt(lv_obj_get_child(s_gps_dots[i], 0), "%s", prn);
        } else {
            lv_obj_add_flag(s_gps_dots[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* GNSS on/off switch state. */
    if (s_gps_pwr_switch) {
        bool powered = s_gps_powered && m10q_get_state() != M10Q_STATE_OFF;
        if (lv_obj_has_state(s_gps_pwr_switch, LV_STATE_CHECKED) != powered) {
            if (powered) {
                lv_obj_add_state(s_gps_pwr_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(s_gps_pwr_switch, LV_STATE_CHECKED);
            }
        }
    }

    /* GPX recording state + Start/Stop button (main/gpx_log.c) - a
     * time-based (every 30s) SD file logger, unrelated to tracking.c's
     * step-gated pedometer (that stays disabled, see TRACKING_ENABLED). */
    if (s_gps_track_label && s_gps_track_btn) {
        bool active = gpx_log_is_active();
        if (active) {
            snprintf(buf, sizeof(buf), "Recording: %lu pts",
                     (unsigned long)gpx_log_point_count());
        } else {
            snprintf(buf, sizeof(buf), "GPX: off");
        }
        lv_label_set_text(s_gps_track_label, buf);
        lv_obj_t *bl = lv_obj_get_child(s_gps_track_btn, 0);
        if (bl) {
            lv_label_set_text(bl, active ? "Stop" : "Start");
        }
        lv_obj_set_style_bg_color(s_gps_track_btn,
                                  active ? lv_color_hex(0x8B0000) : lv_color_hex(0x1B5E20), 0);
    }
}

/* GNSS on/off switch on the GPS screen. Persists the choice so the next boot
 * powers GNSS on only if it was left enabled. lvgl_gps_set_enabled() is the
 * single entry point for the actual transition: on the firmware it queues a
 * request onto the async gps_ctrl_task, which alone calls
 * gps_screen_set_powered() once it has actually acted on it; on the sim
 * (no task) the mock_gnss_set_enabled() it calls into does the equivalent
 * synchronously instead. Do NOT also call gps_screen_set_powered() here -
 * doing so previously raced the real task: this callback would flip
 * gps_screen_is_powered() before the task read it, so the task's own
 * "only transition if the state disagrees" guard saw no disagreement and
 * silently skipped the real m10q_power() call - the switch changed how it
 * looked without changing what GNSS was doing. */
static void gps_pwr_switch_cb(lv_event_t *e)
{
    (void)e;
    if (!s_gps_pwr_switch) {
        return;
    }
    bool on = lv_obj_has_state(s_gps_pwr_switch, LV_STATE_CHECKED);
    ESP_LOGI("gps_screen", "gps_pwr_switch_cb: on=%d", (int)on);
    lvgl_gps_set_enabled(on);
}

/* GPS screen Start/Stop tracking button. */
static void gps_track_btn_cb(lv_event_t *e)
{
    (void)e;
    if (gpx_log_is_active()) {
        gpx_log_stop();
    } else {
        gpx_log_start();
    }
    gps_screen_update(NULL);
}

void lvgl_build_gps_screen(void)
{
    s_gps_screen = screen_new();
    lv_obj_set_style_bg_color(s_gps_screen, lv_color_hex(0x102010), 0);

    lv_obj_t *title = lv_label_create(s_gps_screen);
    lv_label_set_text(title, "GPS");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    /* GNSS on/off switch. Explicit larger size (matches the Settings
     * screens' switches, AGENT.md "Display density & UI sizing") - this
     * screen's skyplot + stacked telemetry rows already use the full
     * panel height with no slack, so unlike Settings this pass is limited
     * to touch targets and the few short standalone labels (title, this
     * one); the info-row stack and the diagnostics line stay at their
     * current sizes - promoting them would need reflowing/shrinking the
     * skyplot, a bigger change than a sizing pass. */
    lv_obj_t *pwr_lbl = lv_label_create(s_gps_screen);
    lv_label_set_text(pwr_lbl, "GNSS");
    lv_obj_set_style_text_font(pwr_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(pwr_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(pwr_lbl, LV_ALIGN_TOP_LEFT, 90, 22);
    s_gps_pwr_switch = lv_switch_create(s_gps_screen);
    lv_obj_set_size(s_gps_pwr_switch, 66, 36);
    lv_obj_align(s_gps_pwr_switch, LV_ALIGN_TOP_RIGHT, -90, 14);
    lv_obj_add_event_cb(s_gps_pwr_switch, gps_pwr_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Skyplot: horizon ring + elevation rings. */
    gps_ring(GPS_SKY_RADIUS);
    gps_ring(GPS_SKY_RADIUS * 2 / 3);
    gps_ring(GPS_SKY_RADIUS / 3);

    /* Cardinal labels. */
    lv_obj_t *n = lv_label_create(s_gps_screen);
    lv_label_set_text(n, "N");
    lv_obj_set_style_text_font(n, s_font_small, 0);
    lv_obj_set_style_text_color(n, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(n, GPS_SKY_CX - 6, GPS_SKY_CY - GPS_SKY_RADIUS - 18);
    lv_obj_t *e = lv_label_create(s_gps_screen);
    lv_label_set_text(e, "E");
    lv_obj_set_style_text_font(e, s_font_small, 0);
    lv_obj_set_style_text_color(e, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(e, GPS_SKY_CX + GPS_SKY_RADIUS - 4, GPS_SKY_CY - 14);
    lv_obj_t *s = lv_label_create(s_gps_screen);
    lv_label_set_text(s, "S");
    lv_obj_set_style_text_font(s, s_font_small, 0);
    lv_obj_set_style_text_color(s, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(s, GPS_SKY_CX - 6, GPS_SKY_CY + GPS_SKY_RADIUS + 2);
    lv_obj_t *w = lv_label_create(s_gps_screen);
    lv_label_set_text(w, "W");
    lv_obj_set_style_text_font(w, s_font_small, 0);
    lv_obj_set_style_text_color(w, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(w, GPS_SKY_CX - GPS_SKY_RADIUS - 12, GPS_SKY_CY - 14);

    /* Satellite dots: 14 px circles with PRN label inside. */
    for (int i = 0; i < M10Q_MAX_SATS; i++) {
        lv_obj_t *dot = lv_obj_create(s_gps_screen);
        lv_obj_set_size(dot, 14, 14);
        lv_obj_set_style_radius(dot, 7, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(0x888888), 0);
        lv_obj_set_style_border_width(dot, 2, 0);
        lv_obj_set_style_border_color(dot, lv_color_hex(0xAAAAAA), 0);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t *prn = lv_label_create(dot);
        lv_label_set_text(prn, "");
        lv_obj_set_style_text_font(prn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(prn, lv_color_hex(0x000000), 0);
        lv_obj_center(prn);
        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
        s_gps_dots[i] = dot;
    }

    /* Info rows below the skyplot. */
    s_gps_status_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_status_label, "");
    lv_obj_set_style_text_font(s_gps_status_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_status_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_gps_status_label, LV_ALIGN_TOP_MID, 0, 306);

    s_gps_pos_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_pos_label, "");
    lv_obj_set_style_text_font(s_gps_pos_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_pos_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_gps_pos_label, LV_ALIGN_TOP_MID, 0, 330);

    s_gps_speed_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_speed_label, "");
    lv_obj_set_style_text_font(s_gps_speed_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_speed_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_gps_speed_label, LV_ALIGN_TOP_MID, 0, 380);

    s_gps_sats_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_sats_label, "");
    lv_obj_set_style_text_font(s_gps_sats_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_sats_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_gps_sats_label, LV_ALIGN_TOP_MID, 0, 406);

    s_gps_diag_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_diag_label, "");
    lv_obj_set_style_text_font(s_gps_diag_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_gps_diag_label, lv_color_hex(0x8A9BA8), 0);
    lv_obj_align(s_gps_diag_label, LV_ALIGN_TOP_MID, 0, 56);

    /* GPX recording state + start/stop (main/gpx_log.c). */
    s_gps_track_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_track_label, "");
    lv_obj_set_style_text_font(s_gps_track_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_track_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_gps_track_label, LV_ALIGN_TOP_MID, 0, 404);

    /* Bigger touch target (was 120x34/cascadia_22 - too small next to every
     * other button this app's high-DPI pass already enlarged). */
    s_gps_track_btn = lv_btn_create(s_gps_screen);
    lv_obj_set_size(s_gps_track_btn, 170, 50);
    lv_obj_align(s_gps_track_btn, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_add_event_cb(s_gps_track_btn, gps_track_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_lbl = lv_label_create(s_gps_track_btn);
    lv_label_set_text(btn_lbl, "Start");
    lv_obj_set_style_text_font(btn_lbl, s_font_sec, 0);
    lv_obj_center(btn_lbl);

    gps_screen_update(NULL);
    lv_timer_create(gps_screen_update, 1000, NULL);
    /* No task-spawn here (unlike the pre-migration original): gps_ctrl_task
     * is already unconditionally started at boot in lvgl_app.c's
     * lvgl_app_start(), well before any screen navigation can reach this
     * function, so the old "spawn if not already running" guard here was
     * dead code - removing it also drops the last ESP-only
     * (xTaskCreate/FreeRTOS) call from this file. */
}
