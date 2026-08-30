/*
 * watch_face.c - the watch face screen: builder + 1 Hz update.
 *
 * This is a genuinely SHARED file, compiled by both the ESP-IDF firmware
 * build (main/CMakeLists.txt) and the host sim build (sim/CMakeLists.txt)
 * from this one copy - not a hand-ported duplicate. That's possible
 * because this file only touches LVGL, cascadia_fonts.h, and driver-facing
 * headers that sim/mock_hw.h already mirrors 1:1 (sensor_cache.h, m10q.h,
 * pcf85063a.h, bhi260ap.h, tracking.h, alarm.h, cd_timer.h,
 * power_mgmt.h) - see sim/mock_hw.h's own header comment. Do not add an
 * include here that isn't either portable or covered by mock_hw.h; if a
 * new one is genuinely needed on both sides, mirror it into mock_hw.h
 * first.
 *
 * menu_timeout_cb()'s 500ms timer is deliberately NOT started here: it's
 * nav-ring infrastructure shared across every screen, not watch-face
 * logic, and doesn't exist in the sim at all (sim/main.c has no
 * menu-timeout concept yet). main/lvgl_app.c's boot_to_watch_face()
 * starts it separately, right after calling lvgl_build_watch_face().
 */
#include "screens.h"
#include "status_bar.h"
#include <stdio.h>
#include <time.h>
#include "cascadia_fonts.h"
#include "sensor_cache.h"
#include "pcf85063a.h"
#include "bhi260ap.h"
#include "tracking.h"
#include "power_mgmt.h"
#include "alarm.h"
#include "cd_timer.h"

static const lv_font_t *s_font_time = &cascadia_72;   /* HH:MM:SS */
static const lv_font_t *s_font_sec  = &cascadia_36;   /* UTC time */
static const lv_font_t *s_font_small = &cascadia_22;  /* body text */

lv_obj_t *s_watch_screen;

static status_bar_t s_status_bar;
static lv_obj_t *s_tz_label;
static lv_obj_t *s_time_label;
static lv_obj_t *s_sec_label;
static lv_obj_t *s_date_label;
static lv_obj_t *s_steps_label;
static lv_obj_t *s_track_dot;
static lv_obj_t *s_snooze_icon;

void watch_face_update(lv_timer_t *timer)
{
    (void)timer;

    /* Fire the alarm when the RTC AF/TF flag is set (also covers a wake from
     * light sleep via the RTC INT line). */
    alarm_check();
    cdtimer_check();

    /* Read the wall-clock time from the RTC (PCF85063A), not the ESP32 system
     * clock, so the display never drifts. The RTC is polled by the background
     * telemetry task; reading the cache keeps I2C off the UI task. */
    pcf85063a_time_t t;
    if (!sensor_cache_get_rtc(&t)) {
        return;
    }

    /* The RTC stores UTC directly; convert to local (process TZ) for the
     * primary display, which stays DST-aware. */
    time_t epoch = pcf85063a_time_to_epoch(&t);
    struct tm lt;
    localtime_r(&epoch, &lt);

    char buf[32];
    if (s_tz_label) {
        char tz[8] = { 0 };
        strftime(tz, sizeof(tz), "%Z", &lt);
        lv_label_set_text(s_tz_label, tz);
    }

    bool sparmodus = power_mgmt_get_sparmodus_active();

    /* Ultra-Sparmodus (docs/application.md section 10.3): only the time,
     * no seconds, everything else hidden - the red/dim rendering itself is
     * applied at blit time by night_mode_draw_bitmap() in lvgl_app.c, not
     * here. Widgets are hidden/shown every tick (not just on the edge that
     * toggles Sparmodus) so leaving the mode self-corrects on the next
     * tick with no separate restore path. */
    if (sparmodus) {
        snprintf(buf, sizeof(buf), "%02d:%02d", lt.tm_hour, lt.tm_min);
        lv_label_set_text(s_time_label, buf);
        if (s_tz_label) { lv_obj_add_flag(s_tz_label, LV_OBJ_FLAG_HIDDEN); }
        if (s_sec_label) { lv_obj_add_flag(s_sec_label, LV_OBJ_FLAG_HIDDEN); }
        if (s_date_label) { lv_obj_add_flag(s_date_label, LV_OBJ_FLAG_HIDDEN); }
        if (s_steps_label) { lv_obj_add_flag(s_steps_label, LV_OBJ_FLAG_HIDDEN); }
        if (s_track_dot) { lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN); }
        if (s_snooze_icon) { lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN); }
        status_bar_set_hidden(&s_status_bar, true);
        return;
    }

    if (s_tz_label) { lv_obj_clear_flag(s_tz_label, LV_OBJ_FLAG_HIDDEN); }
    if (s_sec_label) { lv_obj_clear_flag(s_sec_label, LV_OBJ_FLAG_HIDDEN); }
    if (s_date_label) { lv_obj_clear_flag(s_date_label, LV_OBJ_FLAG_HIDDEN); }
    status_bar_set_hidden(&s_status_bar, false);

    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
    lv_label_set_text(s_time_label, buf);

    /* UTC sub-display: the RTC snapshot is already UTC, no conversion. */
    snprintf(buf, sizeof(buf), "UTC %02d:%02d", t.hour, t.min);
    lv_label_set_text(s_sec_label, buf);

    static const char *wday[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };
    static const char *mon[] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                 "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };
    snprintf(buf, sizeof(buf), "%s  %02d %s %d",
             wday[lt.tm_wday], lt.tm_mday, mon[lt.tm_mon], lt.tm_year + 1900);
    lv_label_set_text(s_date_label, buf);

    /* Step count from the BHI260AP (cached in the driver, no I2C here). */
    if (s_steps_label) {
        uint32_t steps = 0;
        if (bhi260ap_get_daily_steps(&steps) == ESP_OK) {
            snprintf(buf, sizeof(buf), "Steps: %lu", (unsigned long)steps);
        } else {
            snprintf(buf, sizeof(buf), "Steps: --");
        }
        lv_label_set_text(s_steps_label, buf);
    }

    /* Tracking dot: visible only while a tracking session is active. */
    if (s_track_dot) {
        if (tracking_is_active()) {
            lv_obj_clear_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* Snooze icon: visible while the 10 min snooze timer is pending. */
    if (s_snooze_icon) {
        if (alarm_is_snoozing()) {
            lv_obj_clear_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }

    update_status_bar(&s_status_bar);
}

void lvgl_build_watch_face(void)
{
    s_watch_screen = lv_screen_active();
    lv_obj_set_style_bg_color(s_watch_screen, lv_color_hex(0x000000), 0);

    build_status_bar_big(s_watch_screen, &s_status_bar);

    s_date_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_date_label, "");
    lv_obj_set_style_text_font(s_date_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_date_label, LV_ALIGN_TOP_MID, 0, 420);

    /* Tracking indicator: solid red dot, visible only while a tracking
     * session is active. The satellite fix icon used to sit here too, but
     * it duplicated the status bar's own GPS icon (both color-code fix
     * state independently) - removed once that icon got bigger/more
     * prominent in the two-row status bar. */
    s_track_dot = lv_obj_create(lv_screen_active());
    lv_obj_set_size(s_track_dot, 12, 12);
    lv_obj_align(s_track_dot, LV_ALIGN_TOP_MID, 0, 24);
    lv_obj_clear_flag(s_track_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_track_dot, lv_color_hex(0xFF2020), 0);
    lv_obj_set_style_radius(s_track_dot, 6, 0);
    lv_obj_set_style_pad_all(s_track_dot, 0, 0);
    lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);

    /* Snooze indicator: "Zz" over the alarm icon, hidden unless snoozing. */
    s_snooze_icon = lv_label_create(lv_screen_active());
    lv_label_set_text(s_snooze_icon, "Zz");
    lv_obj_set_style_text_font(s_snooze_icon, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_snooze_icon, lv_color_hex(0xFFD54F), 0);
    lv_obj_align(s_snooze_icon, LV_ALIGN_TOP_LEFT, 70, 40);
    lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);

    /* Timezone abbreviation (e.g. "CEST"/"CET") above the local time - the
     * process TZ is already set at boot (main/uwatch_main.c) and
     * watch_face_update() already computes localtime_r() for the primary
     * display, so strftime("%Z", ...) gives this for free, no new TZ
     * plumbing needed. Positioned with a clear gap above the big time font
     * below it (cascadia_72's glyphs reach well above s_time_label's own
     * y) - this used to overlap before the gap was widened. */
    s_tz_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_tz_label, "");
    lv_obj_set_style_text_font(s_tz_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_tz_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_tz_label, LV_ALIGN_TOP_MID, 0, 140);

    s_time_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_time_label, "--:--:--");
    lv_obj_set_style_text_font(s_time_label, s_font_time, 0);
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), 0);
    /* Tighten the monospace cells so the full-width colons don't sprawl. */
    lv_obj_set_style_text_letter_space(s_time_label, -6, 0);
    lv_obj_align(s_time_label, LV_ALIGN_TOP_MID, 0, 175);

    s_sec_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_sec_label, "UTC --:--");
    lv_obj_set_style_text_font(s_sec_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_sec_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_sec_label, LV_ALIGN_TOP_MID, 0, 275);

    s_steps_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_steps_label, "Steps: --");
    lv_obj_set_style_text_font(s_steps_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_steps_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_steps_label, LV_ALIGN_TOP_MID, 0, 325);

    watch_face_update(NULL);
    lv_timer_create(watch_face_update, 1000, NULL);
}
