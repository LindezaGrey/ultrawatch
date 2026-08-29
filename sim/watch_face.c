/*
 * watch_face.c - the real watch-face screen logic from main/lvgl_app.c,
 * copied verbatim (lvgl_build_watch_face / watch_face_update, main branch
 * of both, unmodified) and run against mock_hw.c instead of real drivers.
 *
 * Omitted vs. the firmware original: the menu-timeout timer
 * (lv_timer_create(menu_timeout_cb, ...)) - that's screen-navigation
 * infrastructure shared across all six screens, not watch-face logic, and
 * out of scope for this scaffold.
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two, so a change to the real watch_face_update()/
 * lvgl_build_watch_face() won't automatically show up here.
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_time = &cascadia_72;
static const lv_font_t *s_font_sec  = &cascadia_36;
static const lv_font_t *s_font_small = &cascadia_22;

static lv_obj_t *s_time_label;
static lv_obj_t *s_sec_label;
static lv_obj_t *s_date_label;
static lv_obj_t *s_batt_label;
static lv_obj_t *s_batt_fill;
static lv_obj_t *s_steps_label;
static lv_obj_t *s_gps_icon;
static lv_obj_t *s_track_dot;
static lv_obj_t *s_snooze_icon;

/* Status bar row - verbatim port of main/lvgl_app.c's build_status_bar()/
 * update_status_bar()/status_icon_create(), watch-face instance only (the
 * other four ring screens don't exist in this scaffold yet - see the
 * top-of-file "keep in sync by hand" note). Positioning/colors/icon choices
 * must be kept identical to the firmware original by hand. */
#define STATUS_COLOR_GREY   lv_color_hex(0x888888)
#define STATUS_COLOR_ORANGE lv_color_hex(0xFFB300)
#define STATUS_COLOR_GREEN  lv_color_hex(0x00E676)
#define STATUS_COLOR_RED    lv_color_hex(0xFF5252)
#define STATUS_BAR_Y 54

typedef struct {
    lv_obj_t *sd;
    lv_obj_t *gps;
    lv_obj_t *gpx;
    lv_obj_t *lora;
    lv_obj_t *bt;
    lv_obj_t *wifi;
    lv_obj_t *batt;
    lv_obj_t *chg;
} status_bar_t;

static status_bar_t s_status_bar;

static lv_obj_t *status_icon_create(lv_obj_t *parent, const char *text, lv_align_t align, lv_coord_t x)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, STATUS_COLOR_GREY, 0);
    lv_obj_align(l, align, x, STATUS_BAR_Y);
    return l;
}

static void build_status_bar(lv_obj_t *parent, status_bar_t *out)
{
    out->sd   = status_icon_create(parent, LV_SYMBOL_SD_CARD,   LV_ALIGN_TOP_LEFT,  40);
    out->gps  = status_icon_create(parent, LV_SYMBOL_GPS,       LV_ALIGN_TOP_LEFT,  80);
    out->gpx  = status_icon_create(parent, "GPX",               LV_ALIGN_TOP_LEFT,  118);
    out->lora = status_icon_create(parent, "LoRa",              LV_ALIGN_TOP_LEFT,  166);
    out->bt   = status_icon_create(parent, LV_SYMBOL_BLUETOOTH, LV_ALIGN_TOP_LEFT,  214);
    out->wifi = status_icon_create(parent, LV_SYMBOL_WIFI,      LV_ALIGN_TOP_LEFT,  250);
    out->chg  = status_icon_create(parent, LV_SYMBOL_CHARGE,    LV_ALIGN_TOP_RIGHT, -115);
    out->batt = status_icon_create(parent, LV_SYMBOL_BATTERY_EMPTY " --%", LV_ALIGN_TOP_RIGHT, -40);
}

static void update_status_bar(const status_bar_t *bar)
{
    if (!bar->sd) {
        return;
    }

    lv_obj_set_style_text_color(bar->sd, sd_log_available() ? STATUS_COLOR_RED :
                                (twatch_sd_card_seated() ? STATUS_COLOR_ORANGE : STATUS_COLOR_GREY), 0);

    m10q_state_t gps_st = m10q_get_state();
    lv_obj_set_style_text_color(bar->gps,
        (gps_st == M10Q_STATE_FIXED) ? STATUS_COLOR_GREEN :
        (gps_st == M10Q_STATE_ACQUIRING) ? STATUS_COLOR_ORANGE : STATUS_COLOR_GREY, 0);

    lv_obj_set_style_text_color(bar->gpx, tracking_is_active() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);
    lv_obj_set_style_text_color(bar->lora, STATUS_COLOR_GREEN, 0);
    lv_obj_set_style_text_color(bar->bt, ble_debug_is_connected() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);
    lv_obj_set_style_text_color(bar->wifi, STATUS_COLOR_GREY, 0);

    sensor_cache_t cache;
    sensor_cache_get(&cache);
    if (cache.valid) {
        const char *icon = cache.batt_pct > 87 ? LV_SYMBOL_BATTERY_FULL :
                            cache.batt_pct > 62 ? LV_SYMBOL_BATTERY_3 :
                            cache.batt_pct > 37 ? LV_SYMBOL_BATTERY_2 :
                            cache.batt_pct > 12 ? LV_SYMBOL_BATTERY_1 : LV_SYMBOL_BATTERY_EMPTY;
        char buf[16];
        snprintf(buf, sizeof(buf), "%s %u%%", icon, cache.batt_pct);
        lv_label_set_text(bar->batt, buf);
        lv_obj_set_style_text_color(bar->batt, cache.batt_pct <= 15 ? STATUS_COLOR_RED : lv_color_hex(0xE0E0E0), 0);

        bool charging = (cache.chg_state != AXP2101_CHG_STOP);
        if (charging) {
            lv_obj_clear_flag(bar->chg, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_text_color(bar->chg, STATUS_COLOR_GREEN, 0);
        } else {
            lv_obj_add_flag(bar->chg, LV_OBJ_FLAG_HIDDEN);
        }
    }
}
/* Not static: declared extern in screens.h so nav.c's swipe/menu-timeout
 * code can compare it against lv_screen_active(), same as the original
 * single-file lvgl_app.c did with its file-static s_watch_screen. */
lv_obj_t *s_watch_screen;

static void watch_face_update(lv_timer_t *timer)
{
    (void)timer;

    alarm_check();

    pcf85063a_time_t t;
    if (!sensor_cache_get_rtc(&t)) {
        return;
    }

    /* The RTC stores UTC directly; convert to local for the primary display. */
    time_t epoch = pcf85063a_time_to_epoch(&t);
    struct tm lt;
    localtime_r(&epoch, &lt);

    char buf[32];
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

    sensor_cache_t cache;
    sensor_cache_get(&cache);
    uint8_t pct = cache.batt_pct;
    if (cache.valid && pct <= 100) {
        snprintf(buf, sizeof(buf), "%u%%", pct);
        lv_label_set_text(s_batt_label, buf);
        lv_obj_set_width(s_batt_fill, (lv_coord_t)(140 * pct / 100));
    }

    if (s_steps_label) {
        uint32_t steps = 0;
        if (bhi260ap_get_daily_steps(&steps) == ESP_OK) {
            snprintf(buf, sizeof(buf), "Steps: %lu", (unsigned long)steps);
        } else {
            snprintf(buf, sizeof(buf), "Steps: --");
        }
        lv_label_set_text(s_steps_label, buf);
    }

    if (s_gps_icon) {
        m10q_state_t st = m10q_get_state();
        m10q_fix_t fix;
        m10q_get_fix(&fix);
        lv_color_t c;
        if (st == M10Q_STATE_FIXED && fix.valid && fix.fix_3d) {
            c = lv_color_hex(0x00E676);
        } else if (st == M10Q_STATE_ACQUIRING || (st == M10Q_STATE_FIXED && !fix.fix_3d)) {
            c = lv_color_hex(0xFF5252);
        } else {
            c = lv_color_hex(0x888888);
        }
        lv_obj_set_style_text_color(s_gps_icon, c, 0);
    }

    if (s_track_dot) {
        if (tracking_is_active()) {
            lv_obj_clear_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (s_snooze_icon) {
        if (alarm_is_snoozing()) {
            lv_obj_clear_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }

    update_status_bar(&s_status_bar);
    sim_cdtimer_tick();
}

void sim_watch_face_start(void)
{
    s_watch_screen = lv_screen_active();
    lv_obj_set_style_bg_color(s_watch_screen, lv_color_hex(0x000000), 0);

    build_status_bar(s_watch_screen, &s_status_bar);

    s_date_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_date_label, "");
    lv_obj_set_style_text_font(s_date_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, -100);

    s_gps_icon = lv_label_create(lv_screen_active());
    lv_label_set_text(s_gps_icon, LV_SYMBOL_GPS);
    lv_obj_set_style_text_font(s_gps_icon, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_gps_icon, lv_color_hex(0x888888), 0);
    lv_obj_align(s_gps_icon, LV_ALIGN_TOP_MID, 0, 24);

    s_track_dot = lv_obj_create(lv_screen_active());
    lv_obj_set_size(s_track_dot, 12, 12);
    lv_obj_align(s_track_dot, LV_ALIGN_TOP_MID, 40, 24);
    lv_obj_clear_flag(s_track_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_track_dot, lv_color_hex(0xFF2020), 0);
    lv_obj_set_style_radius(s_track_dot, 6, 0);
    lv_obj_set_style_pad_all(s_track_dot, 0, 0);
    lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);

    s_snooze_icon = lv_label_create(lv_screen_active());
    lv_label_set_text(s_snooze_icon, "Zz");
    lv_obj_set_style_text_font(s_snooze_icon, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_snooze_icon, lv_color_hex(0xFFD54F), 0);
    lv_obj_align(s_snooze_icon, LV_ALIGN_TOP_LEFT, 70, 40);
    lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);

    s_time_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_time_label, "--:--:--");
    lv_obj_set_style_text_font(s_time_label, s_font_time, 0);
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_letter_space(s_time_label, -6, 0);
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, -10);

    s_sec_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_sec_label, "UTC --:--");
    lv_obj_set_style_text_font(s_sec_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_sec_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_sec_label, LV_ALIGN_CENTER, 0, 65);

    lv_obj_t *bar = lv_obj_create(lv_screen_active());
    lv_obj_set_size(bar, 140, 12);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 185);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x666666), 0);
    lv_obj_set_style_border_width(bar, 2, 0);
    lv_obj_set_style_radius(bar, 6, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);

    s_batt_fill = lv_obj_create(bar);
    lv_obj_set_size(s_batt_fill, 0, 8);
    lv_obj_align(s_batt_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_clear_flag(s_batt_fill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_batt_fill, lv_color_hex(0x00E676), 0);
    lv_obj_set_style_radius(s_batt_fill, 4, 0);
    lv_obj_set_style_pad_all(s_batt_fill, 0, 0);

    s_batt_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_batt_label, "--");
    lv_obj_set_style_text_font(s_batt_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_batt_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_batt_label, LV_ALIGN_CENTER, 0, 208);

    s_steps_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_steps_label, "Steps: --");
    lv_obj_set_style_text_font(s_steps_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_steps_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_steps_label, LV_ALIGN_CENTER, 0, 150);

    watch_face_update(NULL);
    lv_timer_create(watch_face_update, 1000, NULL);
}

void sim_watch_face_refresh(void)
{
    watch_face_update(NULL);
}
