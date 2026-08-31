/*
 * settings_screen.c - Settings category list (docs/application.md section
 * 9) + its 6 local sub-pages: Zeit & Zeitzone, Display, Peripherie, Ton &
 * Vibration, Info, Ultra-Sparmodus. Shared between the firmware and the
 * host sim - see watch_face.c's header comment for the mechanism.
 *
 * "Presets verwalten" is deferred (see Phase 5 plan) - LoRa preset text is
 * still editable, just via the `presetset` debug command instead of a
 * Settings category (this project deliberately has no on-watch text
 * keyboard). Everything here applies immediately except the display
 * timeout, which is flagged as taking effect after a restart.
 */
#include "screens.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "cascadia_fonts.h"
#include "power_mgmt.h"
#include "lvgl_app.h"
#include "m10q.h"
#include "ble_debug.h"
#include "alarm.h"
#include "mesh_log.h"
#include "haptic.h"
#include "sensor_cache.h"
#include "sd_log.h"
#include "portable_log.h"

static const lv_font_t *s_font_small = &cascadia_22;  /* body text */
static const lv_font_t *s_font_sec   = &cascadia_36;  /* titles/prominent labels - high-DPI sizing */

lv_obj_t *s_settings_screen;            /* category list */

lv_obj_t *s_set_tz_screen;
static lv_obj_t *s_set_tz_abbrev_label;
static lv_obj_t *s_set_tz_offset_label;

lv_obj_t *s_set_disp_screen;
static lv_obj_t *s_set_disp_timeout_label;
static lv_obj_t *s_set_disp_bright_label;

lv_obj_t *s_set_periph_screen;
static lv_obj_t *s_set_periph_gps_switch;
static lv_obj_t *s_set_periph_bt_switch;

lv_obj_t *s_set_sound_screen;
static lv_obj_t *s_set_sound_alarm_switch;
static lv_obj_t *s_set_sound_notify_switch;

lv_obj_t *s_set_vib_screen;
static lv_obj_t *s_vib_row[HAPTIC_PATTERN_COUNT];
static lv_obj_t *s_vib_row_label[HAPTIC_PATTERN_COUNT];

lv_obj_t *s_set_info_screen;
static lv_obj_t *s_set_info_batt_label;
static lv_obj_t *s_set_info_sd_label;

lv_obj_t *s_set_sparmodus_screen;
static lv_obj_t *s_set_sparmodus_switch;

static void settings_tz_refresh(void);
static void lvgl_build_settings_tz_screen(void);
static void settings_disp_refresh(void);
static void lvgl_build_settings_disp_screen(void);
static void settings_periph_refresh(lv_timer_t *timer);
static void lvgl_build_settings_periph_screen(void);
static void settings_sound_refresh(void);
static void lvgl_build_settings_sound_screen(void);
static void settings_vib_refresh(void);
static void lvgl_build_settings_vib_screen(void);
static void settings_vib_row_open_cb(lv_event_t *e);
static void settings_vib_pattern_click_cb(lv_event_t *e);
static void settings_vib_back_cb(lv_event_t *e);
static void settings_info_refresh(lv_timer_t *timer);
static void lvgl_build_settings_info_screen(void);
static void settings_sparmodus_refresh(void);
static void lvgl_build_settings_sparmodus_screen(void);
static void settings_back_cb(lv_event_t *e);

/* Local swipe-down-to-go-back gesture, shared by every Settings sub-page -
 * a top-to-bottom swipe anywhere on the sub-page returns to the category
 * list, same as tapping the "< Back" button (user_data is that button's
 * own click callback, invoked directly so both paths share one body). */
static void settings_sub_swipe_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_active();
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    static lv_point_t start;
    static bool active;

    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        start = p;
        active = true;
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_RELEASED || !active) {
        return;
    }
    active = false;
    int dx = p.x - start.x;
    int dy = p.y - start.y;
    if (dy < 60 || abs(dy) <= abs(dx)) {
        return;   /* only a top-to-bottom swipe counts as "back" */
    }
    void (*back_cb)(lv_event_t *) = (void (*)(lv_event_t *))lv_event_get_user_data(e);
    back_cb(e);
}

/* Category row -> sub-page dispatch, shared by all 6 rows. Each sub-page
 * is lazily built on first visit, same pattern as every other local
 * sub-screen in this file. Not static: sim/nav.c's UWATCH_SIM_SCREEN dev
 * shortcut calls this directly to reach a sub-page headlessly (none of
 * them have any other entry point besides this row-click dispatch and,
 * for Display only, the watch-face tap-and-hold shortcut). */
void settings_open_subpage(int idx)
{
    switch (idx) {
    case 0:
        if (!s_set_tz_screen) { lvgl_build_settings_tz_screen(); }
        lv_scr_load(s_set_tz_screen);
        settings_tz_refresh();
        break;
    case 1:
        lvgl_show_settings_disp();
        break;
    case 2:
        if (!s_set_periph_screen) { lvgl_build_settings_periph_screen(); }
        lv_scr_load(s_set_periph_screen);
        settings_periph_refresh(NULL);
        break;
    case 3:
        if (!s_set_sound_screen) { lvgl_build_settings_sound_screen(); }
        lv_scr_load(s_set_sound_screen);
        settings_sound_refresh();
        break;
    case 4:
        if (!s_set_info_screen) { lvgl_build_settings_info_screen(); }
        lv_scr_load(s_set_info_screen);
        settings_info_refresh(NULL);
        break;
    case 5:
        if (!s_set_sparmodus_screen) { lvgl_build_settings_sparmodus_screen(); }
        lv_scr_load(s_set_sparmodus_screen);
        settings_sparmodus_refresh();
        break;
    default:
        break;
    }
}

static void settings_row_click_cb(lv_event_t *e)
{
    settings_open_subpage((int)(intptr_t)lv_event_get_user_data(e));
}

static void settings_back_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_settings_screen);
}

void lvgl_build_settings_screen(void)
{
    s_settings_screen = screen_new();
    lv_obj_set_style_bg_color(s_settings_screen, lv_color_hex(0x000000), 0);

    /* High-DPI sizing (see AGENT.md "Display density & UI sizing"): title
     * and row labels use cascadia_36 (s_font_sec) instead of the old
     * cascadia_22 - at ~315 PPI, 22px body text reads under 2mm tall, too
     * small for comfortable reading/tapping. Rows are 64px tall (matches
     * the alarm-edit screen's stepper-button precedent for a
     * fingertip-sized touch target) and the list uses most of the safe
     * width instead of leaving margin on both sides. */
    lv_obj_t *title = lv_label_create(s_settings_screen);
    lv_label_set_text(title, "SETTINGS");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    /* 6th category (Ultra-Sparmodus, Phase 6) added back - rows shrink
     * 64->56px / pad 10->8px to fit all 6 above the bottom safe margin. */
    static const char *cat_names[6] = {
        "Zeit & Zeitzone", "Display", "Peripherie", "Ton & Vibration", "Info",
        "Ultra-Sparmodus",
    };

    lv_obj_t *list_cont = lv_obj_create(s_settings_screen);
    lv_obj_set_size(list_cont, 386, 352);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 90);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 8, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 0, 0);

    for (int i = 0; i < 6; i++) {
        lv_obj_t *row = lv_obj_create(list_cont);
        lv_obj_set_size(row, LV_PCT(100), 52);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x202020), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 10, 0);
        lv_obj_set_style_pad_all(row, 14, 0);
        lv_obj_add_event_cb(row, settings_row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, cat_names[i]);
        lv_obj_set_style_text_font(l, s_font_sec, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
    }
}

/* ---- Zeit & Zeitzone (info-only: no TZ auto-detect infra exists, see
 * Phase 5 plan) ---- */

static void settings_tz_refresh(void)
{
    if (!s_set_tz_abbrev_label) {
        return;
    }
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char abbrev[16];
    strftime(abbrev, sizeof(abbrev), "%Z", &tmv);
    char buf[32];
    snprintf(buf, sizeof(buf), "Zone: %s", abbrev);
    lv_label_set_text(s_set_tz_abbrev_label, buf);

    /* struct tm on this toolchain has no tm_gmtoff (picolibc) and there's
     * no timegm() either - derive the UTC offset from the local vs. UTC
     * wall-clock fields directly instead, day-wrap handled via tm_yday
     * (works for any offset in -24h..+24h, which covers every real zone). */
    struct tm utcv;
    gmtime_r(&now, &utcv);
    long local_secs = tmv.tm_hour * 3600L + tmv.tm_min * 60L + tmv.tm_sec;
    long utc_secs = utcv.tm_hour * 3600L + utcv.tm_min * 60L + utcv.tm_sec;
    long day_diff = tmv.tm_yday - utcv.tm_yday;
    if (tmv.tm_year != utcv.tm_year) {
        day_diff = (tmv.tm_year > utcv.tm_year) ? 1 : -1;
    } else if (day_diff > 1) {
        day_diff = -1;
    } else if (day_diff < -1) {
        day_diff = 1;
    }
    long off_s = (local_secs - utc_secs) + day_diff * 86400L;
    snprintf(buf, sizeof(buf), "UTC%+03ld:%02ld", off_s / 3600, labs(off_s % 3600) / 60);
    lv_label_set_text(s_set_tz_offset_label, buf);
}

static void lvgl_build_settings_tz_screen(void)
{
    s_set_tz_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_tz_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_tz_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_tz_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_tz_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    /* Title sits below the back button (not centered at the very top like
     * the category list) so a wide title never overlaps it - see AGENT.md
     * "Display density & UI sizing". */
    lv_obj_t *title = lv_label_create(s_set_tz_screen);
    lv_label_set_text(title, "TIME & TIMEZONE");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    s_set_tz_abbrev_label = lv_label_create(s_set_tz_screen);
    lv_obj_set_style_text_font(s_set_tz_abbrev_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_set_tz_abbrev_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_tz_abbrev_label, LV_ALIGN_TOP_MID, 0, 252);

    s_set_tz_offset_label = lv_label_create(s_set_tz_screen);
    lv_obj_set_style_text_font(s_set_tz_offset_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_set_tz_offset_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_set_tz_offset_label, LV_ALIGN_TOP_MID, 0, 312);

    settings_tz_refresh();
}

/* ---- Display: timeout (persists, takes effect after restart) + brightness
 * (applies immediately) ---- */

static const uint32_t s_disp_timeout_opts[] = { 5, 10, 15, 20, 30, 60 };
#define DISP_TIMEOUT_OPT_COUNT (sizeof(s_disp_timeout_opts) / sizeof(s_disp_timeout_opts[0]))
static const uint8_t s_disp_bright_opts[] = { 64, 128, 192, 255 };
#define DISP_BRIGHT_OPT_COUNT (sizeof(s_disp_bright_opts) / sizeof(s_disp_bright_opts[0]))

static void settings_disp_refresh(void)
{
    if (!s_set_disp_timeout_label) {
        return;
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "Timeout: %us (after restart)",
             (unsigned)power_mgmt_get_display_timeout_s());
    lv_label_set_text(s_set_disp_timeout_label, buf);

    uint8_t level = power_mgmt_get_brightness();
    snprintf(buf, sizeof(buf), "Brightness: %u%%", (unsigned)(level * 100 / 255));
    lv_label_set_text(s_set_disp_bright_label, buf);
}

static void settings_disp_timeout_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint32_t cur = power_mgmt_get_display_timeout_s();
    int idx = 0;
    for (size_t i = 0; i < DISP_TIMEOUT_OPT_COUNT; i++) {
        if (s_disp_timeout_opts[i] == cur) { idx = (int)i; break; }
    }
    idx = (idx + (int)DISP_TIMEOUT_OPT_COUNT + delta) % (int)DISP_TIMEOUT_OPT_COUNT;
    power_mgmt_set_display_timeout_s(s_disp_timeout_opts[idx]);
    settings_disp_refresh();
}

static void settings_disp_bright_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint8_t cur = power_mgmt_get_brightness();
    int idx = 0;
    for (size_t i = 0; i < DISP_BRIGHT_OPT_COUNT; i++) {
        if (s_disp_bright_opts[i] == cur) { idx = (int)i; break; }
    }
    idx = (idx + (int)DISP_BRIGHT_OPT_COUNT + delta) % (int)DISP_BRIGHT_OPT_COUNT;
    power_mgmt_set_brightness(s_disp_bright_opts[idx]);
    settings_disp_refresh();
}

static void lvgl_build_settings_disp_screen(void)
{
    s_set_disp_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_disp_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_disp_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_disp_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_disp_screen);
    lv_label_set_text(title, "DISPLAY");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    s_set_disp_timeout_label = lv_label_create(s_set_disp_screen);
    lv_obj_set_style_text_font(s_set_disp_timeout_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_disp_timeout_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_disp_timeout_label, LV_ALIGN_TOP_MID, 0, 164);

    lv_obj_t *t_minus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(t_minus, 165, 72);
    lv_obj_align(t_minus, LV_ALIGN_TOP_LEFT, 20, 204);
    lv_obj_t *tml = lv_label_create(t_minus);
    lv_label_set_text(tml, "-");
    lv_obj_set_style_text_font(tml, s_font_sec, 0);
    lv_obj_center(tml);
    lv_obj_add_event_cb(t_minus, settings_disp_timeout_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *t_plus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(t_plus, 165, 72);
    lv_obj_align(t_plus, LV_ALIGN_TOP_RIGHT, -20, 204);
    lv_obj_t *tpl = lv_label_create(t_plus);
    lv_label_set_text(tpl, "+");
    lv_obj_set_style_text_font(tpl, s_font_sec, 0);
    lv_obj_center(tpl);
    lv_obj_add_event_cb(t_plus, settings_disp_timeout_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    s_set_disp_bright_label = lv_label_create(s_set_disp_screen);
    lv_obj_set_style_text_font(s_set_disp_bright_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_disp_bright_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_disp_bright_label, LV_ALIGN_TOP_MID, 0, 320);

    lv_obj_t *b_minus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(b_minus, 165, 72);
    lv_obj_align(b_minus, LV_ALIGN_TOP_LEFT, 20, 360);
    lv_obj_t *bml = lv_label_create(b_minus);
    lv_label_set_text(bml, "-");
    lv_obj_set_style_text_font(bml, s_font_sec, 0);
    lv_obj_center(bml);
    lv_obj_add_event_cb(b_minus, settings_disp_bright_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *b_plus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(b_plus, 165, 72);
    lv_obj_align(b_plus, LV_ALIGN_TOP_RIGHT, -20, 360);
    lv_obj_t *bpl = lv_label_create(b_plus);
    lv_label_set_text(bpl, "+");
    lv_obj_set_style_text_font(bpl, s_font_sec, 0);
    lv_obj_center(bpl);
    lv_obj_add_event_cb(b_plus, settings_disp_bright_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    settings_disp_refresh();
}

void lvgl_show_settings_disp(void)
{
    if (!s_set_disp_screen) {
        lvgl_build_settings_disp_screen();
    }
    lv_scr_load(s_set_disp_screen);
    settings_disp_refresh();
    s_last_touch_tick = lv_tick_get();
}

/* ---- Peripherie: GPS/Bluetooth real toggles, LoRa/WiFi shown disabled
 * (no toggle capability exists for either yet, see Phase 5 plan) ---- */

static void settings_gps_switch_cb(lv_event_t *e)
{
    (void)e;
    bool on = lv_obj_has_state(s_set_periph_gps_switch, LV_STATE_CHECKED);
    ESP_LOGI("settings_screen", "settings_gps_switch_cb: on=%d", (int)on);
    lvgl_gps_set_enabled(on);
}

static void settings_bt_switch_cb(lv_event_t *e)
{
    (void)e;
    ble_debug_set_advertising(lv_obj_has_state(s_set_periph_bt_switch, LV_STATE_CHECKED));
}

/* Mirrors the GPS screen's own switch state formula (gps_screen_is_powered()
 * && m10q_get_state() != M10Q_STATE_OFF, screens/gps_screen.c's
 * gps_screen_update()) so the two switches never disagree about what
 * "on" means. */
static void settings_periph_refresh(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_set_periph_screen) {
        return;
    }
    bool gps_on = gps_screen_is_powered() && m10q_get_state() != M10Q_STATE_OFF;
    if (lv_obj_has_state(s_set_periph_gps_switch, LV_STATE_CHECKED) != gps_on) {
        if (gps_on) { lv_obj_add_state(s_set_periph_gps_switch, LV_STATE_CHECKED); }
        else { lv_obj_clear_state(s_set_periph_gps_switch, LV_STATE_CHECKED); }
    }
    bool bt_on = ble_debug_is_advertising();
    if (lv_obj_has_state(s_set_periph_bt_switch, LV_STATE_CHECKED) != bt_on) {
        if (bt_on) { lv_obj_add_state(s_set_periph_bt_switch, LV_STATE_CHECKED); }
        else { lv_obj_clear_state(s_set_periph_bt_switch, LV_STATE_CHECKED); }
    }
}

static void lvgl_build_settings_periph_screen(void)
{
    s_set_periph_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_periph_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_periph_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_periph_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_periph_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_periph_screen);
    lv_label_set_text(title, "PERIPHERIE");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    /* Each toggle sits in its own dark rounded card (same treatment as the
     * category list's rows) instead of floating label+switch pairs on bare
     * black - fills the panel's width properly and gives every row a
     * fingertip-sized tap target, not just the switch itself. */
    lv_obj_t *gps_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(gps_row, 386, 58);
    lv_obj_align(gps_row, LV_ALIGN_TOP_MID, 0, 152);
    lv_obj_clear_flag(gps_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(gps_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(gps_row, 0, 0);
    lv_obj_set_style_radius(gps_row, 10, 0);
    lv_obj_set_style_pad_all(gps_row, 14, 0);
    lv_obj_t *gps_lbl = lv_label_create(gps_row);
    lv_label_set_text(gps_lbl, "GPS");
    lv_obj_set_style_text_font(gps_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(gps_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(gps_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_periph_gps_switch = lv_switch_create(gps_row);
    lv_obj_set_size(s_set_periph_gps_switch, 66, 36);
    lv_obj_align(s_set_periph_gps_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_periph_gps_switch, settings_gps_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *bt_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(bt_row, 386, 58);
    lv_obj_align(bt_row, LV_ALIGN_TOP_MID, 0, 216);
    lv_obj_clear_flag(bt_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bt_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(bt_row, 0, 0);
    lv_obj_set_style_radius(bt_row, 10, 0);
    lv_obj_set_style_pad_all(bt_row, 14, 0);
    lv_obj_t *bt_lbl = lv_label_create(bt_row);
    lv_label_set_text(bt_lbl, "Bluetooth");
    lv_obj_set_style_text_font(bt_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(bt_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(bt_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_periph_bt_switch = lv_switch_create(bt_row);
    lv_obj_set_size(s_set_periph_bt_switch, 66, 36);
    lv_obj_align(s_set_periph_bt_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_periph_bt_switch, settings_bt_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *lora_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(lora_row, 386, 84);
    lv_obj_align(lora_row, LV_ALIGN_TOP_MID, 0, 280);
    lv_obj_clear_flag(lora_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lora_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(lora_row, 0, 0);
    lv_obj_set_style_radius(lora_row, 10, 0);
    lv_obj_set_style_pad_all(lora_row, 14, 0);
    lv_obj_t *lora_lbl = lv_label_create(lora_row);
    lv_label_set_text(lora_lbl, "LoRa");
    lv_obj_set_style_text_font(lora_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(lora_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(lora_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *lora_cap = lv_label_create(lora_row);
    lv_label_set_text(lora_cap, "hardwired, always on");
    lv_obj_set_style_text_font(lora_cap, s_font_small, 0);
    lv_obj_set_style_text_color(lora_cap, lv_color_hex(0x707070), 0);
    lv_obj_align(lora_cap, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_t *lora_sw = lv_switch_create(lora_row);
    lv_obj_set_size(lora_sw, 66, 36);
    lv_obj_align(lora_sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(lora_sw, LV_STATE_CHECKED | LV_STATE_DISABLED);

    lv_obj_t *wifi_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(wifi_row, 386, 84);
    lv_obj_align(wifi_row, LV_ALIGN_TOP_MID, 0, 370);
    lv_obj_clear_flag(wifi_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(wifi_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(wifi_row, 0, 0);
    lv_obj_set_style_radius(wifi_row, 10, 0);
    lv_obj_set_style_pad_all(wifi_row, 14, 0);
    lv_obj_t *wifi_lbl = lv_label_create(wifi_row);
    lv_label_set_text(wifi_lbl, "WiFi");
    lv_obj_set_style_text_font(wifi_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(wifi_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(wifi_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *wifi_cap = lv_label_create(wifi_row);
    lv_label_set_text(wifi_cap, "not available yet");
    lv_obj_set_style_text_font(wifi_cap, s_font_small, 0);
    lv_obj_set_style_text_color(wifi_cap, lv_color_hex(0x707070), 0);
    lv_obj_align(wifi_cap, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_t *wifi_sw = lv_switch_create(wifi_row);
    lv_obj_set_size(wifi_sw, 66, 36);
    lv_obj_align(wifi_sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(wifi_sw, LV_STATE_DISABLED);

    settings_periph_refresh(NULL);
    lv_timer_create(settings_periph_refresh, 1000, NULL);
}

/* ---- Ton & Vibration ---- */

static void settings_alarm_sound_switch_cb(lv_event_t *e)
{
    (void)e;
    alarm_set_sound_enabled(lv_obj_has_state(s_set_sound_alarm_switch, LV_STATE_CHECKED));
}

static void settings_notify_switch_cb(lv_event_t *e)
{
    (void)e;
    mesh_log_set_notify_enabled(lv_obj_has_state(s_set_sound_notify_switch, LV_STATE_CHECKED));
}

static void settings_sound_refresh(void)
{
    if (!s_set_sound_alarm_switch) {
        return;
    }
    if (alarm_get_sound_enabled()) { lv_obj_add_state(s_set_sound_alarm_switch, LV_STATE_CHECKED); }
    else { lv_obj_clear_state(s_set_sound_alarm_switch, LV_STATE_CHECKED); }
    if (mesh_log_get_notify_enabled()) { lv_obj_add_state(s_set_sound_notify_switch, LV_STATE_CHECKED); }
    else { lv_obj_clear_state(s_set_sound_notify_switch, LV_STATE_CHECKED); }
}

static void lvgl_build_settings_sound_screen(void)
{
    s_set_sound_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_sound_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_sound_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_sound_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_sound_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_sound_screen);
    lv_label_set_text(title, "TON & VIBRATION");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    lv_obj_t *alarm_row = lv_obj_create(s_set_sound_screen);
    lv_obj_set_size(alarm_row, 386, 76);
    lv_obj_align(alarm_row, LV_ALIGN_TOP_MID, 0, 172);
    lv_obj_clear_flag(alarm_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(alarm_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(alarm_row, 0, 0);
    lv_obj_set_style_radius(alarm_row, 10, 0);
    lv_obj_set_style_pad_all(alarm_row, 16, 0);
    lv_obj_t *alarm_lbl = lv_label_create(alarm_row);
    lv_label_set_text(alarm_lbl, "Alarm sound");
    lv_obj_set_style_text_font(alarm_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(alarm_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(alarm_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_sound_alarm_switch = lv_switch_create(alarm_row);
    lv_obj_set_size(s_set_sound_alarm_switch, 66, 36);
    lv_obj_align(s_set_sound_alarm_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_sound_alarm_switch, settings_alarm_sound_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *notify_row = lv_obj_create(s_set_sound_screen);
    lv_obj_set_size(notify_row, 386, 76);
    lv_obj_align(notify_row, LV_ALIGN_TOP_MID, 0, 272);
    lv_obj_clear_flag(notify_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(notify_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(notify_row, 0, 0);
    lv_obj_set_style_radius(notify_row, 10, 0);
    lv_obj_set_style_pad_all(notify_row, 16, 0);
    lv_obj_t *notify_lbl = lv_label_create(notify_row);
    lv_label_set_text(notify_lbl, "LoRa notify vibr.");
    lv_obj_set_style_text_font(notify_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(notify_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(notify_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_sound_notify_switch = lv_switch_create(notify_row);
    lv_obj_set_size(s_set_sound_notify_switch, 66, 36);
    lv_obj_align(s_set_sound_notify_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_sound_notify_switch, settings_notify_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Third row navigates to a nested sub-page (like a category row does
     * from the top-level list) rather than holding its own control here -
     * a list of HAPTIC_PATTERN_COUNT presets needs more room than this
     * screen's remaining space allows. */
    lv_obj_t *vib_row = lv_obj_create(s_set_sound_screen);
    lv_obj_set_size(vib_row, 386, 76);
    lv_obj_align(vib_row, LV_ALIGN_TOP_MID, 0, 372);
    lv_obj_clear_flag(vib_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(vib_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(vib_row, 0, 0);
    lv_obj_set_style_radius(vib_row, 10, 0);
    lv_obj_set_style_pad_all(vib_row, 16, 0);
    lv_obj_t *vib_lbl = lv_label_create(vib_row);
    lv_label_set_text(vib_lbl, "Vibration pattern >");
    lv_obj_set_style_text_font(vib_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(vib_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(vib_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(vib_row, settings_vib_row_open_cb, LV_EVENT_CLICKED, NULL);

    settings_sound_refresh();
}

/* Not static: sim/nav.c's UWATCH_SIM_SCREEN dev shortcut calls this
 * directly too, same reason settings_open_subpage() is exported. */
void settings_open_vib_screen(void)
{
    if (!s_set_vib_screen) { lvgl_build_settings_vib_screen(); }
    lv_scr_load(s_set_vib_screen);
    settings_vib_refresh();
}

static void settings_vib_row_open_cb(lv_event_t *e)
{
    (void)e;
    settings_open_vib_screen();
}

static void settings_vib_back_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_set_sound_screen);
}

/* Tapping a pattern both selects it (persisted) and fires it immediately -
 * the tap itself is the "Test" action, no separate button needed: you feel
 * the difference the instant you pick it. */
static void settings_vib_pattern_click_cb(lv_event_t *e)
{
    size_t idx = (size_t)(intptr_t)lv_event_get_user_data(e);
    haptic_set_pattern_index(idx);
    /* haptic_play_test() does ~420ms of vTaskDelay() around the real I2C
     * play - calling it directly here would block the whole UI for that
     * long, since this callback runs on the LVGL task while
     * esp_lv_adapter_lock() is held (every other screen's timers/redraws/
     * touch input stall too, not just this one). Fire it on a short-lived
     * task instead - haptic_play_test() is already self-contained
     * (enable/play/disable), nothing here needs to wait for it to finish. */
    haptic_play_test_async(HAPTIC_PATTERNS[idx].wave_id);
    settings_vib_refresh();
}

static void settings_vib_refresh(void)
{
    if (!s_vib_row[0]) {
        return;
    }
    size_t sel = haptic_get_pattern_index();
    for (size_t i = 0; i < HAPTIC_PATTERN_COUNT; i++) {
        lv_obj_set_style_bg_color(s_vib_row[i],
            (i == sel) ? lv_color_hex(0x2E4A2E) : lv_color_hex(0x202020), 0);
        lv_obj_set_style_text_color(s_vib_row_label[i],
            (i == sel) ? lv_color_hex(0x80FF80) : lv_color_hex(0xE0E0E0), 0);
    }
}

static void lvgl_build_settings_vib_screen(void)
{
    s_set_vib_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_vib_screen, lv_color_hex(0x000000), 0);
    /* Swipe-back returns to the Sound screen, not the category list - a
     * nested sub-page, same one-level-up convention as Mesh's Node
     * overview swiping back to Mesh rather than the watch face. */
    lv_obj_add_event_cb(s_set_vib_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_vib_back_cb);
    lv_obj_add_event_cb(s_set_vib_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_vib_back_cb);

    lv_obj_t *back = lv_button_create(s_set_vib_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_vib_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_vib_screen);
    lv_label_set_text(title, "VIBRATION PATTERN");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    lv_obj_t *list_cont = lv_obj_create(s_set_vib_screen);
    lv_obj_set_size(list_cont, 386, 320);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 160);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 8, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 0, 0);

    for (size_t i = 0; i < HAPTIC_PATTERN_COUNT; i++) {
        lv_obj_t *row = lv_obj_create(list_cont);
        lv_obj_set_size(row, LV_PCT(100), 52);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 10, 0);
        lv_obj_set_style_pad_all(row, 14, 0);
        lv_obj_add_event_cb(row, settings_vib_pattern_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        s_vib_row[i] = row;

        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, HAPTIC_PATTERNS[i].name);
        lv_obj_set_style_text_font(l, s_font_small, 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
        s_vib_row_label[i] = l;
    }

    settings_vib_refresh();
}

/* ---- Info ---- */

static void settings_info_refresh(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_set_info_screen) {
        return;
    }
    sensor_cache_t cache;
    sensor_cache_get(&cache);
    char buf[48];
    if (cache.valid) {
        snprintf(buf, sizeof(buf), "Battery: %u%%", (unsigned)cache.batt_pct);
    } else {
        snprintf(buf, sizeof(buf), "Battery: --");
    }
    lv_label_set_text(s_set_info_batt_label, buf);

    uint64_t total = 0, free = 0;
    if (sd_log_get_space(&total, &free) == ESP_OK) {
        snprintf(buf, sizeof(buf), "SD: %.1f / %.1f GB free",
                 free / 1073741824.0, total / 1073741824.0);
    } else {
        snprintf(buf, sizeof(buf), "SD: not available");
    }
    lv_label_set_text(s_set_info_sd_label, buf);
}

static void lvgl_build_settings_info_screen(void)
{
    s_set_info_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_info_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_info_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_info_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_info_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_info_screen);
    lv_label_set_text(title, "INFO");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    lv_obj_t *fw_label = lv_label_create(s_set_info_screen);
    char fw_buf[48];
    snprintf(fw_buf, sizeof(fw_buf), "Firmware: v%s", uwatch_firmware_version());
    lv_label_set_text(fw_label, fw_buf);
    lv_obj_set_style_text_font(fw_label, s_font_small, 0);
    lv_obj_set_style_text_color(fw_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(fw_label, LV_ALIGN_TOP_LEFT, 20, 172);

    s_set_info_batt_label = lv_label_create(s_set_info_screen);
    lv_obj_set_style_text_font(s_set_info_batt_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_info_batt_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_info_batt_label, LV_ALIGN_TOP_LEFT, 20, 228);

    s_set_info_sd_label = lv_label_create(s_set_info_screen);
    lv_obj_set_style_text_font(s_set_info_sd_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_info_sd_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_info_sd_label, LV_ALIGN_TOP_LEFT, 20, 284);

    settings_info_refresh(NULL);
    lv_timer_create(settings_info_refresh, 1000, NULL);
}

/* ---- Ultra-Sparmodus (docs/application.md section 10, Phase 6) ----
 * Stage 1: the toggle persists power_mgmt_get/set_sparmodus_active() to
 * NVS only - flipping it does NOT yet shut peripherals down or deep-sleep
 * (that lands in a later stage, see the Phase 6 plan). */

static void settings_sparmodus_switch_cb(lv_event_t *e)
{
    (void)e;
    power_mgmt_set_sparmodus_active(lv_obj_has_state(s_set_sparmodus_switch, LV_STATE_CHECKED));
}

static void settings_sparmodus_refresh(void)
{
    if (!s_set_sparmodus_switch) {
        return;
    }
    if (power_mgmt_get_sparmodus_active()) {
        lv_obj_add_state(s_set_sparmodus_switch, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(s_set_sparmodus_switch, LV_STATE_CHECKED);
    }
}

static void lvgl_build_settings_sparmodus_screen(void)
{
    s_set_sparmodus_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_sparmodus_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_sparmodus_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_sparmodus_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_sparmodus_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_sparmodus_screen);
    lv_label_set_text(title, "ULTRA-SPARMODUS");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    lv_obj_t *sw_row = lv_obj_create(s_set_sparmodus_screen);
    lv_obj_set_size(sw_row, 386, 64);
    lv_obj_align(sw_row, LV_ALIGN_TOP_MID, 0, 160);
    lv_obj_clear_flag(sw_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(sw_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(sw_row, 0, 0);
    lv_obj_set_style_radius(sw_row, 10, 0);
    lv_obj_set_style_pad_all(sw_row, 14, 0);
    lv_obj_t *sw_lbl = lv_label_create(sw_row);
    lv_label_set_text(sw_lbl, "Active");
    lv_obj_set_style_text_font(sw_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(sw_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(sw_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_sparmodus_switch = lv_switch_create(sw_row);
    lv_obj_set_size(s_set_sparmodus_switch, 66, 36);
    lv_obj_align(s_set_sparmodus_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_sparmodus_switch, settings_sparmodus_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *hint = lv_label_create(s_set_sparmodus_screen);
    lv_label_set_text(hint, "Below 10% battery, this\nturns on automatically.");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x707070), 0);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 20, 240);

    settings_sparmodus_refresh();
}
