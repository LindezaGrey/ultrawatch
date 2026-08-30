/*
 * settings_screen.c - the Settings category list + its 5 sub-pages (Zeit &
 * Zeitzone, Display, Peripherie, Ton & Vibration, Info) from
 * main/lvgl_app.c (Phase 5), copied verbatim and run against mock_hw.c
 * instead of the real drivers/NVS.
 *
 * Deviations from the firmware original (all mechanical, not logic
 * changes):
 *   - screen_new() copied in here too, same as every non-watch-face screen.
 *   - No status bar: the sim never ported build_status_bar()/
 *     update_status_bar() to begin with (see mesh_screen.c/gps_screen.c -
 *     neither has one either), so the Settings category list simply omits
 *     it rather than half-porting a piece no other sim screen has.
 *   - lvgl_gps_set_enabled()/ble_debug_is_advertising()/
 *     ble_debug_set_advertising()/sd_log_get_space() are all mocked in
 *     mock_hw.c instead of driving real hardware/NVS.
 *   - esp_app_get_description()->version (ESP-IDF, unavailable on the host)
 *     is replaced with a fixed "sim" string for the Info page.
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_micro = &cascadia_18;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_settings_screen;

static lv_obj_t *s_set_tz_screen;
static lv_obj_t *s_set_tz_abbrev_label;
static lv_obj_t *s_set_tz_offset_label;

static lv_obj_t *s_set_disp_screen;
static lv_obj_t *s_set_disp_timeout_label;
static lv_obj_t *s_set_disp_bright_label;

static lv_obj_t *s_set_periph_screen;
static lv_obj_t *s_set_periph_gps_switch;
static lv_obj_t *s_set_periph_bt_switch;

static lv_obj_t *s_set_sound_screen;
static lv_obj_t *s_set_sound_alarm_switch;
static lv_obj_t *s_set_sound_notify_switch;

static lv_obj_t *s_set_info_screen;
static lv_obj_t *s_set_info_batt_label;
static lv_obj_t *s_set_info_sd_label;

static void lvgl_build_settings_tz_screen(void);
static void settings_tz_refresh(void);
static void lvgl_build_settings_disp_screen(void);
static void settings_disp_refresh(void);
static void lvgl_build_settings_periph_screen(void);
static void settings_periph_refresh(lv_timer_t *timer);
static void lvgl_build_settings_sound_screen(void);
static void settings_sound_refresh(void);
static void lvgl_build_settings_info_screen(void);
static void settings_info_refresh(lv_timer_t *timer);

static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

/* Local swipe-right-to-go-back gesture, shared by every Settings sub-page
 * (docs/application.md section 9.2) - distance-threshold only, registered
 * per-sub-page-root with the sub-page's own back callback as user data. */
#define SWIPE_DIST 60

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
    if (dx < SWIPE_DIST || abs(dx) <= abs(dy)) {
        return;
    }
    void (*back_cb)(lv_event_t *) = (void (*)(lv_event_t *))lv_event_get_user_data(e);
    back_cb(e);
}

static void settings_back_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_settings_screen);
}

static void settings_row_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    switch (idx) {
    case 0:
        if (!s_set_tz_screen) { lvgl_build_settings_tz_screen(); }
        lv_scr_load(s_set_tz_screen);
        settings_tz_refresh();
        break;
    case 1:
        if (!s_set_disp_screen) { lvgl_build_settings_disp_screen(); }
        lv_scr_load(s_set_disp_screen);
        settings_disp_refresh();
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
    default:
        break;
    }
}

static void lvgl_build_settings_screen(void)
{
    s_settings_screen = screen_new();
    lv_obj_set_style_bg_color(s_settings_screen, lv_color_hex(0x000000), 0);

    lv_obj_t *title = lv_label_create(s_settings_screen);
    lv_label_set_text(title, "SETTINGS");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    static const char *cat_names[5] = {
        "Zeit & Zeitzone", "Display", "Peripherie", "Ton & Vibration", "Info",
    };

    lv_obj_t *list_cont = lv_obj_create(s_settings_screen);
    lv_obj_set_size(list_cont, 370, 380);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 8, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 2, 0);

    for (int i = 0; i < 5; i++) {
        lv_obj_t *row = lv_obj_create(list_cont);
        lv_obj_set_size(row, LV_PCT(100), 50);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x202020), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 6, 0);
        lv_obj_set_style_pad_all(row, 10, 0);
        lv_obj_add_event_cb(row, settings_row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, cat_names[i]);
        lv_obj_set_style_text_font(l, s_font_small, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
    }
}

void sim_settings_screen_build(void)
{
    if (!s_settings_screen) {
        lvgl_build_settings_screen();
    }
    lv_scr_load(s_settings_screen);
}

/* ---- Zeit & Zeitzone ---- */

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
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_tz_screen);
    lv_label_set_text(title, "TIME & TIMEZONE");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_set_tz_abbrev_label = lv_label_create(s_set_tz_screen);
    lv_obj_set_style_text_font(s_set_tz_abbrev_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_tz_abbrev_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_tz_abbrev_label, LV_ALIGN_CENTER, 0, -20);

    s_set_tz_offset_label = lv_label_create(s_set_tz_screen);
    lv_obj_set_style_text_font(s_set_tz_offset_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_tz_offset_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_set_tz_offset_label, LV_ALIGN_CENTER, 0, 20);

    settings_tz_refresh();
}

/* ---- Display ---- */

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
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_disp_screen);
    lv_label_set_text(title, "DISPLAY");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_set_disp_timeout_label = lv_label_create(s_set_disp_screen);
    lv_obj_set_style_text_font(s_set_disp_timeout_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_set_disp_timeout_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_disp_timeout_label, LV_ALIGN_TOP_MID, 0, 90);

    lv_obj_t *t_minus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(t_minus, 150, 64);
    lv_obj_align(t_minus, LV_ALIGN_TOP_LEFT, 20, 130);
    lv_obj_t *tml = lv_label_create(t_minus);
    lv_label_set_text(tml, "-");
    lv_obj_set_style_text_font(tml, s_font_small, 0);
    lv_obj_center(tml);
    lv_obj_add_event_cb(t_minus, settings_disp_timeout_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *t_plus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(t_plus, 150, 64);
    lv_obj_align(t_plus, LV_ALIGN_TOP_RIGHT, -20, 130);
    lv_obj_t *tpl = lv_label_create(t_plus);
    lv_label_set_text(tpl, "+");
    lv_obj_set_style_text_font(tpl, s_font_small, 0);
    lv_obj_center(tpl);
    lv_obj_add_event_cb(t_plus, settings_disp_timeout_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    s_set_disp_bright_label = lv_label_create(s_set_disp_screen);
    lv_obj_set_style_text_font(s_set_disp_bright_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_set_disp_bright_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_disp_bright_label, LV_ALIGN_TOP_MID, 0, 230);

    lv_obj_t *b_minus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(b_minus, 150, 64);
    lv_obj_align(b_minus, LV_ALIGN_TOP_LEFT, 20, 270);
    lv_obj_t *bml = lv_label_create(b_minus);
    lv_label_set_text(bml, "-");
    lv_obj_set_style_text_font(bml, s_font_small, 0);
    lv_obj_center(bml);
    lv_obj_add_event_cb(b_minus, settings_disp_bright_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *b_plus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(b_plus, 150, 64);
    lv_obj_align(b_plus, LV_ALIGN_TOP_RIGHT, -20, 270);
    lv_obj_t *bpl = lv_label_create(b_plus);
    lv_label_set_text(bpl, "+");
    lv_obj_set_style_text_font(bpl, s_font_small, 0);
    lv_obj_center(bpl);
    lv_obj_add_event_cb(b_plus, settings_disp_bright_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    settings_disp_refresh();
}

/* Reachable both from the Settings category list and via tap-and-hold on
 * the watch face. */
void sim_settings_disp_screen_build(void)
{
    if (!s_set_disp_screen) {
        lvgl_build_settings_disp_screen();
    }
    lv_scr_load(s_set_disp_screen);
    settings_disp_refresh();
}

/* ---- Peripherie ---- */

static void settings_gps_switch_cb(lv_event_t *e)
{
    (void)e;
    lvgl_gps_set_enabled(lv_obj_has_state(s_set_periph_gps_switch, LV_STATE_CHECKED));
}

static void settings_bt_switch_cb(lv_event_t *e)
{
    (void)e;
    ble_debug_set_advertising(lv_obj_has_state(s_set_periph_bt_switch, LV_STATE_CHECKED));
}

static void settings_periph_refresh(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_set_periph_screen) {
        return;
    }
    bool gps_on = m10q_get_state() != M10Q_STATE_OFF;
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
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_periph_screen);
    lv_label_set_text(title, "PERIPHERIE");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *gps_lbl = lv_label_create(s_set_periph_screen);
    lv_label_set_text(gps_lbl, "GPS");
    lv_obj_set_style_text_font(gps_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(gps_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(gps_lbl, LV_ALIGN_TOP_LEFT, 20, 90);
    s_set_periph_gps_switch = lv_switch_create(s_set_periph_screen);
    lv_obj_align(s_set_periph_gps_switch, LV_ALIGN_TOP_RIGHT, -20, 88);
    lv_obj_add_event_cb(s_set_periph_gps_switch, settings_gps_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *bt_lbl = lv_label_create(s_set_periph_screen);
    lv_label_set_text(bt_lbl, "Bluetooth");
    lv_obj_set_style_text_font(bt_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(bt_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(bt_lbl, LV_ALIGN_TOP_LEFT, 20, 150);
    s_set_periph_bt_switch = lv_switch_create(s_set_periph_screen);
    lv_obj_align(s_set_periph_bt_switch, LV_ALIGN_TOP_RIGHT, -20, 148);
    lv_obj_add_event_cb(s_set_periph_bt_switch, settings_bt_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *lora_lbl = lv_label_create(s_set_periph_screen);
    lv_label_set_text(lora_lbl, "LoRa");
    lv_obj_set_style_text_font(lora_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(lora_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(lora_lbl, LV_ALIGN_TOP_LEFT, 20, 210);
    lv_obj_t *lora_sw = lv_switch_create(s_set_periph_screen);
    lv_obj_align(lora_sw, LV_ALIGN_TOP_RIGHT, -20, 208);
    lv_obj_add_state(lora_sw, LV_STATE_CHECKED | LV_STATE_DISABLED);
    lv_obj_t *lora_cap = lv_label_create(s_set_periph_screen);
    lv_label_set_text(lora_cap, "always on (rail hardwired)");
    lv_obj_set_style_text_font(lora_cap, s_font_micro, 0);
    lv_obj_set_style_text_color(lora_cap, lv_color_hex(0x707070), 0);
    lv_obj_align(lora_cap, LV_ALIGN_TOP_LEFT, 20, 240);

    lv_obj_t *wifi_lbl = lv_label_create(s_set_periph_screen);
    lv_label_set_text(wifi_lbl, "WiFi");
    lv_obj_set_style_text_font(wifi_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(wifi_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(wifi_lbl, LV_ALIGN_TOP_LEFT, 20, 280);
    lv_obj_t *wifi_sw = lv_switch_create(s_set_periph_screen);
    lv_obj_align(wifi_sw, LV_ALIGN_TOP_RIGHT, -20, 278);
    lv_obj_add_state(wifi_sw, LV_STATE_DISABLED);
    lv_obj_t *wifi_cap = lv_label_create(s_set_periph_screen);
    lv_label_set_text(wifi_cap, "not available yet");
    lv_obj_set_style_text_font(wifi_cap, s_font_micro, 0);
    lv_obj_set_style_text_color(wifi_cap, lv_color_hex(0x707070), 0);
    lv_obj_align(wifi_cap, LV_ALIGN_TOP_LEFT, 20, 310);

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
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_sound_screen);
    lv_label_set_text(title, "TON & VIBRATION");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *alarm_lbl = lv_label_create(s_set_sound_screen);
    lv_label_set_text(alarm_lbl, "Alarm sound");
    lv_obj_set_style_text_font(alarm_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(alarm_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(alarm_lbl, LV_ALIGN_TOP_LEFT, 20, 100);
    s_set_sound_alarm_switch = lv_switch_create(s_set_sound_screen);
    lv_obj_align(s_set_sound_alarm_switch, LV_ALIGN_TOP_RIGHT, -20, 98);
    lv_obj_add_event_cb(s_set_sound_alarm_switch, settings_alarm_sound_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *notify_lbl = lv_label_create(s_set_sound_screen);
    lv_label_set_text(notify_lbl, "LoRa notify vibr.");
    lv_obj_set_style_text_font(notify_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(notify_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(notify_lbl, LV_ALIGN_TOP_LEFT, 20, 160);
    s_set_sound_notify_switch = lv_switch_create(s_set_sound_screen);
    lv_obj_align(s_set_sound_notify_switch, LV_ALIGN_TOP_RIGHT, -20, 158);
    lv_obj_add_event_cb(s_set_sound_notify_switch, settings_notify_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    settings_sound_refresh();
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
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_info_screen);
    lv_label_set_text(title, "INFO");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *fw_label = lv_label_create(s_set_info_screen);
    lv_label_set_text(fw_label, "Firmware: sim");
    lv_obj_set_style_text_font(fw_label, s_font_micro, 0);
    lv_obj_set_style_text_color(fw_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(fw_label, LV_ALIGN_TOP_LEFT, 20, 100);

    s_set_info_batt_label = lv_label_create(s_set_info_screen);
    lv_obj_set_style_text_font(s_set_info_batt_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_set_info_batt_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_info_batt_label, LV_ALIGN_TOP_LEFT, 20, 140);

    s_set_info_sd_label = lv_label_create(s_set_info_screen);
    lv_obj_set_style_text_font(s_set_info_sd_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_set_info_sd_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_info_sd_label, LV_ALIGN_TOP_LEFT, 20, 180);

    settings_info_refresh(NULL);
    lv_timer_create(settings_info_refresh, 1000, NULL);
}
