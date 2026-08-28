/*
 * power_screen.c - the real power-management screen from main/lvgl_app.c
 * (lines ~472-687 as of the porting pass), copied verbatim
 * (power_screen_update / power_chg_switch_cb / power_night_switch_cb /
 * power_usb_switch_cb / power_cur_btn_cb / lvgl_build_power_screen,
 * unmodified widget layout and logic) and run against mock_hw.c instead of
 * real drivers.
 *
 * Deviations from the firmware original (all mechanical, not logic
 * changes):
 *   - screen_new() is copied in here too (portable helper, no ESP-IDF deps,
 *     shared verbatim by every non-watch-face screen per the porting brief).
 *   - No esp_lv_adapter_lock()/unlock() around anything: the sim is
 *     single-threaded, so the guarded code runs directly.
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include <stdlib.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_small = &cascadia_22;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_power_screen;

static lv_obj_t *s_pw_batt_label;
static lv_obj_t *s_pw_chg_label;
static lv_obj_t *s_pw_temp_label;
static lv_obj_t *s_pw_runtime_label;
static lv_obj_t *s_pw_chg_switch;
static lv_obj_t *s_pw_night_switch;
static lv_obj_t *s_pw_usb_switch;
static lv_obj_t *s_pw_cur_100;
static lv_obj_t *s_pw_cur_400;

/* New screen with scrolling disabled: no scrollbars, content locked so
 * swipes always reach the screen-swipe navigation instead of scrolling the
 * content. (main/lvgl_app.c screen_new(), copied verbatim.) */
static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

static void power_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_pw_batt_label) {
        return;
    }
    /* The timer fires every second forever; skip the AXP I2C reads when the
     * power screen is not active (saves battery + bus traffic). */
    if (lv_screen_active() != s_power_screen) {
        return;
    }

    char buf[64];
    sensor_cache_t cache;
    sensor_cache_get(&cache);

    uint8_t pct = cache.batt_pct;
    uint16_t mv = cache.batt_mv;
    snprintf(buf, sizeof(buf), "Battery  %u%%  %.3f V", pct, mv / 1000.0f);
    lv_label_set_text(s_pw_batt_label, buf);

    if (cache.valid) {
        const char *s;
        switch (cache.chg_state) {
        case AXP2101_CHG_TRI: s = "Trickle"; break;
        case AXP2101_CHG_PRE: s = "Pre-charge"; break;
        case AXP2101_CHG_CC:  s = "Charging CC"; break;
        case AXP2101_CHG_CV:  s = "Charging CV"; break;
        case AXP2101_CHG_DONE: s = "Charged"; break;
        default:              s = "Not charging"; break;
        }
        snprintf(buf, sizeof(buf), "%s%s  %umA", s, cache.chg_enabled ? "" : " (disabled)",
                 cache.chg_ma);
        lv_label_set_text(s_pw_chg_label, buf);

        /* Runtime / charge-time from the background battery gauge estimate. */
        battery_estimate_t est;
        battery_estimate_get(&est);
        if (s_pw_runtime_label) {
            if (est.estimate_valid && est.runtime_h > 0) {
                uint32_t mins = (uint32_t)(est.runtime_h * 60.0f);
                snprintf(buf, sizeof(buf), "Runtime: ~%uh %02um", (unsigned)(mins / 60), (unsigned)(mins % 60));
            } else {
                snprintf(buf, sizeof(buf), "Runtime: --");
            }
            lv_label_set_text(s_pw_runtime_label, buf);
        }

        int16_t tc = cache.batt_temp_c10;
        snprintf(buf, sizeof(buf), "Battery temp  %d.%d C", tc / 10, abs(tc % 10));
        lv_label_set_text(s_pw_temp_label, buf);

        /* Keep switch/current buttons reflecting hardware state. */
        if (lv_obj_has_state(s_pw_chg_switch, LV_STATE_CHECKED) != cache.chg_enabled) {
            if (cache.chg_enabled) {
                lv_obj_add_state(s_pw_chg_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(s_pw_chg_switch, LV_STATE_CHECKED);
            }
        }
        /* Keep the config switches reflecting the persisted settings. */
        if (lv_obj_has_state(s_pw_night_switch, LV_STATE_CHECKED) != power_mgmt_get_night_mode_auto()) {
            if (power_mgmt_get_night_mode_auto()) {
                lv_obj_add_state(s_pw_night_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(s_pw_night_switch, LV_STATE_CHECKED);
            }
        }
        if (lv_obj_has_state(s_pw_usb_switch, LV_STATE_CHECKED) != power_mgmt_get_skip_sleep_on_usb()) {
            if (power_mgmt_get_skip_sleep_on_usb()) {
                lv_obj_add_state(s_pw_usb_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(s_pw_usb_switch, LV_STATE_CHECKED);
            }
        }
        bool cur100 = (cache.chg_ma <= 150);
        lv_obj_add_state(s_pw_cur_100, LV_STATE_CHECKED);
        lv_obj_add_state(s_pw_cur_400, LV_STATE_CHECKED);
        lv_obj_clear_state(cur100 ? s_pw_cur_100 : s_pw_cur_400, LV_STATE_CHECKED);
    }
}

static void power_chg_switch_cb(lv_event_t *e)
{
    (void)e;
    bool en = lv_obj_has_state(s_pw_chg_switch, LV_STATE_CHECKED);
    axp2101_set_charge_enabled(twatch_pmu_dev, en);
}

static void power_night_switch_cb(lv_event_t *e)
{
    (void)e;
    power_mgmt_set_night_mode_auto(lv_obj_has_state(s_pw_night_switch, LV_STATE_CHECKED));
}

static void power_usb_switch_cb(lv_event_t *e)
{
    (void)e;
    power_mgmt_set_skip_sleep_on_usb(lv_obj_has_state(s_pw_usb_switch, LV_STATE_CHECKED));
}

static void power_cur_btn_cb(lv_event_t *e)
{
    (void)e;
    uint16_t ma = (uint16_t)(uintptr_t)lv_event_get_user_data(e);
    axp2101_set_charge_current_ma(twatch_pmu_dev, ma);
    lv_obj_add_state(s_pw_cur_100, LV_STATE_CHECKED);
    lv_obj_add_state(s_pw_cur_400, LV_STATE_CHECKED);
    lv_obj_clear_state(ma == 100 ? s_pw_cur_100 : s_pw_cur_400, LV_STATE_CHECKED);
}

static void lvgl_build_power_screen(void)
{
    s_power_screen = screen_new();
    lv_obj_set_style_bg_color(s_power_screen, lv_color_hex(0x0A1030), 0);

    lv_obj_t *title = lv_label_create(s_power_screen);
    lv_label_set_text(title, "Power Management");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_pw_batt_label = lv_label_create(s_power_screen);
    lv_label_set_text(s_pw_batt_label, "");
    lv_obj_set_style_text_font(s_pw_batt_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_pw_batt_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_pw_batt_label, LV_ALIGN_TOP_MID, 0, 60);

    s_pw_chg_label = lv_label_create(s_power_screen);
    lv_label_set_text(s_pw_chg_label, "");
    lv_obj_set_style_text_font(s_pw_chg_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_pw_chg_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_pw_chg_label, LV_ALIGN_TOP_MID, 0, 100);

    s_pw_temp_label = lv_label_create(s_power_screen);
    lv_label_set_text(s_pw_temp_label, "");
    lv_obj_set_style_text_font(s_pw_temp_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_pw_temp_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_pw_temp_label, LV_ALIGN_TOP_MID, 0, 140);

    s_pw_runtime_label = lv_label_create(s_power_screen);
    lv_label_set_text(s_pw_runtime_label, "");
    lv_obj_set_style_text_font(s_pw_runtime_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_pw_runtime_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_pw_runtime_label, LV_ALIGN_TOP_MID, 0, 170);

    /* Charging enable switch. */
    lv_obj_t *sw_lbl = lv_label_create(s_power_screen);
    lv_label_set_text(sw_lbl, "Charging");
    lv_obj_set_style_text_font(sw_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(sw_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(sw_lbl, LV_ALIGN_TOP_LEFT, 40, 200);

    s_pw_chg_switch = lv_switch_create(s_power_screen);
    lv_obj_align(s_pw_chg_switch, LV_ALIGN_TOP_RIGHT, -40, 200);
    lv_obj_add_event_cb(s_pw_chg_switch, power_chg_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Charging current: two fixed settings (100 mA default / 400 mA max). */
    lv_obj_t *cur_lbl = lv_label_create(s_power_screen);
    lv_label_set_text(cur_lbl, "Charge current");
    lv_obj_set_style_text_font(cur_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(cur_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(cur_lbl, LV_ALIGN_TOP_LEFT, 40, 260);

    s_pw_cur_100 = lv_button_create(s_power_screen);
    lv_obj_set_size(s_pw_cur_100, 160, 44);
    lv_obj_align(s_pw_cur_100, LV_ALIGN_TOP_LEFT, 40, 310);
    lv_obj_t *l100 = lv_label_create(s_pw_cur_100);
    lv_label_set_text(l100, "100mA");
    lv_obj_set_style_text_font(l100, s_font_small, 0);
    lv_obj_center(l100);
    lv_obj_add_flag(s_pw_cur_100, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_pw_cur_100, power_cur_btn_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)100);

    s_pw_cur_400 = lv_button_create(s_power_screen);
    lv_obj_set_size(s_pw_cur_400, 160, 44);
    lv_obj_align(s_pw_cur_400, LV_ALIGN_TOP_RIGHT, -40, 310);
    lv_obj_t *l400 = lv_label_create(s_pw_cur_400);
    lv_label_set_text(l400, "400mA");
    lv_obj_set_style_text_font(l400, s_font_small, 0);
    lv_obj_center(l400);
    lv_obj_add_flag(s_pw_cur_400, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_pw_cur_400, power_cur_btn_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)400);

    /* Night mode auto switch. */
    lv_obj_t *night_lbl = lv_label_create(s_power_screen);
    lv_label_set_text(night_lbl, "Night mode auto");
    lv_obj_set_style_text_font(night_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(night_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(night_lbl, LV_ALIGN_TOP_LEFT, 40, 380);

    s_pw_night_switch = lv_switch_create(s_power_screen);
    lv_obj_align(s_pw_night_switch, LV_ALIGN_TOP_RIGHT, -40, 380);
    lv_obj_add_event_cb(s_pw_night_switch, power_night_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Do-not-sleep-on-USB switch. */
    /* y=436, not 416: see main/lvgl_app.c's comment at this same spot -
     * opens up separation from the night-mode switch above it. */
    lv_obj_t *usb_lbl = lv_label_create(s_power_screen);
    lv_label_set_text(usb_lbl, "No sleep on USB");
    lv_obj_set_style_text_font(usb_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(usb_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(usb_lbl, LV_ALIGN_TOP_LEFT, 40, 436);

    s_pw_usb_switch = lv_switch_create(s_power_screen);
    lv_obj_align(s_pw_usb_switch, LV_ALIGN_TOP_RIGHT, -40, 436);
    lv_obj_add_event_cb(s_pw_usb_switch, power_usb_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Hint. */
    lv_obj_t *hint = lv_label_create(s_power_screen);
    lv_label_set_text(hint, "swipe down to go back");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -6);

    lv_timer_create(power_screen_update, 1000, NULL);
    power_screen_update(NULL);   /* populate instantly from the cached snapshot */
}

void sim_power_screen_build(void)
{
    if (!s_power_screen) {
        lvgl_build_power_screen();
    }
    lv_scr_load(s_power_screen);
}
