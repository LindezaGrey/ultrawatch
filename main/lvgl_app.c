/*
 * lvgl_app.c - LVGL UI on the CO5300 AMOLED via esp_lvgl_adapter.
 *
 * The adapter runs LVGL in its own FreeRTOS task (tick + locking included).
 *   - RGB565_SWAPPED: the CO5300 samples big-endian RGB565.
 *   - Even-coordinate areas rounded BEFORE rendering (SH8601 requirement).
 *   - High-DPI vector fonts (Roboto) via LVGL FreeType from SPIFFS.
 *   - Boot screen -> watch face (time from the RTC-synced system clock).
 */
#include "lvgl_app.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_app_desc.h"
#include "esp_spiffs.h"
#include "driver/usb_serial_jtag.h"
#include "esp_lv_adapter.h"
#include "lvgl.h"
#include "libs/freetype/lv_freetype.h"
#include "co5300.h"
#include "cst9217.h"
#include "bhi260ap.h"
#include "sd_log.h"
#include "pcf85063a.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "m10q.h"
#include "power_mgmt.h"

static const char *TAG = "lvgl_app";

/* Fonts. */
static const lv_font_t *s_font_time = NULL;   /* 96 px  HH:MM */
static const lv_font_t *s_font_sec  = NULL;   /* 40 px  seconds */
static const lv_font_t *s_font_small = NULL;  /* 28 px  date/battery */

/* Watch face objects. */
static lv_obj_t *s_time_label;
static lv_obj_t *s_sec_label;
static lv_obj_t *s_date_label;
static lv_obj_t *s_batt_label;
static lv_obj_t *s_batt_fill;

/* Power management screen. */
static lv_obj_t *s_power_screen;
static lv_obj_t *s_pw_batt_label;
static lv_obj_t *s_pw_chg_label;
static lv_obj_t *s_pw_temp_label;
static lv_obj_t *s_pw_chg_switch;
static lv_obj_t *s_pw_cur_100;
static lv_obj_t *s_pw_cur_400;

/* BHI260AP status screen. */
static lv_obj_t *s_bhi_screen;
static lv_obj_t *s_bhi_status_label;
static lv_obj_t *s_bhi_steps_label;
static lv_obj_t *s_bhi_accel_label;
static lv_obj_t *s_bhi_gyro_label;
static lv_obj_t *s_bhi_ori_label;
static lv_obj_t *s_bhi_rv_label;
static lv_obj_t *s_bhi_activity_label;
static lv_obj_t *s_bhi_gesture_label;

/* GPS screen (skyplot + fix info). */
static lv_obj_t *s_gps_screen;
static lv_obj_t *s_gps_status_label;
static lv_obj_t *s_gps_pos_label;
static lv_obj_t *s_gps_speed_label;
static lv_obj_t *s_gps_sats_label;
static lv_obj_t *s_gps_dots[M10Q_MAX_SATS];   /* satellite dots (in view order) */
static bool s_gps_powered;
static uint32_t s_gps_acq_start_ms;            /* power-on timestamp */

/* Swipe detection at the input-device level (works regardless of widget). */
#define SWIPE_DIST         60
#define MENU_TIMEOUT_MS    5000   /* return to watch face after this idle */
static lv_indev_t *s_touch_indev;
static lv_point_t s_swipe_start;
static bool s_swipe_active;
static lv_obj_t *s_watch_screen;
static uint32_t s_last_touch_tick;   /* lv_tick_get() at last touch */

static void swipe_event_cb(lv_event_t *e);
static void lvgl_build_power_screen(void);
static void lvgl_build_bhi_screen(void);
static void lvgl_build_gps_screen(void);
static void gps_power(bool on);
static void lvgl_build_watch_face(void);
static void menu_timeout_cb(lv_timer_t *timer);
static void lvgl_show_watch_face(void);

/* Round invalidated areas to even coordinates (SH8601 requirement) BEFORE
 * LVGL renders, so the buffer content always matches the flushed area. */
static void area_rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}

/* Red-only night-mode transform. LVGL renders RGB565_SWAPPED (big-endian on
 * the panel): each pixel is 2 bytes, byte0 = MSB = RRRRR GGG, byte1 = GGG BBBBB.
 * Keeping only the red channel zeroes green/blue. Applied in-place; the blit is
 * synchronous, so the buffer is safe to mutate before esp_lcd_panel_draw_bitmap. */
static esp_err_t night_mode_draw_bitmap(lv_display_t *disp, esp_lcd_panel_handle_t panel,
                                        int x_start, int y_start, int x_end, int y_end,
                                        const void *color_map, void *user_ctx)
{
    (void)disp;
    (void)user_ctx;
    if (power_mgmt_is_night_mode()) {
        uint8_t *buf = (uint8_t *)color_map;
        size_t n = (size_t)(x_end - x_start) * (y_end - y_start);
        for (size_t i = 0; i < n; i++) {
            buf[i * 2]     &= 0xF8;   /* keep 5-bit red */
            buf[i * 2 + 1]  = 0x00;   /* drop green/blue */
        }
    }
    return esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end, y_end, color_map);
}

/* On a night-mode change, invalidate the whole screen so every pixel is
 * redrawn through the red-only transform (content flushed before the state
 * flip would otherwise keep its old colors). */
static void night_mode_changed(bool night)
{
    (void)night;
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        lv_obj_invalidate(lv_screen_active());
        esp_lv_adapter_unlock();
    }
}

/* Full-screen redraw (e.g. after waking from sleep, when co5300_blank() left
 * the GRAM black and the adapter's SPI path does not auto-refresh on resume). */
void lvgl_force_redraw(void)
{
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        lv_obj_invalidate(lv_screen_active());
        esp_lv_adapter_unlock();
    }
}

static void lvgl_build_boot_screen(void)
{
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), 0);

    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "UWatch");
    lv_obj_set_style_text_font(title, s_font_time, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -70);

    lv_obj_t *sub = lv_label_create(lv_screen_active());
    lv_label_set_text(sub, "LILYGO T-Watch Ultra");
    lv_obj_set_style_text_font(sub, s_font_small, 0);
    lv_obj_set_style_text_color(sub, lv_color_hex(0x888888), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, -10);

    char text[64];
    lv_obj_t *ver = lv_label_create(lv_screen_active());
    snprintf(text, sizeof(text), "v%s", esp_app_get_description()->version);
    lv_label_set_text(ver, text);
    lv_obj_set_style_text_font(ver, s_font_small, 0);
    lv_obj_set_style_text_color(ver, lv_color_hex(0x666666), 0);
    lv_obj_align(ver, LV_ALIGN_CENTER, 0, 40);

    lv_obj_t *hash = lv_label_create(lv_screen_active());
    snprintf(text, sizeof(text), "git %s", UWATCH_GIT_HASH);
    lv_label_set_text(hash, text);
    lv_obj_set_style_text_font(hash, s_font_small, 0);
    lv_obj_set_style_text_color(hash, lv_color_hex(0x555555), 0);
    lv_obj_align(hash, LV_ALIGN_CENTER, 0, 80);
}

static void watch_face_update(lv_timer_t *timer)
{
    (void)timer;

    /* Read the wall-clock time from the RTC (PCF85063A), not the ESP32 system
     * clock, so the display never drifts. */
    pcf85063a_time_t t;
    if (pcf85063a_get_time(twatch_rtc_dev, &t) != ESP_OK) {
        return;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "%02d:%02d", t.hour, t.min);
    lv_label_set_text(s_time_label, buf);

    snprintf(buf, sizeof(buf), "%02d", t.sec);
    lv_label_set_text(s_sec_label, buf);

    static const char *wday[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };
    static const char *mon[] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                 "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };
    int wd = (t.weekday >= 1 && t.weekday <= 7) ? t.weekday - 1 : 0;
    snprintf(buf, sizeof(buf), "%s  %02d %s %d",
             wday[wd], t.day, mon[t.month - 1], t.year);
    lv_label_set_text(s_date_label, buf);

    uint8_t pct = 0;
    if (axp2101_get_battery_pct(twatch_pmu_dev, &pct) == ESP_OK && pct <= 100) {
        snprintf(buf, sizeof(buf), "%u%%", pct);
        lv_label_set_text(s_batt_label, buf);
        lv_obj_set_width(s_batt_fill, (lv_coord_t)(140 * pct / 100));
    }
}

static void lvgl_build_watch_face(void)
{
    s_watch_screen = lv_screen_active();
    lv_obj_set_style_bg_color(s_watch_screen, lv_color_hex(0x000000), 0);

    s_date_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_date_label, "");
    lv_obj_set_style_text_font(s_date_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, -110);

    s_time_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_time_label, "--:--");
    lv_obj_set_style_text_font(s_time_label, s_font_time, 0);
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, -20);

    s_sec_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_sec_label, "--");
    lv_obj_set_style_text_font(s_sec_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_sec_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_sec_label, LV_ALIGN_CENTER, 0, 70);

    /* Battery bar. */
    lv_obj_t *bar = lv_obj_create(lv_screen_active());
    lv_obj_set_size(bar, 140, 12);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 160);
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
    lv_obj_align(s_batt_label, LV_ALIGN_CENTER, 0, 200);

    watch_face_update(NULL);
    lv_timer_create(watch_face_update, 1000, NULL);

    /* Menu inactivity timeout (runs forever; no-op on the watch face). */
    lv_timer_create(menu_timeout_cb, 500, NULL);
}

/* Show the boot screen for a few seconds, then the watch face. */
static void boot_to_watch_face(lv_timer_t *timer)
{
    lv_timer_delete(timer);
    lv_obj_clean(lv_screen_active());
    lvgl_build_watch_face();
}

/* ---- Power management screen ---- */

static void power_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_pw_batt_label) {
        return;
    }

    char buf[64];
    uint8_t pct = 0;
    uint16_t mv = 0;
    axp2101_get_battery_pct(twatch_pmu_dev, &pct);
    axp2101_get_battery_mv(twatch_pmu_dev, &mv);
    snprintf(buf, sizeof(buf), "Battery  %u%%  %.3f V", pct, mv / 1000.0f);
    lv_label_set_text(s_pw_batt_label, buf);

    axp2101_charge_state_t chg;
    if (axp2101_get_charge_status(twatch_pmu_dev, &chg) == ESP_OK) {
        bool en = false;
        axp2101_is_charge_enabled(twatch_pmu_dev, &en);
        const char *s;
        switch (chg) {
        case AXP2101_CHG_TRI: s = "Trickle"; break;
        case AXP2101_CHG_PRE: s = "Pre-charge"; break;
        case AXP2101_CHG_CC:  s = "Charging CC"; break;
        case AXP2101_CHG_CV:  s = "Charging CV"; break;
        case AXP2101_CHG_DONE: s = "Charged"; break;
        default:              s = "Not charging"; break;
        }
        uint16_t ma = 0;
        axp2101_get_charge_current_ma(twatch_pmu_dev, &ma);
        snprintf(buf, sizeof(buf), "%s%s  %umA", s, en ? "" : " (disabled)", ma);
        lv_label_set_text(s_pw_chg_label, buf);
    }

    int16_t tc = 0;
    if (axp2101_get_battery_temp(twatch_pmu_dev, &tc) == ESP_OK) {
        snprintf(buf, sizeof(buf), "Battery temp  %d.%d C", tc / 10, abs(tc % 10));
    } else {
        snprintf(buf, sizeof(buf), "Battery temp  -- C");
    }
    lv_label_set_text(s_pw_temp_label, buf);

    /* Keep switch/current buttons reflecting hardware state. */
    bool en = false;
    axp2101_is_charge_enabled(twatch_pmu_dev, &en);
    if (lv_obj_has_state(s_pw_chg_switch, LV_STATE_CHECKED) != en) {
        if (en) {
            lv_obj_add_state(s_pw_chg_switch, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(s_pw_chg_switch, LV_STATE_CHECKED);
        }
    }
    uint16_t ma = 0;
    if (axp2101_get_charge_current_ma(twatch_pmu_dev, &ma) == ESP_OK) {
        bool cur100 = (ma <= 150);
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
    s_power_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_power_screen, lv_color_hex(0x0A1030), 0);

    lv_obj_t *title = lv_label_create(s_power_screen);
    lv_label_set_text(title, "Power Management");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

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

    /* Hint. */
    lv_obj_t *hint = lv_label_create(s_power_screen);
    lv_label_set_text(hint, "swipe down to go back");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -10);

    lv_timer_create(power_screen_update, 1000, NULL);
}

/* ---- BHI260AP status screen ---- */

static const char *activity_name(uint8_t activity)
{
    switch (activity) {
    case BHI260AP_ACTIVITY_STILL: return "still";
    case BHI260AP_ACTIVITY_WALKING: return "walking";
    case BHI260AP_ACTIVITY_RUNNING: return "running";
    case BHI260AP_ACTIVITY_ON_BICYCLE: return "cycling";
    case BHI260AP_ACTIVITY_IN_VEHICLE: return "in vehicle";
    case BHI260AP_ACTIVITY_TILTING: return "tilting";
    default: return "unknown";
    }
}

static void bhi_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_bhi_status_label) {
        return;
    }
    char buf[64];

    bool ready = false;
    uint32_t steps = 0;
    bhi260ap_get_status(&ready, &steps);
    lv_label_set_text(s_bhi_status_label, ready ? "BHI260AP: ready" : "BHI260AP: not ready");
    snprintf(buf, sizeof(buf), "Steps: %lu", (unsigned long)steps);
    lv_label_set_text(s_bhi_steps_label, buf);

    int16_t ax = 0, ay = 0, az = 0;
    bhi260ap_get_accel(&ax, &ay, &az);
    snprintf(buf, sizeof(buf), "Acc (mg): %d %d %d", ax, ay, az);
    lv_label_set_text(s_bhi_accel_label, buf);

    int16_t gx = 0, gy = 0, gz = 0;
    bhi260ap_get_gyro(&gx, &gy, &gz);
    snprintf(buf, sizeof(buf), "Gyro (dps): %d %d %d", gx, gy, gz);
    lv_label_set_text(s_bhi_gyro_label, buf);

    int16_t hd = 0, pt = 0, rl = 0;
    bhi260ap_get_orientation(&hd, &pt, &rl);
    snprintf(buf, sizeof(buf), "Ori (deg): %d %d %d", hd, pt, rl);
    lv_label_set_text(s_bhi_ori_label, buf);

    int16_t rx = 0, ry = 0, rz = 0, rw = 0;
    uint16_t racc = 0;
    bhi260ap_get_rotation(&rx, &ry, &rz, &rw, &racc);
    snprintf(buf, sizeof(buf), "RV: %d %d %d %d acc%d", rx, ry, rz, rw, (int)racc);
    lv_label_set_text(s_bhi_rv_label, buf);

    uint8_t activity = BHI260AP_ACTIVITY_UNKNOWN;
    bhi260ap_get_activity(&activity);
    snprintf(buf, sizeof(buf), "Activity: %s", activity_name(activity));
    lv_label_set_text(s_bhi_activity_label, buf);

    /* Keep the last gesture shown until a new one fires (otherwise the text
     * would clear on the next 1 s refresh). */
    bool tilt = false, wake = false, glance = false, pickup = false, tdet = false;
    if (bhi260ap_consume_gestures(&tilt, &wake, &glance, &pickup, &tdet) == ESP_OK) {
        snprintf(buf, sizeof(buf), "Gesture: %s%s%s%s%s",
                 tilt ? "wristtilt " : "", wake ? "wake " : "", glance ? "glance " : "",
                 pickup ? "pickup " : "", tdet ? "tiltdet " : "");
        if (strcmp(buf, "Gesture: ") != 0) {
            lv_label_set_text(s_bhi_gesture_label, buf);
        }
    }
}

static lv_obj_t *bhi_text_row(lv_obj_t *parent, const char *text, lv_obj_t **label)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, s_font_small, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
    *label = l;
    return l;
}

static void lvgl_build_bhi_screen(void)
{
    s_bhi_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_bhi_screen, lv_color_hex(0x002030), 0);

    lv_obj_t *title = lv_label_create(s_bhi_screen);
    lv_label_set_text(title, "BHI260AP");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    /* All values as plain text rows. */
    lv_obj_t *l;
    l = bhi_text_row(s_bhi_screen, "", &s_bhi_status_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 36);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_steps_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 66);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_accel_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 96);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_gyro_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 126);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_ori_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 156);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_rv_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 186);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_activity_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 216);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_gesture_label);
    lv_obj_set_style_text_color(l, lv_color_hex(0xFFD54D), 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 8, 246);

    lv_obj_t *hint = lv_label_create(s_bhi_screen);
    lv_label_set_text(hint, "swipe right to go back");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);

    bhi_screen_update(NULL);
    lv_timer_create(bhi_screen_update, 1000, NULL);
}

/* ---- GPS screen (skyplot + fix info) ----
 * GNSS is powered on demand: the BLDO1 rail is enabled when this screen is
 * opened and disabled when it is left. The always-on VRTC backup rail keeps
 * the receiver's ephemeris/RTC alive, so each power-up is a warm/hot start. */

#define GPS_SKY_RADIUS    140
#define GPS_SKY_CX        205
#define GPS_SKY_CY        205

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

static void gps_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_gps_status_label) {
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
        snprintf(buf, sizeof(buf), "Fix: %d sats  hAcc +/-%u m",
                 (int)fix.sat_count, (unsigned)fix.hacc_m);
        lv_label_set_text(s_gps_status_label, buf);

        snprintf(buf, sizeof(buf), "%.5f, %.5f  %.0f m",
                 fix.lat, fix.lon, fix.alt_m);
        lv_label_set_text(s_gps_pos_label, buf);

        snprintf(buf, sizeof(buf), "Speed: %u km/h  course %u deg",
                 (unsigned)fix.speed_kmh, (unsigned)fix.course_deg);
        lv_label_set_text(s_gps_speed_label, buf);

        snprintf(buf, sizeof(buf), "%02u:%02u:%02u UTC  (%u in view)",
                 (unsigned)fix.hour, (unsigned)fix.minute, (unsigned)fix.second,
                 (unsigned)fix.sat_in_view);
        lv_label_set_text(s_gps_sats_label, buf);
    } else if (st == M10Q_STATE_ACQUIRING) {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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

    /* Satellite dots. */
    for (int i = 0; i < M10Q_MAX_SATS; i++) {
        if (!s_gps_dots[i]) {
            break;
        }
        if (i < (int)fix.sat_in_view && fix.sats[i].snr_db >= 0) {
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
}

static void lvgl_build_gps_screen(void)
{
    s_gps_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_gps_screen, lv_color_hex(0x102010), 0);

    lv_obj_t *title = lv_label_create(s_gps_screen);
    lv_label_set_text(title, "GPS");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

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
    lv_obj_align(s_gps_status_label, LV_ALIGN_TOP_MID, 0, 356);

    s_gps_pos_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_pos_label, "");
    lv_obj_set_style_text_font(s_gps_pos_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_pos_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_gps_pos_label, LV_ALIGN_TOP_MID, 0, 384);

    s_gps_speed_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_speed_label, "");
    lv_obj_set_style_text_font(s_gps_speed_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_speed_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_gps_speed_label, LV_ALIGN_TOP_MID, 0, 412);

    s_gps_sats_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_sats_label, "");
    lv_obj_set_style_text_font(s_gps_sats_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_sats_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_gps_sats_label, LV_ALIGN_TOP_MID, 0, 440);

    lv_obj_t *hint = lv_label_create(s_gps_screen);
    lv_label_set_text(hint, "swipe left to go back");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -6);

    gps_screen_update(NULL);
    lv_timer_create(gps_screen_update, 1000, NULL);
}

/* Power the GNSS receiver on/off with the GPS screen. */
static void gps_power(bool on)
{
    if (on == s_gps_powered) {
        return;
    }
    if (on) {
        s_gps_acq_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        m10q_power(true);
        s_gps_powered = true;
        ESP_LOGI(TAG, "GNSS powered on");
    } else {
        m10q_power(false);
        s_gps_powered = false;
        ESP_LOGI(TAG, "GNSS powered off");
    }
}

/* ---- Swipe navigation (indev-level: fires for every touch) ---- */

static void swipe_event_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_active();
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        s_swipe_start = p;
        s_swipe_active = true;
        s_last_touch_tick = lv_tick_get();
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_RELEASED || !s_swipe_active) {
        return;
    }
    s_swipe_active = false;
    s_last_touch_tick = lv_tick_get();

    int dy = p.y - s_swipe_start.y;
    int dx = p.x - s_swipe_start.x;
    if (abs(dx) < SWIPE_DIST && abs(dy) < SWIPE_DIST) {
        return;
    }
    bool horiz = abs(dx) > abs(dy);
    lv_obj_t *cur = lv_screen_active();

    if (cur == s_watch_screen) {
        /* Away from the clock. */
        if (horiz) {
            if (dx < 0) {        /* left  -> BHI status */
                if (!s_bhi_screen) {
                    lvgl_build_bhi_screen();
                }
                lv_scr_load(s_bhi_screen);
            } else {             /* right -> GPS */
                if (!s_gps_screen) {
                    lvgl_build_gps_screen();
                }
                gps_power(true);
                lv_scr_load(s_gps_screen);
            }
        } else if (dy > 0) {     /* down  -> Power Management */
            if (!s_power_screen) {
                lvgl_build_power_screen();
            }
            lv_scr_load(s_power_screen);
        }
    } else if (cur == s_power_screen) {
        if (!horiz && dy < 0) {  /* up -> clock */
            lvgl_show_watch_face();
        }
    } else if (cur == s_bhi_screen) {
        if (horiz && dx > 0) {   /* right -> clock */
            lvgl_show_watch_face();
        }
    } else if (cur == s_gps_screen) {
        if (horiz && dx < 0) {   /* left -> clock */
            gps_power(false);
            lvgl_show_watch_face();
        }
    }
}

/* ---- Menu inactivity timeout ----
 * Any non-watch-face screen returns to the watch face after MENU_TIMEOUT_MS
 * without a touch. The BHI sensor and GPS screens are exempt: they are meant
 * for longer observation. */
static void menu_timeout_cb(lv_timer_t *timer)
{
    (void)timer;
    lv_obj_t *cur = lv_screen_active();
    if (cur == s_watch_screen || cur == s_bhi_screen || cur == s_gps_screen) {
        return;
    }
    if (lv_tick_get() - s_last_touch_tick >= MENU_TIMEOUT_MS) {
        lvgl_show_watch_face();
    }
}

/* Load the watch face and refresh the clock labels. Full-screen invalidation
 * here is avoided: a 502-row full redraw queues ~11 band flushes in one cycle,
 * which transiently spikes internal DMA heap usage and can fail the SPI flush
 * (ESP_ERR_NO_MEM / screen corruption). Refreshing the labels re-reads the RTC
 * (time may have changed while a menu was open) at low cost. */
static void lvgl_show_watch_face(void)
{
    lv_scr_load(s_watch_screen);
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        watch_face_update(NULL);
        esp_lv_adapter_unlock();
    }
}

/* Sensor task: bring up the BHI260AP (RAM firmware upload + boot) once the
 * assets partition is mounted, then poll the FIFO to stream sensor events.
 *
 * The sensor rail (ALDO4) is power-cycled by auto-sleep, which erases the
 * chip's RAM firmware. After wake the chip answers FIFO reads with empty data
 * (bootloader mode), so FIFO errors don't reliably appear. Instead, detect a
 * dead stream by data staleness: accel samples at 12.5 Hz when alive, so a
 * stale age (> 5 s) means the chip needs re-initialization. */
#define BHI_STALE_MS 5000

static void bhi260_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(50));
    bool initialized = (bhi260ap_init(twatch_imu_dev) == ESP_OK);
    if (!initialized) {
        ESP_LOGW(TAG, "BHI260AP init failed; retrying");
    }
    for (;;) {
        /* While the chip is in AP-suspend (host sleeping), no data flows by
         * design: skip polling and the stale re-init so the wake-up gesture
         * stream stays armed (re-init would leave AP-suspend and flood the
         * wake task with gesture events). */
        if (bhi260ap_is_suspended()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (initialized) {
            if (bhi260ap_process_fifo() != ESP_OK) {
                ESP_LOGW(TAG, "BHI260AP FIFO read failed");
            }
            if (bhi260ap_get_data_age_ms() > BHI_STALE_MS) {
                ESP_LOGW(TAG, "BHI260AP data stale, re-initializing");
                bhi260ap_deinit();
                initialized = false;
            }
        } else if (bhi260ap_init(twatch_imu_dev) == ESP_OK) {
            initialized = true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void mount_assets(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/assets",
        .partition_label = "assets",
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_OK) {
        size_t used = 0, total = 0;
        esp_spiffs_info(conf.partition_label, &total, &used);
        ESP_LOGI(TAG, "assets SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    } else {
        ESP_LOGW(TAG, "assets SPIFFS mount failed: %s", esp_err_to_name(ret));
    }
}

static const lv_font_t *load_font(int size)
{
    return lv_freetype_font_create("/assets/fonts/Roboto-Regular.ttf",
                                   LV_FREETYPE_FONT_RENDER_MODE_BITMAP, size,
                                   LV_FREETYPE_FONT_STYLE_NORMAL);
}

esp_err_t lvgl_app_start(void)
{
    mount_assets();

    const esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    esp_lv_adapter_config_t adapter_cfg_mut = adapter_cfg;
    /* Auto light sleep: pause LVGL after idle, then tickless light sleep. */
    adapter_cfg_mut.auto_sleep.enable = true;
    adapter_cfg_mut.auto_sleep.mode = ESP_LV_ADAPTER_AUTO_SLEEP_MODE_PAUSE;
    adapter_cfg_mut.auto_sleep.idle_timeout_ms = 5000;
    adapter_cfg_mut.auto_sleep.callbacks.on_enter_sleep = power_mgmt_enter_sleep;
    adapter_cfg_mut.auto_sleep.callbacks.on_exit_sleep = power_mgmt_exit_sleep;
    ESP_RETURN_ON_ERROR(esp_lv_adapter_init(&adapter_cfg_mut), TAG, "adapter init");

    /* Draw buffers in internal DMA-capable RAM: PSRAM buffers would need a
     * temp internal-DMA copy per band flush, and rapid redraws exhaust the
     * ~113 KB internal DMA pool (ESP_ERR_NO_MEM -> corrupted screen). A single
     * 48-row band (38 KB) fits internal RAM and DMA reads it directly. */
    esp_lv_adapter_display_config_t display_cfg = ESP_LV_ADAPTER_DISPLAY_SPI_WITHOUT_PSRAM_DEFAULT_CONFIG(
        co5300_get_panel(),
        co5300_get_panel_io(),
        CO5300_RES_X,
        CO5300_RES_Y,
        ESP_LV_ADAPTER_ROTATE_0);   /* rotation not supported for QSPI */
    display_cfg.profile.buffer_height = 48;   /* partial bands; full frame exceeds SPI DMA max */

    lv_display_t *disp = esp_lv_adapter_register_display(&display_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "register display failed");
        return ESP_FAIL;
    }

    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    lv_display_add_event_cb(disp, area_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    /* Red-only night-mode pixel transform (no-op outside night mode). */
    const esp_lv_adapter_draw_bitmap_callbacks_t draw_cbs = {
        .custom_draw_bitmap = night_mode_draw_bitmap,
    };
    esp_lv_adapter_set_draw_bitmap_callbacks(disp, &draw_cbs, NULL);

    ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), TAG, "adapter start");

    /* Power management: DFS + light sleep + wake sources. */
    power_mgmt_init();

    /* Default charge current: 100 mA (gentle for the 1100 mAh cell; 400 mA is
     * the user-selectable maximum on the power screen). */
    axp2101_set_charge_current_ma(twatch_pmu_dev, 100);

    /* Force a full redraw when night mode toggles so the red-only transform
     * reaches every pixel. */
    power_mgmt_register_night_mode_cb(night_mode_changed);

    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        s_font_time  = load_font(96);
        s_font_sec   = load_font(40);
        s_font_small = load_font(28);
        lvgl_build_boot_screen();
        lv_timer_create(boot_to_watch_face, 3000, NULL);
        esp_lv_adapter_unlock();
    }

    /* Touch input (CST9217). */
    esp_lcd_touch_handle_t tp = cst9217_get_handle();
    if (tp) {
        esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, tp);
        uint16_t nx = 0, ny = 0;
        if (cst9217_get_resolution(&nx, &ny) == ESP_OK && nx && ny) {
            touch_cfg.scale.x = (float)CO5300_RES_X / nx;
            touch_cfg.scale.y = (float)CO5300_RES_Y / ny;
        }
        s_touch_indev = esp_lv_adapter_register_touch(&touch_cfg);
        if (s_touch_indev) {
            ESP_LOGI(TAG, "touch registered");
            /* Indev-level swipe detection: fires for every touch regardless of
             * which widget/screen is active. */
            lv_indev_add_event_cb(s_touch_indev, swipe_event_cb, LV_EVENT_PRESSED, NULL);
            lv_indev_add_event_cb(s_touch_indev, swipe_event_cb, LV_EVENT_RELEASED, NULL);
        } else {
            ESP_LOGE(TAG, "touch registration failed");
        }
        /* The touch driver re-enables its GPIO interrupt; re-apply night mode
         * so touch stays disabled (no accidental wake) during night hours. */
        power_mgmt_recheck_night_mode();
    }

    ESP_LOGI(TAG, "LVGL started (watch face)");

    /* BHI260AP sensor task (needs SPIFFS assets, already mounted above). */
    xTaskCreate(bhi260_task, "bhi260", 4096, NULL, 5, NULL);

    return ESP_OK;
}

/* ---- Screenshot dump (debug) ----
 * Captures the active LVGL screen and prints it to the console as base64 of
 * RGB565 (little-endian) pixels, framed for the host decode script:
 *   ==SHOT:<w>x<h>==  <base64>  ==ENDSHOT==
 * Byte order matches the native RGB565 snapshot; the host script byte-swaps
 * for the big-endian panel if needed. Call in LVGL task context (with the
 * adapter lock held). */

/* ---- Screenshot dump (debug) ----
 * Captures the active LVGL screen and streams it over the USB-JTAG console
 * as raw RGB565 (little-endian) pixels, framed for the host decode script:
 *   ==SHOT:<w>x<h>==  <raw pixels>  ==ENDSHOT==
 * Byte order is the native little-endian RGB565 snapshot. Call from any task;
 * the LVGL lock is held only during the snapshot, not during the transfer. */

esp_err_t lvgl_app_dump_screenshot(void)
{
    int w = CO5300_RES_X;
    int h = CO5300_RES_Y;
    size_t px_size = (size_t)w * h * 2;

    /* Snapshot buffer must come from PSRAM: internal RAM (~365 KB) cannot hold
     * a 410x502 RGB565 frame. */
    void *px = heap_caps_aligned_alloc(16, px_size, MALLOC_CAP_SPIRAM);
    if (!px) {
        ESP_LOGE(TAG, "screenshot: px alloc failed (%u B)", (unsigned)px_size);
        return ESP_ERR_NO_MEM;
    }

    /* Capture under the LVGL lock (fast), then release before any transfer. */
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        free(px);
        return ESP_FAIL;
    }
    lv_image_dsc_t dsc;
    lv_result_t res = lv_snapshot_take_to_buf(lv_screen_active(), LV_COLOR_FORMAT_RGB565,
                                              &dsc, px, px_size);
    esp_lv_adapter_unlock();
    if (res != LV_RESULT_OK) {
        ESP_LOGE(TAG, "screenshot: snapshot failed");
        free(px);
        return ESP_FAIL;
    }

    /* Prefer saving to the SD card as PNG; fall back to streaming raw RGB565
     * over the USB-JTAG console when no card is present. */
    if (sd_log_save_screenshot((const uint16_t *)px, w, h) == ESP_OK) {
        free(px);
        return ESP_OK;
    }

    /* Suppress ESP logging while streaming so no other task interleaves text. */
    esp_log_level_t lvl = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_NONE);
    fflush(stdout);

    char header[64];
    snprintf(header, sizeof(header), "==SHOT:%dx%d==\n", w, h);
    fputs(header, stdout);

    /* Raw RGB565 little-endian via stdout (mirrors to USB-JTAG). */
    size_t chunk = 4096;
    const uint8_t *src = px;
    size_t left = px_size;
    while (left > 0) {
        size_t c = (left < chunk) ? left : chunk;
        size_t wr = fwrite(src, 1, c, stdout);
        src += wr;
        left -= wr;
    }
    printf("==ENDSHOT==\n");

    free(px);
    fflush(stdout);
    esp_log_level_set("*", lvl);
    return ESP_OK;
}
