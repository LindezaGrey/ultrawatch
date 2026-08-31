/*
 * wifi_screen.c - WiFi screen: power switch + scanned-network list, read
 * only (no connect flow). Shared between the firmware and the host sim -
 * see watch_face.c's header comment for the mechanism.
 *
 * The background scan task (wifi_scan.c) runs whenever the power switch is
 * on, independent of whether this screen is open; wifi_screen_update()
 * only pulls a snapshot to display.
 */
#include "screens.h"
#include <stdio.h>
#include "cascadia_fonts.h"
#include "wifi_scan.h"

static const lv_font_t *s_font_sec   = &cascadia_36;
static const lv_font_t *s_font_small = &cascadia_22;

lv_obj_t *s_wifi_screen;

static lv_obj_t *s_wifi_pwr_switch;   /* radio on/off (wifi_scan_set_enabled()) */
static lv_obj_t *s_wifi_empty_label;
static lv_obj_t *s_wifi_list_cont;    /* scrollable row container */
static lv_obj_t *s_wifi_row_label[WIFI_SCAN_MAX_RESULTS];

/* Async on/off switch, same pattern as the GPS/Mesh screens' power
 * switches (see gps_screen.c's gps_pwr_switch_cb() header comment): the
 * callback only sets the target flag, wifi_scan_task applies it and
 * wifi_screen_update() reflects the result back on its own next tick. */
static void wifi_pwr_switch_cb(lv_event_t *e)
{
    (void)e;
    if (!s_wifi_pwr_switch) {
        return;
    }
    bool on = lv_obj_has_state(s_wifi_pwr_switch, LV_STATE_CHECKED);
    wifi_scan_set_enabled(on);
}

void wifi_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_wifi_screen) {
        return;
    }

    if (s_wifi_pwr_switch) {
        bool on = wifi_scan_get_enabled();
        if (lv_obj_has_state(s_wifi_pwr_switch, LV_STATE_CHECKED) != on) {
            if (on) {
                lv_obj_add_state(s_wifi_pwr_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(s_wifi_pwr_switch, LV_STATE_CHECKED);
            }
        }
    }

    wifi_scan_result_t results[WIFI_SCAN_MAX_RESULTS];
    size_t n = wifi_scan_get_enabled() ? wifi_scan_get_results(results, WIFI_SCAN_MAX_RESULTS) : 0;

    if (n == 0) {
        const char *msg = wifi_scan_get_enabled()
            ? (wifi_scan_is_scanning() ? "Scanning..." : "No networks found")
            : "WiFi off";
        lv_label_set_text(s_wifi_empty_label, msg);
        lv_obj_clear_flag(s_wifi_empty_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_wifi_empty_label, LV_OBJ_FLAG_HIDDEN);
    }

    for (size_t i = 0; i < WIFI_SCAN_MAX_RESULTS; i++) {
        if (i >= n) {
            lv_label_set_text(s_wifi_row_label[i], "");
            continue;
        }
        /* Plain ASCII, not LV_SYMBOL_* - the custom cascadia_* bitmap fonts
         * only cover 0x20-0x7E (see cascadia_88's generation comment in
         * cascadia_fonts.h), so LVGL's symbol glyphs render as tofu boxes. */
        char buf[WIFI_SCAN_SSID_MAX + 24];
        snprintf(buf, sizeof(buf), "%s  %s  %d dBm",
                 results[i].ssid[0] ? results[i].ssid : "(hidden)",
                 results[i].open ? "open" : "lock",
                 (int)results[i].rssi_dbm);
        lv_label_set_text(s_wifi_row_label[i], buf);
    }
}

void lvgl_build_wifi_screen(void)
{
    s_wifi_screen = screen_new();
    lv_obj_set_style_bg_color(s_wifi_screen, lv_color_hex(0x201810), 0);

    lv_obj_t *title = lv_label_create(s_wifi_screen);
    lv_label_set_text(title, "WIFI");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    /* Power switch - same row/position convention as the GPS/Mesh
     * screens' power switches. */
    lv_obj_t *pwr_lbl = lv_label_create(s_wifi_screen);
    lv_label_set_text(pwr_lbl, "WiFi");
    lv_obj_set_style_text_font(pwr_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(pwr_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(pwr_lbl, LV_ALIGN_TOP_LEFT, 90, 22);
    s_wifi_pwr_switch = lv_switch_create(s_wifi_screen);
    lv_obj_set_size(s_wifi_pwr_switch, 66, 36);
    lv_obj_align(s_wifi_pwr_switch, LV_ALIGN_TOP_RIGHT, -90, 14);
    lv_obj_add_event_cb(s_wifi_pwr_switch, wifi_pwr_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_wifi_empty_label = lv_label_create(s_wifi_screen);
    lv_label_set_text(s_wifi_empty_label, "WiFi off");
    lv_obj_set_style_text_font(s_wifi_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_wifi_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_wifi_empty_label, LV_ALIGN_CENTER, 0, 0);

    s_wifi_list_cont = lv_obj_create(s_wifi_screen);
    lv_obj_set_size(s_wifi_list_cont, 380, 250);
    lv_obj_align(s_wifi_list_cont, LV_ALIGN_TOP_MID, 0, 96);
    lv_obj_set_flex_flow(s_wifi_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_wifi_list_cont, 4, 0);
    lv_obj_set_style_bg_opa(s_wifi_list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_wifi_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_wifi_list_cont, 2, 0);

    for (int i = 0; i < WIFI_SCAN_MAX_RESULTS; i++) {
        lv_obj_t *row = lv_label_create(s_wifi_list_cont);
        lv_label_set_text(row, "");
        lv_obj_set_style_text_font(row, s_font_small, 0);
        lv_obj_set_style_text_color(row, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_width(row, LV_PCT(100));
        lv_label_set_long_mode(row, LV_LABEL_LONG_CLIP);
        s_wifi_row_label[i] = row;
    }

    wifi_screen_update(NULL);
    lv_timer_create(wifi_screen_update, 1000, NULL);
}
