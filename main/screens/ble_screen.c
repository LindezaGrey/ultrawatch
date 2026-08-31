/*
 * ble_screen.c - Bluetooth screen: power switch + one-shot 30s device scan,
 * read only (no pair/connect flow). Shared between the firmware and the
 * host sim - see watch_face.c's header comment for the mechanism.
 */
#include "screens.h"
#include <stdio.h>
#include "cascadia_fonts.h"
#include "ble_scan.h"

static const lv_font_t *s_font_sec   = &cascadia_36;
static const lv_font_t *s_font_small = &cascadia_22;

lv_obj_t *s_ble_screen;

static lv_obj_t *s_ble_pwr_switch;   /* scan on/off (ble_scan_set_enabled()) */
static lv_obj_t *s_ble_empty_label;
static lv_obj_t *s_ble_list_cont;    /* scrollable row container */
static lv_obj_t *s_ble_row_label[BLE_SCAN_MAX_RESULTS];

/* Async on/off switch, same pattern as the WiFi/Mesh screens' power
 * switches. Unlike those, ble_scan_get_enabled() can also flip back to
 * false on its own (the 30s scan window completing) - handled by
 * ble_screen_update() re-syncing the switch every tick, same as the GPS
 * screen already does for its own switch. */
static void ble_pwr_switch_cb(lv_event_t *e)
{
    (void)e;
    if (!s_ble_pwr_switch) {
        return;
    }
    bool on = lv_obj_has_state(s_ble_pwr_switch, LV_STATE_CHECKED);
    ble_scan_set_enabled(on);
}

void ble_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_ble_screen) {
        return;
    }

    if (s_ble_pwr_switch) {
        bool on = ble_scan_get_enabled();
        if (lv_obj_has_state(s_ble_pwr_switch, LV_STATE_CHECKED) != on) {
            if (on) {
                lv_obj_add_state(s_ble_pwr_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(s_ble_pwr_switch, LV_STATE_CHECKED);
            }
        }
    }

    ble_scan_result_t results[BLE_SCAN_MAX_RESULTS];
    size_t n = ble_scan_get_results(results, BLE_SCAN_MAX_RESULTS);

    if (n == 0) {
        const char *msg = ble_scan_low_mem()
            ? "Not enough memory right now"
            : ble_scan_get_enabled()
                ? (ble_scan_is_scanning() ? "Scanning..." : "Starting scan...")
                : "Bluetooth off";
        lv_label_set_text(s_ble_empty_label, msg);
        lv_obj_clear_flag(s_ble_empty_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ble_empty_label, LV_OBJ_FLAG_HIDDEN);
    }

    for (size_t i = 0; i < BLE_SCAN_MAX_RESULTS; i++) {
        if (i >= n) {
            lv_label_set_text(s_ble_row_label[i], "");
            continue;
        }
        char buf[BLE_SCAN_NAME_MAX + 32];
        snprintf(buf, sizeof(buf), "%s  %02x:%02x:%02x:%02x:%02x:%02x  %d dBm",
                 results[i].name[0] ? results[i].name : "(unnamed)",
                 results[i].addr[5], results[i].addr[4], results[i].addr[3],
                 results[i].addr[2], results[i].addr[1], results[i].addr[0],
                 (int)results[i].rssi_dbm);
        lv_label_set_text(s_ble_row_label[i], buf);
    }
}

void lvgl_build_ble_screen(void)
{
    s_ble_screen = screen_new();
    lv_obj_set_style_bg_color(s_ble_screen, lv_color_hex(0x201810), 0);

    lv_obj_t *title = lv_label_create(s_ble_screen);
    lv_label_set_text(title, "BLE");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    lv_obj_t *pwr_lbl = lv_label_create(s_ble_screen);
    lv_label_set_text(pwr_lbl, "Scan");
    lv_obj_set_style_text_font(pwr_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(pwr_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(pwr_lbl, LV_ALIGN_TOP_LEFT, 90, 22);
    s_ble_pwr_switch = lv_switch_create(s_ble_screen);
    lv_obj_set_size(s_ble_pwr_switch, 66, 36);
    lv_obj_align(s_ble_pwr_switch, LV_ALIGN_TOP_RIGHT, -90, 14);
    lv_obj_add_event_cb(s_ble_pwr_switch, ble_pwr_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_ble_empty_label = lv_label_create(s_ble_screen);
    lv_label_set_text(s_ble_empty_label, "Bluetooth off");
    lv_obj_set_style_text_font(s_ble_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_ble_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_ble_empty_label, LV_ALIGN_CENTER, 0, 0);

    s_ble_list_cont = lv_obj_create(s_ble_screen);
    lv_obj_set_size(s_ble_list_cont, 380, 250);
    lv_obj_align(s_ble_list_cont, LV_ALIGN_TOP_MID, 0, 96);
    lv_obj_set_flex_flow(s_ble_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_ble_list_cont, 4, 0);
    lv_obj_set_style_bg_opa(s_ble_list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_ble_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_ble_list_cont, 2, 0);

    for (int i = 0; i < BLE_SCAN_MAX_RESULTS; i++) {
        lv_obj_t *row = lv_label_create(s_ble_list_cont);
        lv_label_set_text(row, "");
        lv_obj_set_style_text_font(row, s_font_small, 0);
        lv_obj_set_style_text_color(row, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_width(row, LV_PCT(100));
        lv_label_set_long_mode(row, LV_LABEL_LONG_CLIP);
        s_ble_row_label[i] = row;
    }

    ble_screen_update(NULL);
    lv_timer_create(ble_screen_update, 1000, NULL);
}
