/*
 * ring_screen.c - the alarm/timer ringing screen. Shared between the
 * firmware and the host sim - see watch_face.c's header comment for the
 * mechanism.
 *
 * Reachability: shown only via main/lvgl_app.c's alarm_ring_cb() (the real
 * ring start/stop callback, runs on the alarm ring task, needs
 * esp_lv_adapter_lock() - stays firmware-only) or, in the sim, main.c's
 * 'R'/'T' dev-shortcut keys.
 */
#include "screens.h"
#include <stdio.h>
#include "cascadia_fonts.h"
#include "alarm.h"

static const lv_font_t *s_font_time  = &cascadia_72;
static const lv_font_t *s_font_sec   = &cascadia_36;

lv_obj_t *s_ring_screen;                /* alarm/timer RINGING screen - not a nav-ring screen */
lv_obj_t *s_ring_title_label;           /* "ALARM" or "TIMER" */
lv_obj_t *s_ring_time_label;
lv_obj_t *s_ring_snooze_btn;            /* hidden for a timer-sourced ring (no snooze concept) */

static void alarm_dismiss_btn_cb(lv_event_t *e)
{
    (void)e;
    alarm_dismiss();
}

static void alarm_snooze_btn_cb(lv_event_t *e)
{
    (void)e;
    alarm_snooze();
}

void lvgl_build_ring_screen(void)
{
    s_ring_screen = screen_new();
    lv_obj_set_style_bg_color(s_ring_screen, lv_color_hex(0x300000), 0);

    /* High-DPI sizing (AGENT.md "Display density & UI sizing"): title
     * promotes to cascadia_36 (nudged up 4px to keep clear of the giant
     * cascadia_72 time below it); the Dismiss/Snooze button labels
     * promote too - these buttons are already huge (330x112) with room
     * to spare for the bigger glyphs. */
    s_ring_title_label = lv_label_create(s_ring_screen);
    lv_label_set_text(s_ring_title_label, "ALARM");
    lv_obj_set_style_text_font(s_ring_title_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_ring_title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_ring_title_label, LV_ALIGN_TOP_MID, 0, 14);

    s_ring_time_label = lv_label_create(s_ring_screen);
    lv_obj_set_style_text_font(s_ring_time_label, s_font_time, 0);
    lv_obj_set_style_text_color(s_ring_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_ring_time_label, LV_ALIGN_TOP_MID, 0, 60);

    lv_obj_t *dismiss = lv_button_create(s_ring_screen);
    lv_obj_set_size(dismiss, 330, 112);
    lv_obj_align(dismiss, LV_ALIGN_TOP_MID, 0, 220);
    lv_obj_t *dl = lv_label_create(dismiss);
    lv_label_set_text(dl, "Dismiss");
    lv_obj_set_style_text_font(dl, s_font_sec, 0);
    lv_obj_center(dl);
    /* Fire on touch-down so the first tap acts immediately, regardless of
     * click state or timing. Kept as a touch fallback alongside the
     * physical-button dismiss (main/power_mgmt.c's button callback -> see
     * alarm.c's alarm_button_cb()) - the doc doesn't say touch-dismiss must
     * be removed, only that stopping is button-driven. */
    lv_obj_add_event_cb(dismiss, alarm_dismiss_btn_cb, LV_EVENT_PRESSED, NULL);

    s_ring_snooze_btn = lv_button_create(s_ring_screen);
    lv_obj_set_size(s_ring_snooze_btn, 330, 112);
    lv_obj_align(s_ring_snooze_btn, LV_ALIGN_TOP_MID, 0, 340);
    lv_obj_t *sl = lv_label_create(s_ring_snooze_btn);
    lv_label_set_text(sl, "Snooze 10 min");
    lv_obj_set_style_text_font(sl, s_font_sec, 0);
    lv_obj_center(sl);
    lv_obj_add_event_cb(s_ring_snooze_btn, alarm_snooze_btn_cb, LV_EVENT_PRESSED, NULL);
}
