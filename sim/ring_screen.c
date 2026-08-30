/*
 * ring_screen.c - the alarm/timer ringing screen from main/lvgl_app.c
 * (lvgl_build_ring_screen, 2026-08-29 multi-alarm overhaul: source-aware
 * title/snooze-visibility), copied verbatim and run against mock_hw.c
 * instead of real drivers.
 *
 * Reachability, sim-only: in the firmware this screen is only ever shown by
 * alarm_ring_cb(), fired from the real alarm-firing/RTC-interrupt system,
 * which doesn't exist on the host. alarm_ring_cb() itself is NOT ported
 * here. Instead main.c wires 'R' (alarm-sourced preview) and 'T'
 * (timer-sourced preview, snooze hidden) directly to
 * sim_ring_screen_build(alarm_ring_source_t), purely so this screen's
 * layout can be eyeballed in both variants - it does not exercise the real
 * ring/dismiss/snooze state machine (alarm_is_ringing()) the way a real
 * alarm/timer firing would. Dismiss/Snooze still call the real
 * alarm_dismiss()/alarm_snooze() mocks, same as the firmware.
 *
 * Deviations from the firmware original (mechanical only):
 *   - screen_new() copied in here too, same as every non-watch-face screen.
 *   - No esp_lv_adapter_lock()/unlock(): none was present in this range of
 *     the original anyway (that's only in the skipped alarm_ring_cb()).
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_time = &cascadia_72;
static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_sec   = &cascadia_36;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_ring_screen;

static lv_obj_t *s_ring_title_label;
static lv_obj_t *s_ring_time_label;
static lv_obj_t *s_ring_snooze_btn;

static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

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

static void lvgl_build_ring_screen(void)
{
    s_ring_screen = screen_new();
    lv_obj_set_style_bg_color(s_ring_screen, lv_color_hex(0x300000), 0);

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

void sim_ring_screen_build(alarm_ring_source_t source)
{
    if (!s_ring_screen) {
        lvgl_build_ring_screen();
    }
    if (source == ALARM_RING_SOURCE_TIMER) {
        lv_label_set_text(s_ring_title_label, "TIMER");
        lv_label_set_text(s_ring_time_label, "Done");
        lv_obj_add_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_label_set_text(s_ring_title_label, "ALARM");
        /* Stamp the first configured alarm's time (no real "which entry
         * fired" state to query in this preview-only screen). */
        alarm_entry_t list[ALARM_MAX_COUNT];
        alarm_get_all(list, ALARM_MAX_COUNT);
        char buf[8] = "--:--";
        for (int i = 0; i < ALARM_MAX_COUNT; i++) {
            if (list[i].in_use) {
                snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)list[i].hour, (unsigned)list[i].min);
                break;
            }
        }
        lv_label_set_text(s_ring_time_label, buf);
        lv_obj_clear_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
    }
    lv_scr_load(s_ring_screen);
}
