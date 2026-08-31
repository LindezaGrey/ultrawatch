/*
 * activity_screen.c - Activity screen: today's step count + the
 * per-activity duration breakdown (BHI260AP activity recognition:
 * still/walking/running/cycling/in vehicle/tilting/other), read-only.
 * Shared between the firmware and the host sim - see watch_face.c's
 * header comment for the mechanism.
 *
 * Same underlying data as the debug-only BHI screen's activity rows
 * (bhi_screen.c) and the `dailylog` console command, just without the
 * orientation cube/gesture/raw-sensor debug clutter - a proper ring
 * screen a normal swipe reaches, not a debug-command-only one.
 */
#include "screens.h"
#include <stdio.h>
#include "cascadia_fonts.h"
#include "bhi260ap.h"
#include "daily_log.h"

static const lv_font_t *s_font_title = &cascadia_36;
static const lv_font_t *s_font_steps = &cascadia_72;   /* unused elsewhere - see cascadia_fonts.h */
static const lv_font_t *s_font_small = &cascadia_22;

lv_obj_t *s_activity_screen;

static lv_obj_t *s_act_steps_label;
static lv_obj_t *s_act_row_label[DAILY_ACT_COUNT];

/* Same compact duration formatting as bhi_screen.c's per-activity rows:
 * Xh Ym once over an hour, Xm Ys under that, Xs while still under a
 * minute - so an activity that just started shows real precision instead
 * of "0" until a whole minute has passed. */
static void format_duration(uint32_t s, char *out, size_t out_len)
{
    if (s >= 3600) {
        snprintf(out, out_len, "%uh %um", (unsigned)(s / 3600), (unsigned)((s % 3600) / 60));
    } else if (s >= 60) {
        snprintf(out, out_len, "%um %us", (unsigned)(s / 60), (unsigned)(s % 60));
    } else {
        snprintf(out, out_len, "%us", (unsigned)s);
    }
}

void activity_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_activity_screen) {
        return;
    }

    char buf[32];
    uint32_t steps = 0;
    if (bhi260ap_get_daily_steps(&steps) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)steps);
    } else {
        snprintf(buf, sizeof(buf), "--");
    }
    lv_label_set_text(s_act_steps_label, buf);

    static const char *names[DAILY_ACT_COUNT] = {
        "Still", "Walking", "Running", "Cycling", "In vehicle", "Tilting", "Other" };
    const uint32_t *secs[DAILY_ACT_COUNT];
    if (daily_log_get_activity_seconds(secs) == ESP_OK) {
        for (int i = 0; i < DAILY_ACT_COUNT; i++) {
            char dur[16];
            format_duration(*secs[i], dur, sizeof(dur));
            /* Plain ASCII fixed-width padding, not a table widget - the
             * cascadia_* bitmap fonts only cover 0x20-0x7E (see
             * wifi_screen.c's own note on this), so a plain space-padded
             * name column keeps the duration column aligned without
             * needing LVGL flex/grid on 7 rows. */
            char row[40];
            snprintf(row, sizeof(row), "%-11s%s", names[i], dur);
            lv_label_set_text(s_act_row_label[i], row);
        }
    }
}

void lvgl_build_activity_screen(void)
{
    s_activity_screen = screen_new();
    lv_obj_set_style_bg_color(s_activity_screen, lv_color_hex(0x102010), 0);

    lv_obj_t *title = lv_label_create(s_activity_screen);
    lv_label_set_text(title, "ACTIVITY");
    lv_obj_set_style_text_font(title, s_font_title, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    s_act_steps_label = lv_label_create(s_activity_screen);
    lv_label_set_text(s_act_steps_label, "--");
    lv_obj_set_style_text_font(s_act_steps_label, s_font_steps, 0);
    lv_obj_set_style_text_color(s_act_steps_label, lv_color_hex(0x69F0AE), 0);
    lv_obj_align(s_act_steps_label, LV_ALIGN_TOP_MID, 0, 66);

    lv_obj_t *sub = lv_label_create(s_activity_screen);
    lv_label_set_text(sub, "steps today");
    lv_obj_set_style_text_font(sub, s_font_small, 0);
    lv_obj_set_style_text_color(sub, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, 156);

    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        lv_obj_t *row = lv_label_create(s_activity_screen);
        lv_label_set_text(row, "");
        lv_obj_set_style_text_font(row, s_font_small, 0);
        lv_obj_set_style_text_color(row, lv_color_hex(0xE0E0E0), 0);
        lv_obj_align(row, LV_ALIGN_TOP_LEFT, 55, 210 + i * 30);
        s_act_row_label[i] = row;
    }

    activity_screen_update(NULL);
    lv_timer_create(activity_screen_update, 1000, NULL);
}
