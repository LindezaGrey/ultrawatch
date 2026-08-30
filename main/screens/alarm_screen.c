/*
 * alarm_screen.c - Alarms/Timers list screen (docs/application.md section
 * 8) + its 2 local sub-screens: create/edit one alarm, start a timer.
 * Shared between the firmware and the host sim - see watch_face.c's
 * header comment for the mechanism.
 *
 * The ringing screen (lvgl_build_ring_screen) lives in ring_screen.c,
 * same split the sim already used. alarm_ring_cb() (main/lvgl_app.c) -
 * the real ring start/stop callback, runs on the alarm ring task and
 * needs esp_lv_adapter_lock() - stays firmware-only and calls into
 * lvgl_build_ring_screen() there, then sets the ring screen's own
 * title/time/snooze-button directly for each new ring (see screens.h).
 */
#include "screens.h"
#include <stdio.h>
#include <string.h>
#include "cascadia_fonts.h"
#include "alarm.h"
#include "cd_timer.h"

static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_sec   = &cascadia_36;

lv_obj_t *s_alarm_screen;               /* the LIST screen (nav-ring slot) */
static lv_obj_t *s_alarm_row[ALARM_MAX_COUNT]; /* one row per slot, hidden if !in_use */
static lv_obj_t *s_alarm_row_time_label[ALARM_MAX_COUNT];
static lv_obj_t *s_alarm_row_switch[ALARM_MAX_COUNT];
static lv_obj_t *s_alarm_timer_label;          /* active countdown, hidden if none running */
static lv_obj_t *s_alarm_timer_cancel_btn;

lv_obj_t *s_alarm_edit_screen;          /* create/edit one alarm */
static lv_obj_t *s_alarm_edit_time_label;
static lv_obj_t *s_alarm_edit_mode_beep;
static lv_obj_t *s_alarm_edit_mode_vib;
static lv_obj_t *s_alarm_edit_mode_both;
static lv_obj_t *s_alarm_edit_wday_btn[7];     /* Sun..Sat */
static lv_obj_t *s_alarm_edit_delete_btn;      /* hidden while creating a new entry */
static int s_alarm_edit_idx = -1;              /* -1 = creating new, >=0 = editing that slot */
static uint8_t s_alarm_edit_hour, s_alarm_edit_min, s_alarm_edit_mode, s_alarm_edit_wmask;

lv_obj_t *s_timer_screen;               /* countdown-timer duration presets */

/* "Daily"/"Mon-Fri"/comma list, mirrors debug_cmds.c's print_weekday_mask()
 * but into a buffer instead of stdout. */
static void weekday_summary(uint8_t wmask, char *out, size_t outlen)
{
    static const char *names[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    if (wmask == ALARM_WEEKDAY_ALL) {
        snprintf(out, outlen, "Daily");
        return;
    }
    if (wmask == 0x3E) {   /* Mon..Fri */
        snprintf(out, outlen, "Mon-Fri");
        return;
    }
    out[0] = '\0';
    bool first = true;
    for (int i = 0; i < 7; i++) {
        if (wmask & (1u << i)) {
            size_t len = strlen(out);
            snprintf(out + len, outlen - len, "%s%s", first ? "" : ",", names[i]);
            first = false;
        }
    }
}

/* Re-populates the 8 fixed alarm rows and the active-timer display. Called
 * on build, from the list screen's periodic refresh timer, and whenever
 * an edit/add/remove/timer-start returns to this screen. */
void alarm_list_refresh(void)
{
    alarm_entry_t list[ALARM_MAX_COUNT];
    alarm_get_all(list, ALARM_MAX_COUNT);
    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        if (!list[i].in_use) {
            lv_obj_add_flag(s_alarm_row[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(s_alarm_row[i], LV_OBJ_FLAG_HIDDEN);
        char wbuf[16];
        weekday_summary(list[i].weekday_mask, wbuf, sizeof(wbuf));
        char buf[32];
        snprintf(buf, sizeof(buf), "%02u:%02u  %s", (unsigned)list[i].hour,
                 (unsigned)list[i].min, wbuf);
        lv_label_set_text(s_alarm_row_time_label[i], buf);
        if (list[i].enabled) {
            lv_obj_add_state(s_alarm_row_switch[i], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(s_alarm_row_switch[i], LV_STATE_CHECKED);
        }
    }

    if (cdtimer_is_active()) {
        uint32_t s = cdtimer_remaining_seconds();
        char buf[16];
        snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
        lv_label_set_text(s_alarm_timer_label, buf);
        lv_obj_clear_flag(s_alarm_timer_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_alarm_timer_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_alarm_timer_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_alarm_timer_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void alarm_timer_cancel_btn_cb(lv_event_t *e)
{
    (void)e;
    cdtimer_cancel();
    alarm_list_refresh();
}

static void alarm_row_switch_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    bool en = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    alarm_set_enabled(idx, en);
}

/* Tapping a row's label area (not its switch - LVGL only delivers CLICKED
 * to the row container for events that land on the container itself, not
 * on the child switch) opens that entry in the edit screen. */
static void alarm_row_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    alarm_entry_t list[ALARM_MAX_COUNT];
    alarm_get_all(list, ALARM_MAX_COUNT);
    if (!list[idx].in_use) {
        return;
    }
    s_alarm_edit_idx = idx;
    s_alarm_edit_hour = list[idx].hour;
    s_alarm_edit_min = list[idx].min;
    s_alarm_edit_mode = list[idx].ring_mode;
    s_alarm_edit_wmask = list[idx].weekday_mask;
    if (!s_alarm_edit_screen) {
        lvgl_build_alarm_edit_screen();
    }
    lv_obj_clear_flag(s_alarm_edit_delete_btn, LV_OBJ_FLAG_HIDDEN);
    lv_scr_load(s_alarm_edit_screen);
}

static void alarm_add_btn_cb(lv_event_t *e)
{
    (void)e;
    s_alarm_edit_idx = -1;
    s_alarm_edit_hour = 7;
    s_alarm_edit_min = 0;
    s_alarm_edit_mode = ALARM_RING_BEEP;
    s_alarm_edit_wmask = ALARM_WEEKDAY_ALL;
    if (!s_alarm_edit_screen) {
        lvgl_build_alarm_edit_screen();
    }
    lv_obj_add_flag(s_alarm_edit_delete_btn, LV_OBJ_FLAG_HIDDEN);   /* nothing to delete yet */
    lv_scr_load(s_alarm_edit_screen);
}

static void timer_add_btn_cb(lv_event_t *e)
{
    (void)e;
    if (!s_timer_screen) {
        lvgl_build_timer_screen();
    }
    lv_scr_load(s_timer_screen);
}

/* The list screen's other controls (switches, +Alarm/+Timer, edit/timer
 * sub-screens) all commit immediately, so - like the old single-alarm
 * screen - this timer exists mainly to keep the active countdown/row
 * states live while the screen is open. */
static void alarm_screen_refresh_timer(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_alarm_screen) {
        return;
    }
    alarm_list_refresh();
}

void lvgl_build_alarm_screen(void)
{
    s_alarm_screen = screen_new();
    lv_obj_set_style_bg_color(s_alarm_screen, lv_color_hex(0x201020), 0);

    /* High-DPI sizing (AGENT.md "Display density & UI sizing"): title and
     * the +Alarm/+Timer buttons promote to cascadia_36; the timer-cancel
     * button and list-row labels stay at cascadia_22 - both sit next to a
     * fixed-width sibling (the 90px Cancel button, the switch on each
     * row) and weekday-summary text is variable-length enough ("Mon-Fri"
     * vs "Daily" etc.) that cascadia_36 risks overflowing into it. Rows
     * and their switches still get a bigger touch target. */
    lv_obj_t *title = lv_label_create(s_alarm_screen);
    lv_label_set_text(title, "ALARMS");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    /* Active countdown, hidden unless a timer is running (alarm_list_refresh()). */
    s_alarm_timer_label = lv_label_create(s_alarm_screen);
    lv_label_set_text(s_alarm_timer_label, "");
    lv_obj_set_style_text_font(s_alarm_timer_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_alarm_timer_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_alarm_timer_label, LV_ALIGN_TOP_LEFT, 30, 82);
    lv_obj_add_flag(s_alarm_timer_label, LV_OBJ_FLAG_HIDDEN);

    s_alarm_timer_cancel_btn = lv_button_create(s_alarm_screen);
    lv_obj_set_size(s_alarm_timer_cancel_btn, 90, 44);
    lv_obj_align(s_alarm_timer_cancel_btn, LV_ALIGN_TOP_RIGHT, -30, 78);
    lv_obj_t *ctl = lv_label_create(s_alarm_timer_cancel_btn);
    lv_label_set_text(ctl, "Cancel");
    lv_obj_set_style_text_font(ctl, s_font_small, 0);
    lv_obj_center(ctl);
    lv_obj_add_event_cb(s_alarm_timer_cancel_btn, alarm_timer_cancel_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_alarm_timer_cancel_btn, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *add_alarm = lv_button_create(s_alarm_screen);
    lv_obj_set_size(add_alarm, 170, 44);
    lv_obj_align(add_alarm, LV_ALIGN_TOP_LEFT, 20, 135);
    lv_obj_t *aal = lv_label_create(add_alarm);
    lv_label_set_text(aal, "+ Alarm");
    lv_obj_set_style_text_font(aal, s_font_sec, 0);
    lv_obj_center(aal);
    lv_obj_add_event_cb(add_alarm, alarm_add_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *add_timer = lv_button_create(s_alarm_screen);
    lv_obj_set_size(add_timer, 170, 44);
    lv_obj_align(add_timer, LV_ALIGN_TOP_RIGHT, -20, 135);
    lv_obj_t *atl = lv_label_create(add_timer);
    lv_label_set_text(atl, "+ Timer");
    lv_obj_set_style_text_font(atl, s_font_sec, 0);
    lv_obj_center(atl);
    lv_obj_add_event_cb(add_timer, timer_add_btn_cb, LV_EVENT_CLICKED, NULL);

    /* Scrollable list, up to ALARM_MAX_COUNT rows (fixed-count, hidden/shown
     * per slot - same pattern as the mesh screen's message rows). */
    lv_obj_t *list_cont = lv_obj_create(s_alarm_screen);
    lv_obj_set_size(list_cont, 370, 280);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 190);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 6, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 4, 0);

    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        lv_obj_t *row = lv_obj_create(list_cont);
        lv_obj_set_size(row, LV_PCT(100), 48);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x301830), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_set_style_pad_all(row, 8, 0);
        lv_obj_add_event_cb(row, alarm_row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        s_alarm_row[i] = row;

        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_small, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
        s_alarm_row_time_label[i] = l;

        lv_obj_t *sw = lv_switch_create(row);
        lv_obj_set_size(sw, 66, 36);
        lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_add_event_cb(sw, alarm_row_switch_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);
        s_alarm_row_switch[i] = sw;

        lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);   /* shown by alarm_list_refresh() if in_use */
    }

    lv_obj_t *hint = lv_label_create(s_alarm_screen);
    lv_label_set_text(hint, "tap a row to edit");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -6);

    alarm_list_refresh();
    lv_timer_create(alarm_screen_refresh_timer, 1000, NULL);
}

/* ---- Alarm create/edit sub-screen (local, not in the nav ring) ---- */

static void alarm_edit_screen_refresh(void)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)s_alarm_edit_hour, (unsigned)s_alarm_edit_min);
    lv_label_set_text(s_alarm_edit_time_label, buf);

    lv_obj_clear_state(s_alarm_edit_mode_beep, LV_STATE_CHECKED);
    lv_obj_clear_state(s_alarm_edit_mode_vib, LV_STATE_CHECKED);
    lv_obj_clear_state(s_alarm_edit_mode_both, LV_STATE_CHECKED);
    lv_obj_add_state(s_alarm_edit_mode == ALARM_RING_BEEP ? s_alarm_edit_mode_beep :
                     s_alarm_edit_mode == ALARM_RING_VIB ? s_alarm_edit_mode_vib : s_alarm_edit_mode_both,
                     LV_STATE_CHECKED);

    for (int i = 0; i < 7; i++) {
        if (s_alarm_edit_wmask & (1u << i)) {
            lv_obj_add_state(s_alarm_edit_wday_btn[i], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(s_alarm_edit_wday_btn[i], LV_STATE_CHECKED);
        }
    }
}

static void alarm_edit_hour_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit_hour = (uint8_t)((s_alarm_edit_hour + 24 + delta) % 24);
    alarm_edit_screen_refresh();
}

/* Steps by 30 min (not 1) - the doc specifies full/half-hour presets, not
 * freetext/digit entry, and a 30 min step is a tap-based preset in
 * everything but name while reusing the existing +/- stepper widget instead
 * of a new scrollable time-list. */
static void alarm_edit_min_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit_min = (uint8_t)((s_alarm_edit_min + 60 + delta) % 60);
    alarm_edit_screen_refresh();
}

static void alarm_edit_mode_btn_cb(lv_event_t *e)
{
    s_alarm_edit_mode = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    alarm_edit_screen_refresh();
}

static void alarm_edit_wday_btn_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit_wmask ^= (uint8_t)(1u << i);
}

static void alarm_edit_save_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t wmask = s_alarm_edit_wmask ? s_alarm_edit_wmask : ALARM_WEEKDAY_ALL;
    if (s_alarm_edit_idx < 0) {
        alarm_add(s_alarm_edit_hour, s_alarm_edit_min, s_alarm_edit_mode, wmask);
    } else {
        alarm_update(s_alarm_edit_idx, s_alarm_edit_hour, s_alarm_edit_min, s_alarm_edit_mode, wmask);
    }
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void alarm_edit_delete_btn_cb(lv_event_t *e)
{
    (void)e;
    if (s_alarm_edit_idx >= 0) {
        alarm_remove(s_alarm_edit_idx);
    }
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void alarm_edit_back_btn_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

void lvgl_build_alarm_edit_screen(void)
{
    s_alarm_edit_screen = screen_new();
    lv_obj_set_style_bg_color(s_alarm_edit_screen, lv_color_hex(0x201020), 0);

    /* Back button uses the same rounded-corner-safe position/size as
     * Settings (AGENT.md "Display density & UI sizing", commit cfb7307).
     * Everything else on this screen (time/stepper/weekday/mode/save
     * buttons) is already packed with no vertical slack - see the
     * comment at the weekday row below for what could still be sized up
     * safely without cascading the whole layout. */
    lv_obj_t *back = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, alarm_edit_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_alarm_edit_screen);
    lv_label_set_text(title, "ALARM");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_alarm_edit_time_label = lv_label_create(s_alarm_edit_screen);
    lv_obj_set_style_text_font(s_alarm_edit_time_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_alarm_edit_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_alarm_edit_time_label, LV_ALIGN_TOP_MID, 0, 55);

    lv_obj_t *h_minus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(h_minus, 150, 64);
    lv_obj_align(h_minus, LV_ALIGN_TOP_LEFT, 20, 110);
    lv_obj_t *hml = lv_label_create(h_minus);
    lv_label_set_text(hml, "H-");
    lv_obj_set_style_text_font(hml, s_font_small, 0);
    lv_obj_center(hml);
    lv_obj_add_event_cb(h_minus, alarm_edit_hour_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *h_plus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(h_plus, 150, 64);
    lv_obj_align(h_plus, LV_ALIGN_TOP_RIGHT, -20, 110);
    lv_obj_t *hpl = lv_label_create(h_plus);
    lv_label_set_text(hpl, "H+");
    lv_obj_set_style_text_font(hpl, s_font_small, 0);
    lv_obj_center(hpl);
    lv_obj_add_event_cb(h_plus, alarm_edit_hour_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    lv_obj_t *m_minus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(m_minus, 150, 64);
    lv_obj_align(m_minus, LV_ALIGN_TOP_LEFT, 20, 185);
    lv_obj_t *mml = lv_label_create(m_minus);
    lv_label_set_text(mml, "M-30");
    lv_obj_set_style_text_font(mml, s_font_small, 0);
    lv_obj_center(mml);
    lv_obj_add_event_cb(m_minus, alarm_edit_min_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-30);

    lv_obj_t *m_plus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(m_plus, 150, 64);
    lv_obj_align(m_plus, LV_ALIGN_TOP_RIGHT, -20, 185);
    lv_obj_t *mpl = lv_label_create(m_plus);
    lv_label_set_text(mpl, "M+30");
    lv_obj_set_style_text_font(mpl, s_font_small, 0);
    lv_obj_center(mpl);
    lv_obj_add_event_cb(m_plus, alarm_edit_min_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)30);

    lv_obj_t *wday_lbl = lv_label_create(s_alarm_edit_screen);
    lv_label_set_text(wday_lbl, "Repeat");
    lv_obj_set_style_text_font(wday_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(wday_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(wday_lbl, LV_ALIGN_TOP_LEFT, 20, 260);

    /* 2-char labels fit comfortably at cascadia_22 (fixed box size unchanged -
     * no cascading effect on the tightly-packed rows below/above). */
    static const char *wday_names[7] = { "Su", "Mo", "Tu", "We", "Th", "Fr", "Sa" };
    for (int i = 0; i < 7; i++) {
        lv_obj_t *wb = lv_button_create(s_alarm_edit_screen);
        lv_obj_set_size(wb, 48, 40);
        lv_obj_align(wb, LV_ALIGN_TOP_LEFT, 20 + i * 52, 285);
        lv_obj_t *wl = lv_label_create(wb);
        lv_label_set_text(wl, wday_names[i]);
        lv_obj_set_style_text_font(wl, s_font_small, 0);
        lv_obj_center(wl);
        lv_obj_add_flag(wb, LV_OBJ_FLAG_CHECKABLE);
        lv_obj_add_event_cb(wb, alarm_edit_wday_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        s_alarm_edit_wday_btn[i] = wb;
    }

    lv_obj_t *mode_lbl = lv_label_create(s_alarm_edit_screen);
    lv_label_set_text(mode_lbl, "Ring");
    lv_obj_set_style_text_font(mode_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(mode_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(mode_lbl, LV_ALIGN_TOP_LEFT, 20, 345);

    s_alarm_edit_mode_beep = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_mode_beep, 105, 56);
    lv_obj_align(s_alarm_edit_mode_beep, LV_ALIGN_TOP_LEFT, 20, 370);
    lv_obj_t *mb = lv_label_create(s_alarm_edit_mode_beep);
    lv_label_set_text(mb, "Beep");
    lv_obj_set_style_text_font(mb, s_font_small, 0);
    lv_obj_center(mb);
    lv_obj_add_flag(s_alarm_edit_mode_beep, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_edit_mode_beep, alarm_edit_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_BEEP);

    s_alarm_edit_mode_vib = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_mode_vib, 105, 56);
    lv_obj_align(s_alarm_edit_mode_vib, LV_ALIGN_TOP_MID, 0, 370);
    lv_obj_t *mv = lv_label_create(s_alarm_edit_mode_vib);
    lv_label_set_text(mv, "Vib");
    lv_obj_set_style_text_font(mv, s_font_small, 0);
    lv_obj_center(mv);
    lv_obj_add_flag(s_alarm_edit_mode_vib, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_edit_mode_vib, alarm_edit_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_VIB);

    s_alarm_edit_mode_both = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_mode_both, 105, 56);
    lv_obj_align(s_alarm_edit_mode_both, LV_ALIGN_TOP_RIGHT, -20, 370);
    lv_obj_t *mbt = lv_label_create(s_alarm_edit_mode_both);
    lv_label_set_text(mbt, "Both");
    lv_obj_set_style_text_font(mbt, s_font_small, 0);
    lv_obj_center(mbt);
    lv_obj_add_flag(s_alarm_edit_mode_both, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_edit_mode_both, alarm_edit_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_BOTH);

    lv_obj_t *save = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(save, 170, 50);
    lv_obj_align(save, LV_ALIGN_BOTTOM_LEFT, 20, -14);
    lv_obj_t *svl = lv_label_create(save);
    lv_label_set_text(svl, "Save");
    lv_obj_set_style_text_font(svl, s_font_small, 0);
    lv_obj_center(svl);
    lv_obj_add_event_cb(save, alarm_edit_save_btn_cb, LV_EVENT_CLICKED, NULL);

    s_alarm_edit_delete_btn = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_delete_btn, 170, 50);
    lv_obj_align(s_alarm_edit_delete_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -14);
    lv_obj_set_style_bg_color(s_alarm_edit_delete_btn, lv_color_hex(0x802020), 0);
    lv_obj_t *dl = lv_label_create(s_alarm_edit_delete_btn);
    lv_label_set_text(dl, "Delete");
    lv_obj_set_style_text_font(dl, s_font_small, 0);
    lv_obj_center(dl);
    lv_obj_add_event_cb(s_alarm_edit_delete_btn, alarm_edit_delete_btn_cb, LV_EVENT_CLICKED, NULL);

    alarm_edit_screen_refresh();
}

/* ---- Timer-start sub-screen (local, not in the nav ring) ---- */

static const uint16_t s_timer_presets_min[8] = { 1, 5, 10, 15, 30, 60, 90, 120 };

static void timer_preset_btn_cb(lv_event_t *e)
{
    uint16_t minutes = (uint16_t)(uintptr_t)lv_event_get_user_data(e);
    cdtimer_start((uint32_t)minutes * 60);
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void timer_back_btn_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

void lvgl_build_timer_screen(void)
{
    s_timer_screen = screen_new();
    lv_obj_set_style_bg_color(s_timer_screen, lv_color_hex(0x102020), 0);

    /* Back button: same rounded-corner-safe position/size as Settings
     * (AGENT.md "Display density & UI sizing", commit cfb7307). Title
     * stays centered so it doesn't need to move (no horizontal overlap
     * with the button regardless of y); the preset grid shifts down 10px
     * to clear the now-taller button. */
    lv_obj_t *back = lv_button_create(s_timer_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, timer_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_timer_screen);
    lv_label_set_text(title, "TIMER");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    for (int i = 0; i < 8; i++) {
        int col = i % 2;
        int row = i / 2;
        lv_obj_t *btn = lv_button_create(s_timer_screen);
        lv_obj_set_size(btn, 170, 70);
        lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 20 + col * 190, 90 + row * 90);
        lv_obj_t *l = lv_label_create(btn);
        char buf[16];
        snprintf(buf, sizeof(buf), "%u min", s_timer_presets_min[i]);
        lv_label_set_text(l, buf);
        lv_obj_set_style_text_font(l, s_font_sec, 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(btn, timer_preset_btn_cb, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)s_timer_presets_min[i]);
    }
}
