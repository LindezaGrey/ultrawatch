/*
 * alarm_screen.c - the real alarm-set screen from main/lvgl_app.c, copied
 * verbatim (alarm_commit / alarm_hour_btn_cb / alarm_min_btn_cb /
 * alarm_mode_btn_cb / alarm_en_switch_cb / lvgl_build_alarm_screen,
 * unmodified widget layout and logic) and run against mock_hw.c instead of
 * real drivers.
 *
 * Post-2026-08-27 overhaul: every control commits immediately via
 * alarm_set() (no separate "Set" button - see main/lvgl_app.c's
 * alarm_commit() comment), the Beep/Vib/Both buttons have real gaps between
 * them (were exactly edge-to-edge), and there's a swipe-back hint like every
 * other screen (this one didn't have one before).
 *
 * Deviations from the firmware original (mechanical only):
 *   - screen_new() copied in here too, same as every non-watch-face screen.
 *   - No esp_lv_adapter_lock()/unlock(): none was present in this range of
 *     the original anyway.
 *
 * The ringing screen (lvgl_build_ring_screen, alarm_dismiss_btn_cb,
 * alarm_snooze_btn_cb) lives in ring_screen.c, since it's reached completely
 * differently in the sim (see that file's header comment).
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_sec  = &cascadia_36;
static const lv_font_t *s_font_small = &cascadia_22;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_alarm_screen;

static lv_obj_t *s_alarm_time_label;
static lv_obj_t *s_alarm_en_switch;
static lv_obj_t *s_alarm_mode_beep;
static lv_obj_t *s_alarm_mode_vib;
static lv_obj_t *s_alarm_mode_both;
static alarm_config_t s_alarm_edit;

static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

/* Every control commits immediately via alarm_set() - no separate "Set"
 * confirm step, matching main/lvgl_app.c's post-overhaul alarm screen. See
 * that file's alarm_commit() comment for the reasoning. */
static void alarm_commit(void)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)s_alarm_edit.hour,
             (unsigned)s_alarm_edit.min);
    lv_label_set_text(s_alarm_time_label, buf);
    alarm_set(s_alarm_edit.hour, s_alarm_edit.min, s_alarm_edit.enabled,
              s_alarm_edit.ring_mode);
}

static void alarm_hour_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit.hour = (uint8_t)((s_alarm_edit.hour + 24 + delta) % 24);
    alarm_commit();
}

static void alarm_min_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit.min = (uint8_t)((s_alarm_edit.min + 60 + delta) % 60);
    alarm_commit();
}

static void alarm_mode_btn_cb(lv_event_t *e)
{
    uint8_t mode = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    s_alarm_edit.ring_mode = mode;
    /* Visual: uncheck the others. */
    lv_obj_clear_state(s_alarm_mode_beep, LV_STATE_CHECKED);
    lv_obj_clear_state(s_alarm_mode_vib, LV_STATE_CHECKED);
    lv_obj_clear_state(s_alarm_mode_both, LV_STATE_CHECKED);
    lv_obj_add_state((mode == ALARM_RING_BEEP) ? s_alarm_mode_beep :
                     (mode == ALARM_RING_VIB) ? s_alarm_mode_vib : s_alarm_mode_both,
                     LV_STATE_CHECKED);
    alarm_commit();
}

static void alarm_en_switch_cb(lv_event_t *e)
{
    (void)e;
    s_alarm_edit.enabled = lv_obj_has_state(s_alarm_en_switch, LV_STATE_CHECKED);
    alarm_commit();
}

static void lvgl_build_alarm_screen(void)
{
    s_alarm_screen = screen_new();
    lv_obj_set_style_bg_color(s_alarm_screen, lv_color_hex(0x201020), 0);

    lv_obj_t *title = lv_label_create(s_alarm_screen);
    lv_label_set_text(title, "Alarm");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_alarm_time_label = lv_label_create(s_alarm_screen);
    lv_obj_set_style_text_font(s_alarm_time_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_alarm_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_alarm_time_label, LV_ALIGN_TOP_MID, 0, 40);

    /* Hour +/- (plus on the right) */
    lv_obj_t *h_plus = lv_button_create(s_alarm_screen);
    lv_obj_set_size(h_plus, 160, 96);
    lv_obj_align(h_plus, LV_ALIGN_TOP_RIGHT, -25, 90);
    lv_obj_t *h_plus_lbl = lv_label_create(h_plus);
    lv_label_set_text(h_plus_lbl, "H+");
    lv_obj_set_style_text_font(h_plus_lbl, s_font_small, 0);
    lv_obj_center(h_plus_lbl);
    lv_obj_add_event_cb(h_plus, alarm_hour_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    lv_obj_t *h_minus = lv_button_create(s_alarm_screen);
    lv_obj_set_size(h_minus, 160, 96);
    lv_obj_align(h_minus, LV_ALIGN_TOP_LEFT, 25, 90);
    lv_obj_t *h_minus_lbl = lv_label_create(h_minus);
    lv_label_set_text(h_minus_lbl, "H-");
    lv_obj_set_style_text_font(h_minus_lbl, s_font_small, 0);
    lv_obj_center(h_minus_lbl);
    lv_obj_add_event_cb(h_minus, alarm_hour_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *m_plus = lv_button_create(s_alarm_screen);
    lv_obj_set_size(m_plus, 160, 96);
    lv_obj_align(m_plus, LV_ALIGN_TOP_RIGHT, -25, 195);
    lv_obj_t *m_plus_lbl = lv_label_create(m_plus);
    lv_label_set_text(m_plus_lbl, "M+");
    lv_obj_set_style_text_font(m_plus_lbl, s_font_small, 0);
    lv_obj_center(m_plus_lbl);
    lv_obj_add_event_cb(m_plus, alarm_min_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    lv_obj_t *m_minus = lv_button_create(s_alarm_screen);
    lv_obj_set_size(m_minus, 160, 96);
    lv_obj_align(m_minus, LV_ALIGN_TOP_LEFT, 25, 195);
    lv_obj_t *m_minus_lbl = lv_label_create(m_minus);
    lv_label_set_text(m_minus_lbl, "M-");
    lv_obj_set_style_text_font(m_minus_lbl, s_font_small, 0);
    lv_obj_center(m_minus_lbl);
    lv_obj_add_event_cb(m_minus, alarm_min_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    /* Ring mode selector. */
    lv_obj_t *mode_lbl = lv_label_create(s_alarm_screen);
    lv_label_set_text(mode_lbl, "Ring");
    lv_obj_set_style_text_font(mode_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(mode_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(mode_lbl, LV_ALIGN_TOP_LEFT, 40, 305);

    s_alarm_mode_beep = lv_button_create(s_alarm_screen);
    lv_obj_set_size(s_alarm_mode_beep, 110, 72);
    lv_obj_align(s_alarm_mode_beep, LV_ALIGN_TOP_LEFT, 25, 330);
    lv_obj_t *mb = lv_label_create(s_alarm_mode_beep);
    lv_label_set_text(mb, "Beep");
    lv_obj_set_style_text_font(mb, s_font_small, 0);
    lv_obj_center(mb);
    lv_obj_add_flag(s_alarm_mode_beep, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_mode_beep, alarm_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_BEEP);

    s_alarm_mode_vib = lv_button_create(s_alarm_screen);
    lv_obj_set_size(s_alarm_mode_vib, 110, 72);
    lv_obj_align(s_alarm_mode_vib, LV_ALIGN_TOP_LEFT, 150, 330);
    lv_obj_t *mv = lv_label_create(s_alarm_mode_vib);
    lv_label_set_text(mv, "Vib");
    lv_obj_set_style_text_font(mv, s_font_small, 0);
    lv_obj_center(mv);
    lv_obj_add_flag(s_alarm_mode_vib, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_mode_vib, alarm_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_VIB);

    s_alarm_mode_both = lv_button_create(s_alarm_screen);
    lv_obj_set_size(s_alarm_mode_both, 110, 72);
    lv_obj_align(s_alarm_mode_both, LV_ALIGN_TOP_LEFT, 275, 330);
    lv_obj_t *mbt = lv_label_create(s_alarm_mode_both);
    lv_label_set_text(mbt, "Both");
    lv_obj_set_style_text_font(mbt, s_font_small, 0);
    lv_obj_center(mbt);
    lv_obj_add_flag(s_alarm_mode_both, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_mode_both, alarm_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_BOTH);

    /* On/off switch - applies immediately (alarm_en_switch_cb). */
    lv_obj_t *en_lbl = lv_label_create(s_alarm_screen);
    lv_label_set_text(en_lbl, "Enabled");
    lv_obj_set_style_text_font(en_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(en_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(en_lbl, LV_ALIGN_TOP_LEFT, 40, 424);

    s_alarm_en_switch = lv_switch_create(s_alarm_screen);
    lv_obj_align(s_alarm_en_switch, LV_ALIGN_TOP_RIGHT, -40, 420);
    lv_obj_set_size(s_alarm_en_switch, 70, 40);
    lv_obj_add_event_cb(s_alarm_en_switch, alarm_en_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *hint = lv_label_create(s_alarm_screen);
    lv_label_set_text(hint, "swipe down to go back");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -6);

    /* Load current config and reflect it (no commit - this is a read). */
    alarm_get_config(&s_alarm_edit);
    char buf[8];
    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)s_alarm_edit.hour,
             (unsigned)s_alarm_edit.min);
    lv_label_set_text(s_alarm_time_label, buf);
    if (s_alarm_edit.enabled) {
        lv_obj_add_state(s_alarm_en_switch, LV_STATE_CHECKED);
    }
    lv_obj_add_state(s_alarm_edit.ring_mode == ALARM_RING_BEEP ? s_alarm_mode_beep :
                     s_alarm_edit.ring_mode == ALARM_RING_VIB ? s_alarm_mode_vib : s_alarm_mode_both,
                     LV_STATE_CHECKED);
}

void sim_alarm_screen_build(void)
{
    if (!s_alarm_screen) {
        lvgl_build_alarm_screen();
    }
    lv_scr_load(s_alarm_screen);
}
