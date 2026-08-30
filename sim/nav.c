/*
 * nav.c - the real swipe/menu-timeout navigation from main/lvgl_app.c (lines
 * ~1698-1801 as of the porting pass: swipe_event_cb / menu_timeout_cb /
 * lvgl_show_watch_face), copied verbatim (same swipe-distance/timeout
 * constants, same per-screen swipe-direction rules) and wired up against the
 * sim's mouse indev instead of the real touch driver.
 *
 * Deviations from the firmware original (mechanical only):
 *   - No esp_lv_adapter_lock()/unlock(): none was present in this range of
 *     the original anyway.
 *   - swipe_event_cb's per-screen lazy-build-then-load bodies (originally
 *     inline, e.g. "if (!s_bhi_screen) { lvgl_build_bhi_screen(); }
 *     lv_scr_load(s_bhi_screen);") are replaced 1:1 with calls to each
 *     screen's sim_*_screen_build() entry point (screens.h), since the
 *     static lvgl_build_*_screen() functions now live in their own screen
 *     .c files and can't be called directly from here. The laziness and the
 *     swipe-direction logic are unchanged.
 *   - lvgl_show_watch_face() is renamed sim_show_watch_face() (exposed via
 *     nav.h) since it's no longer file-static to a single lvgl_app.c
 *     translation unit; its body calls watch_face_update(NULL) directly -
 *     main/screens/watch_face.c is a genuinely shared file now, so this is
 *     the exact same function the firmware calls, not a sim-only wrapper.
 *   - Registration: main/lvgl_app.c registers swipe_event_cb on the real
 *     touch indev inside lvgl_app_start() (see its s_touch_indev /
 *     lv_indev_add_event_cb() call site). sim_nav_init() below does the
 *     same against the sim's mouse indev, called once from main.c.
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include <stdlib.h>
#include "lvgl.h"
#include "screens.h"
#include "screens/screens.h"   /* shared: lvgl_build_watch_face(), watch_face_update() */
#include "nav.h"

/* Swipe detection at the input-device level (works regardless of widget). */
#define SWIPE_DIST         60
#define MENU_TIMEOUT_MS    5000   /* return to watch face after this idle */
#define BHI_TIMEOUT_MS     10000  /* BHI screen keeps the cube up a bit longer */

static lv_point_t s_swipe_start;
static bool s_swipe_active;
uint32_t s_last_touch_tick;   /* lv_tick_get() at last touch - extern via screens/screens.h */

void sim_show_watch_face(void)
{
    lv_scr_load(s_watch_screen);
    watch_face_update(NULL);
}

/* GPS screen: now shared (main/screens/gps_screen.c), which - unlike the
 * old hand-ported sim/gps_screen.c - exposes only lvgl_build_gps_screen()/
 * gps_screen_update(), no lazy-build-then-load wrapper (the firmware does
 * that lazy check inline at its own call site, same pattern used here). */
void sim_gps_screen_build(void)
{
    if (!s_gps_screen) {
        lvgl_build_gps_screen();
    }
    lv_scr_load(s_gps_screen);
}

/* Mesh screen + Node overview: now shared (main/screens/mesh_screen.c). */
void sim_mesh_screen_build(void)
{
    if (!s_mesh_screen) {
        lvgl_build_mesh_screen();
    }
    lv_scr_load(s_mesh_screen);
    mesh_screen_update(NULL);
}

/* Public entry point for screenshot/dev-shortcut use (see main.c's
 * UWATCH_SIM_SCREEN env var) - the node overview is reached by an upward
 * fling from Mesh on real hardware, not a swipe this file's own
 * swipe_event_cb wires up. */
void sim_node_screen_build(void)
{
    lvgl_show_node_overview();
}

/* Settings category list + its 6 sub-pages: now shared
 * (main/screens/settings_screen.c). Only the category list needs its own
 * build-or-reuse+load wrapper here - lvgl_show_settings_disp() (Display)
 * already IS one (shared, called directly by watch_face_long_press_cb()
 * below); the other 4 sub-pages have no swipe/tap entry point in the sim
 * at all (matches the firmware - only reachable via the category list),
 * so these wrappers exist purely for main.c's UWATCH_SIM_SCREEN dev
 * shortcut/screenshot use. */
void sim_settings_screen_build(void)
{
    if (!s_settings_screen) {
        lvgl_build_settings_screen();
    }
    lv_scr_load(s_settings_screen);
}

void sim_settings_disp_screen_build(void)
{
    lvgl_show_settings_disp();
}

void sim_settings_tz_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(0);
}

void sim_settings_periph_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(2);
}

void sim_settings_sound_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(3);
}

void sim_settings_info_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(4);
}

void sim_settings_sparmodus_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(5);
}

/* BHI260AP status screen: now shared (main/screens/bhi_screen.c). */
void sim_bhi_screen_build(void)
{
    if (!s_bhi_screen) {
        lvgl_build_bhi_screen();
    }
    lv_scr_load(s_bhi_screen);
    bhi_screen_update(NULL);
    s_last_touch_tick = lv_tick_get();
}

/* Alarms/Timers list + its 2 sub-screens + ringing screen: now shared
 * (main/screens/alarm_screen.c, main/screens/ring_screen.c). */
void sim_alarm_screen_build(void)
{
    if (!s_alarm_screen) {
        lvgl_build_alarm_screen();
    }
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

/* Public entry points for screenshot/dev-shortcut use (see main.c's
 * UWATCH_SIM_SCREEN env var) - both sub-screens are normally reached by
 * tapping a control on the Alarms list, not a direct swipe. */
void sim_alarm_edit_screen_build(void)
{
    if (!s_alarm_edit_screen) {
        lvgl_build_alarm_edit_screen();
    }
    lv_scr_load(s_alarm_edit_screen);
}

void sim_timer_screen_build(void)
{
    if (!s_timer_screen) {
        lvgl_build_timer_screen();
    }
    lv_scr_load(s_timer_screen);
}

/* Ring screen preview: normally shown only via the firmware's
 * alarm_ring_cb() (runs on the alarm ring task, needs
 * esp_lv_adapter_lock() - no sim equivalent), so main.c's 'R'/'T'
 * dev-shortcut keys call this directly instead, replicating just the
 * source-aware title/time-label logic alarm_ring_cb() also does. */
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
                snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)list[i].hour,
                         (unsigned)list[i].min);
                break;
            }
        }
        lv_label_set_text(s_ring_time_label, buf);
        lv_obj_clear_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
    }
    lv_scr_load(s_ring_screen);
}

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
                sim_bhi_screen_build();
            } else {             /* right -> GPS */
                sim_gps_screen_build();
            }
        } else if (dy < 0) {     /* up   -> Alarm */
            sim_alarm_screen_build();
        }
    } else if (cur == s_alarm_screen) {
        if (!horiz && dy > 0) {  /* down -> clock */
            sim_show_watch_face();
        }
    } else if (cur == s_bhi_screen) {
        if (horiz && dx > 0) {   /* right -> clock */
            sim_show_watch_face();
        }
    } else if (cur == s_gps_screen) {
        if (horiz && dx < 0) {   /* left -> clock */
            sim_show_watch_face();
        } else if (horiz && dx > 0) {   /* right -> Mesh */
            sim_mesh_screen_build();
        }
    } else if (cur == s_mesh_screen) {
        if (horiz && dx < 0) {   /* left -> GPS */
            sim_gps_screen_build();
        } else if (!horiz && dy < 0) {   /* up -> Node overview */
            lvgl_show_node_overview();
        }
    }
}

/* ---- Menu inactivity timeout ----
 * Any non-watch-face screen returns to the watch face after MENU_TIMEOUT_MS
 * without a touch. The BHI sensor screen keeps its orientation cube up for
 * BHI_TIMEOUT_MS (10 s); the GPS and alarm/ring screens are exempt (longer
 * observation). */
static void menu_timeout_cb(lv_timer_t *timer)
{
    (void)timer;
    lv_obj_t *cur = lv_screen_active();
    if (cur == s_gps_screen || cur == s_alarm_screen || cur == s_ring_screen ||
        cur == s_settings_screen) {
        return;
    }
    if (cur == s_watch_screen) {
        return;
    }
    uint32_t timeout = (cur == s_bhi_screen) ? BHI_TIMEOUT_MS : MENU_TIMEOUT_MS;
    if (lv_tick_get() - s_last_touch_tick >= timeout) {
        sim_show_watch_face();
    }
}

/* Tap-and-hold on the watch face -> Display settings (docs/application.md
 * section 9.3), mirroring main/lvgl_app.c's watch_face_long_press_cb(). */
static void watch_face_long_press_cb(lv_event_t *e)
{
    (void)e;
    if (lv_screen_active() != s_watch_screen) {
        return;
    }
    sim_settings_disp_screen_build();
}

void sim_nav_init(lv_indev_t *touch_indev)
{
    if (touch_indev) {
        /* Indev-level swipe detection: fires for every touch regardless of
         * which widget/screen is active. */
        lv_indev_add_event_cb(touch_indev, swipe_event_cb, LV_EVENT_PRESSED, NULL);
        lv_indev_add_event_cb(touch_indev, swipe_event_cb, LV_EVENT_RELEASED, NULL);
        lv_indev_add_event_cb(touch_indev, watch_face_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
    }
    /* Menu inactivity timeout (runs forever; no-op on the watch face). */
    lv_timer_create(menu_timeout_cb, 500, NULL);
}
