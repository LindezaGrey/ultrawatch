/*
 * screens.h - shared declarations for screens under main/screens/, built
 * identically by the firmware (main/lvgl_app.c) and the host sim
 * (sim/main.c). Grows as more screens move out of main/lvgl_app.c - see
 * the plan this migration follows for the target file list.
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* New screen with scrolling disabled: no scrollbars, content locked so
 * swipes always reach the screen-swipe navigation instead of scrolling
 * the content. Trivial and state-free enough to inline here rather than
 * add a one-function .c file - every screen (shared or not yet migrated)
 * calls this first thing in its builder. */
static inline lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

/* Watch face. Screen handles are registered in navigation.c. */
extern lv_obj_t *s_watch_screen;

void lvgl_build_watch_face(void);
void watch_face_update(lv_timer_t *timer);

/* Activity screen (main/screens/activity_screen.c): today's step count +
 * per-activity duration breakdown (BHI260AP activity recognition),
 * read-only - see main/bhi260ap.h / main/daily_log.h. */
extern lv_obj_t *s_activity_screen;

void lvgl_build_activity_screen(void);
void activity_screen_update(lv_timer_t *timer);

/* GPS screen (main/screens/gps_screen.c). */
extern lv_obj_t *s_gps_screen;

void lvgl_build_gps_screen(void);
void gps_screen_update(lv_timer_t *timer);

/* "Is GNSS powered" flag + its power-on timestamp, owned by gps_screen.c
 * but touched from main/lvgl_app.c's gps_ctrl_task (the real background
 * power-transition worker, firmware-only) and settings_periph_refresh()
 * (the Settings/Peripherie screen, which mirrors the same on/off state). */
bool gps_screen_is_powered(void);
void gps_screen_set_powered(bool on);

/* Mesh and its node overview. */
extern lv_obj_t *s_mesh_screen;
extern lv_obj_t *s_node_screen;

void lvgl_build_mesh_screen(void);
void mesh_screen_update(lv_timer_t *timer);
void lvgl_show_node_overview(void);

/* WiFi screen (main/screens/wifi_screen.c): power switch + scanned-network
 * list, read-only (no connect flow) - see main/wifi_scan.h. */
extern lv_obj_t *s_wifi_screen;

void lvgl_build_wifi_screen(void);
void wifi_screen_update(lv_timer_t *timer);

/* Bluetooth screen (main/screens/ble_screen.c): power switch + one-shot
 * 30s device scan, read-only (no pair/connect flow) - see
 * main/ble_scan.h. */
extern lv_obj_t *s_ble_screen;

void lvgl_build_ble_screen(void);
void ble_screen_update(lv_timer_t *timer);

/* Settings and its subpages; preserved across normal display sleep. */
extern lv_obj_t *s_settings_screen;
extern lv_obj_t *s_set_tz_screen;
extern lv_obj_t *s_set_disp_screen;
extern lv_obj_t *s_set_periph_screen;
extern lv_obj_t *s_set_sound_screen;
extern lv_obj_t *s_set_info_screen;
extern lv_obj_t *s_set_sparmodus_screen;
/* Nested under Ton & Vibration (Sound), not a top-level category - see
 * main/screens/settings_screen.c's settings_vib_row_open_cb(). */
extern lv_obj_t *s_set_vib_screen;
void settings_open_vib_screen(void);

void lvgl_build_settings_screen(void);
/* Reachable both from the Settings category list and via tap-and-hold on
 * the watch face (docs/application.md section 9.3) - navigation.c's
 * long-press handler calls this directly. */
void lvgl_show_settings_disp(void);
/* Category row -> sub-page dispatch (0=TZ, 1=Display, 2=Peripherie,
 * 3=Sound, 4=Info, 5=Ultra-Sparmodus). Not just the row-click handler's
 * internals: sim/nav.c's UWATCH_SIM_SCREEN dev shortcut calls this
 * directly too, since none of these sub-pages (besides Display) have any
 * other entry point. */
void settings_open_subpage(int idx);

/* Alarms, editors and ringing UI. */
extern lv_obj_t *s_alarm_screen;
extern lv_obj_t *s_alarm_edit_screen;
extern lv_obj_t *s_timer_screen;
extern lv_obj_t *s_ring_screen;
extern lv_obj_t *s_ring_title_label;
extern lv_obj_t *s_ring_time_label;
extern lv_obj_t *s_ring_snooze_btn;

void lvgl_build_alarm_screen(void);
void lvgl_build_alarm_edit_screen(void);
void lvgl_build_timer_screen(void);
void lvgl_build_ring_screen(void);
void alarm_list_refresh(void);

/* Sensor diagnostics, reachable from the debug console. */
extern lv_obj_t *s_bhi_screen;
void lvgl_build_bhi_screen(void);
/* bhi_screen_update() gates on being the active screen, so the call
 * inside lvgl_build_bhi_screen() (before lv_scr_load()) is a no-op -
 * callers must refresh again right after loading, same "load first,
 * then refresh" fix as mesh_screen_update()/navigation_open(). */
void bhi_screen_update(lv_timer_t *timer);

extern lv_obj_t *s_nfc_screen;
void lvgl_build_nfc_screen(void);
void nfc_screen_create(lv_event_cb_t start_cb);
extern lv_obj_t *s_nfc_status_label, *s_nfc_uid_label, *s_nfc_start_btn;

#ifdef __cplusplus
}
#endif
