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

/* Watch face (main/screens/watch_face.c). Not static: menu_timeout_cb()
 * and the nav-ring machinery in main/lvgl_app.c compare against it, same
 * as sim/nav.c does on the sim side. */
extern lv_obj_t *s_watch_screen;

void lvgl_build_watch_face(void);
void watch_face_update(lv_timer_t *timer);

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

#ifdef __cplusplus
}
#endif
