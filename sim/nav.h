/*
 * nav.h - screen-swipe + menu-inactivity navigation for the sim, ported from
 * main/lvgl_app.c's swipe_event_cb/menu_timeout_cb/lvgl_show_watch_face.
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register indev-level swipe detection on touch_indev (the mouse indev on
 * the host - mouse-drag on the SDL window stands in for a touch swipe) and
 * start the menu-inactivity timeout timer. touch_indev may be NULL (no
 * swipe navigation, matching the firmware's own "if (tp) {...}" guard when
 * touch registration fails). */
void sim_nav_init(lv_indev_t *touch_indev);

/* Load the watch face and refresh its clock labels. main/lvgl_app.c's
 * lvgl_show_watch_face(), renamed since it's no longer file-static to a
 * single lvgl_app.c translation unit. */
void sim_show_watch_face(void);

#ifdef __cplusplus
}
#endif
