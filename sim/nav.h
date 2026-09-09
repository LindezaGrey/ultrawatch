/* Simulator entry points for shared navigation and screenshot shortcuts. */
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

void sim_show_watch_face(void);

#ifdef __cplusplus
}
#endif
