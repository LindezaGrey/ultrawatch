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

/* Watch face (main/screens/watch_face.c). Not static: menu_timeout_cb()
 * and the nav-ring machinery in main/lvgl_app.c compare against it, same
 * as sim/nav.c does on the sim side. */
extern lv_obj_t *s_watch_screen;

void lvgl_build_watch_face(void);
void watch_face_update(lv_timer_t *timer);

#ifdef __cplusplus
}
#endif
