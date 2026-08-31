/*
 * screens.h - shared screen handles + entry points for the six ported
 * screens, standing in for main/lvgl_app.c's single-TU file-statics
 * (s_watch_screen, s_power_screen, ...) now that each screen lives in its
 * own .c file.
 *
 * Each lv_obj_t* is DEFINED (non-extern) in its owning screen's .c file and
 * declared here so nav.c (swipe/menu-timeout navigation) can compare against
 * lv_screen_active() exactly like the original single-file swipe_event_cb/
 * menu_timeout_cb did.
 *
 * Each sim_*_screen_build() is the lazy-build-then-load entry point used by
 * nav.c: "if the screen doesn't exist yet, build it; then lv_scr_load() it" -
 * the same laziness main/lvgl_app.c's swipe_event_cb has inline, just
 * factored out since the build functions themselves are static to their own
 * screen file.
 */
#pragma once

#include "lvgl.h"
#include "mock_hw.h"   /* alarm_ring_source_t, for sim_ring_screen_build() */

#ifdef __cplusplus
extern "C" {
#endif

extern lv_obj_t *s_watch_screen;
extern lv_obj_t *s_bhi_screen;
extern lv_obj_t *s_gps_screen;
extern lv_obj_t *s_mesh_screen;
extern lv_obj_t *s_alarm_screen;
extern lv_obj_t *s_ring_screen;
extern lv_obj_t *s_settings_screen;

/* Watch face: now a genuinely shared file, main/screens/watch_face.c,
 * compiled from this one copy by both builds (see its own header comment).
 * Its lvgl_build_watch_face()/watch_face_update() are declared in
 * main/screens/screens.h ("screens/screens.h" on this build's include
 * path - main/ is already added in sim/CMakeLists.txt) - callers here
 * (main.c, nav.c) include that header directly instead of a sim-only
 * wrapper. */

/* Lazy build-or-reuse + lv_scr_load() for each of the other screens. */
void sim_bhi_screen_build(void);
void sim_gps_screen_build(void);
void sim_mesh_screen_build(void);
void sim_wifi_screen_build(void);
void sim_ble_screen_build(void);
void sim_node_screen_build(void);
void sim_alarm_screen_build(void);
void sim_alarm_edit_screen_build(void);
void sim_timer_screen_build(void);
void sim_settings_screen_build(void);
/* Reachable both from the Settings category list and via tap-and-hold on
 * the watch face - exposed separately since it's the long-press target. */
void sim_settings_disp_screen_build(void);
void sim_settings_tz_screen_build(void);
void sim_settings_periph_screen_build(void);
void sim_settings_sound_screen_build(void);
void sim_settings_vib_screen_build(void);
void sim_settings_info_screen_build(void);
void sim_settings_sparmodus_screen_build(void);
/* source picks which ring-screen variant to preview: ALARM_RING_SOURCE_ALARM
 * (shows a configured time, offers Snooze) or ALARM_RING_SOURCE_TIMER (shows
 * "Time's up", no Snooze) - see ring_screen.c's header comment. */
void sim_ring_screen_build(alarm_ring_source_t source);

#ifdef __cplusplus
}
#endif
