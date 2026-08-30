/*
 * main.c - LVGL PC simulator: all six real screens (watch face, power, BHI,
 * GPS, alarm-set, alarm-ring - each copied from main/lvgl_app.c into its own
 * sim/*_screen.c) against mocked drivers (mock_hw.c), with swipe navigation
 * between them (nav.c) and the physical panel's rounded-corner safe-area
 * overlay (assets/ui/safe_area_transparent.png) drawn on top so
 * corner-clipping is visible exactly like it would be on the real
 * T-Watch Ultra.
 *
 * Window is sized to the actual panel (410x502, see docs/hardware.md).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <SDL2/SDL.h>
#include "lvgl.h"
#include "screens.h"
#include "nav.h"
#include "screenshot.h"

#define SIM_HOR_RES 410
#define SIM_VER_RES 502

#ifndef UWATCH_ASSETS_UI_DIR
#define UWATCH_ASSETS_UI_DIR "."
#endif

int main(void)
{
    lv_init();
    lv_tick_set_cb(SDL_GetTicks);

    lv_display_t *disp = lv_sdl_window_create(SIM_HOR_RES, SIM_VER_RES);
    lv_sdl_window_set_title(disp, "UWatch simulator");
    lv_indev_t *mouse_indev = lv_sdl_mouse_create();

    sim_watch_face_start();
    /* Swipe navigation (mouse-drag stands in for a touch swipe) + the
     * menu-inactivity timeout, wired against the mouse indev just created. */
    sim_nav_init(mouse_indev);

    /* Rounded-corner safe-area overlay, drawn on the always-on-top layer so
     * it stays visible over every screen (including one-shot screenshots
     * below - moved ahead of that block so it's present in the capture). */
    lv_obj_t *safe_area = lv_image_create(lv_layer_top());
    lv_image_set_src(safe_area, "A:" UWATCH_ASSETS_UI_DIR "/safe_area_transparent.png");
    lv_obj_set_pos(safe_area, 0, 0);
    lv_obj_remove_flag(safe_area, LV_OBJ_FLAG_CLICKABLE);

    /* Headless screen selection + one-shot screenshot, for scripted/CI use
     * where driving the sim via real mouse/keyboard input isn't reliable
     * (e.g. a sandboxed X server with no visible desktop). Set
     * UWATCH_SIM_SCREEN to jump straight to a screen by name before the
     * event loop starts, and/or UWATCH_SIM_SHOT to a file path to dump one
     * frame (via screenshot.h, straight from the SDL renderer - no window
     * manager/compositor involved) and exit immediately after. */
    const char *screen_name = getenv("UWATCH_SIM_SCREEN");
    if (screen_name) {
        if (strcmp(screen_name, "watch") == 0) {
            sim_show_watch_face();
        } else if (strcmp(screen_name, "settings") == 0) {
            sim_settings_screen_build();
        } else if (strcmp(screen_name, "settings_tz") == 0) {
            sim_settings_tz_screen_build();
        } else if (strcmp(screen_name, "settings_disp") == 0) {
            sim_settings_disp_screen_build();
        } else if (strcmp(screen_name, "settings_periph") == 0) {
            sim_settings_periph_screen_build();
        } else if (strcmp(screen_name, "settings_sound") == 0) {
            sim_settings_sound_screen_build();
        } else if (strcmp(screen_name, "settings_info") == 0) {
            sim_settings_info_screen_build();
        } else if (strcmp(screen_name, "gps") == 0) {
            sim_gps_screen_build();
        } else if (strcmp(screen_name, "mesh") == 0) {
            sim_mesh_screen_build();
        } else if (strcmp(screen_name, "alarm") == 0) {
            sim_alarm_screen_build();
        } else if (strcmp(screen_name, "bhi") == 0) {
            sim_bhi_screen_build();
        } else if (strcmp(screen_name, "ring_alarm") == 0) {
            sim_ring_screen_build(ALARM_RING_SOURCE_ALARM);
        } else if (strcmp(screen_name, "ring_timer") == 0) {
            sim_ring_screen_build(ALARM_RING_SOURCE_TIMER);
        } else {
            fprintf(stderr, "UWATCH_SIM_SCREEN: unknown screen '%s'\n", screen_name);
        }
    }

    const char *autoshot_path = getenv("UWATCH_SIM_SHOT");
    if (autoshot_path) {
        /* A few refresh cycles so the requested screen's widgets/labels are
         * actually laid out and drawn before the read-back - a single
         * lv_timer_handler() call right after building a screen only
         * guarantees invalidation, not a completed flush. */
        for (int i = 0; i < 5; i++) {
            lv_timer_handler();
            SDL_Delay(20);
        }
        sim_screenshot_take(disp, autoshot_path);
        return 0;
    }

    /* Sim-only dev shortcut: the ring screen is normally only reachable via
     * the real alarm-firing/RTC-interrupt system, which doesn't exist on the
     * host (see ring_screen.c's header comment) - press 'R' for an
     * alarm-sourced preview (Snooze visible), 'T' for a timer-sourced one
     * (Snooze hidden, "Time's up").
     *
     * This polls SDL_GetKeyboardState() (a snapshot updated as a side effect
     * of SDL_PumpEvents(), not a queue read) rather than running our own
     * SDL_PollEvent() loop: LVGL's own SDL driver already owns the SDL event
     * queue, draining it itself once per refresh cycle from an internal 5 ms
     * lv_timer (see managed_components/lvgl__lvgl/src/drivers/sdl/
     * lv_sdl_window.c's sdl_event_handler(), registered from
     * lv_sdl_window_create()). A second SDL_PollEvent() loop here would race
     * that internal one for the same queue and could steal mouse/window
     * events LVGL needs; reading the keyboard-state snapshot instead never
     * touches the queue. */
    bool r_key_prev = false;
    bool t_key_prev = false;
    bool a_key_prev = false;
    bool s_key_prev = false;
    bool p_key_prev = false;
    int shot_seq = 0;

    for (;;) {
        uint32_t next = lv_timer_handler();

        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        bool r_key = keys[SDL_SCANCODE_R];
        if (r_key && !r_key_prev) {
            sim_ring_screen_build(ALARM_RING_SOURCE_ALARM);
        }
        r_key_prev = r_key;

        bool t_key = keys[SDL_SCANCODE_T];
        if (t_key && !t_key_prev) {
            sim_ring_screen_build(ALARM_RING_SOURCE_TIMER);
        }
        t_key_prev = t_key;

        /* 'A' for the Alarms/Timers list screen, same dev-shortcut idea as
         * 'R'/'T' above - swiping up from the watch face reaches it too
         * (nav.c), this is just a direct jump for quick iteration. */
        bool a_key = keys[SDL_SCANCODE_A];
        if (a_key && !a_key_prev) {
            sim_alarm_screen_build();
        }
        a_key_prev = a_key;

        /* 'S' for the Settings category list (docs/application.md section
         * 9) - not reachable via swipe yet (nav.c still mirrors the old
         * pre-nav-ring tree, see settings_screen.c's header comment), so
         * this is the only way in without editing main.c. */
        bool s_key = keys[SDL_SCANCODE_S];
        if (s_key && !s_key_prev) {
            sim_settings_screen_build();
        }
        s_key_prev = s_key;

        /* 'P' dumps the current frame straight from the SDL renderer's
         * framebuffer to /tmp/uwatch_sim_shot_NNN.ppm - see screenshot.h.
         * Works regardless of whether the window is actually visible on
         * screen (no window-manager/X11 pixel capture involved). */
        bool p_key = keys[SDL_SCANCODE_P];
        if (p_key && !p_key_prev) {
            char path[64];
            snprintf(path, sizeof(path), "/tmp/uwatch_sim_shot_%03d.ppm", shot_seq++);
            sim_screenshot_take(disp, path);
        }
        p_key_prev = p_key;

        SDL_Delay(next);
    }
    return 0;
}
