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
#include <SDL2/SDL.h>
#include "lvgl.h"
#include "screens.h"
#include "nav.h"

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
     * it stays visible over every screen. */
    lv_obj_t *safe_area = lv_image_create(lv_layer_top());
    lv_image_set_src(safe_area, "A:" UWATCH_ASSETS_UI_DIR "/safe_area_transparent.png");
    lv_obj_set_pos(safe_area, 0, 0);
    lv_obj_remove_flag(safe_area, LV_OBJ_FLAG_CLICKABLE);

    /* Sim-only dev shortcut: the ring screen is normally only reachable via
     * the real alarm-firing/RTC-interrupt system, which doesn't exist on the
     * host (see ring_screen.c's header comment) - press 'R' to view it.
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

    for (;;) {
        uint32_t next = lv_timer_handler();

        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        bool r_key = keys[SDL_SCANCODE_R];
        if (r_key && !r_key_prev) {
            sim_ring_screen_build();
        }
        r_key_prev = r_key;

        SDL_Delay(next);
    }
    return 0;
}
