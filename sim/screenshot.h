/*
 * screenshot.h - dev-only screenshot capture straight from the sim's SDL
 * renderer framebuffer (SDL_RenderReadPixels), bypassing the host window
 * manager/compositor entirely. Useful in sandboxes where grabbing the
 * actual on-screen window pixels (X11 screen capture, etc.) doesn't work.
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Writes the current contents of disp's SDL renderer to `path` as a
 * binary PPM (P6) file - no image library dependency. Convert to PNG
 * with e.g. `magick shot.ppm shot.png` if needed. Logs to stderr and
 * returns without writing anything on failure. */
void sim_screenshot_take(lv_display_t *disp, const char *path);

#ifdef __cplusplus
}
#endif
