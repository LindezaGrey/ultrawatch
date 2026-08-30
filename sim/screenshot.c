/*
 * screenshot.c - see screenshot.h. Reads pixels straight back from the SDL
 * renderer's own framebuffer (SDL_RenderReadPixels), so this works
 * regardless of whether the window is actually visible/composited on
 * screen - unlike external tools (import/ffmpeg x11grab/etc.) which grab
 * from the window manager and can come back blank in some sandboxed X11
 * setups (e.g. rootless XWayland with no real root pixmap).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <SDL2/SDL.h>
#include "lvgl.h"
#include "screenshot.h"

void sim_screenshot_take(lv_display_t *disp, const char *path)
{
    SDL_Renderer *renderer = (SDL_Renderer *)lv_sdl_window_get_renderer(disp);
    if (!renderer) {
        fprintf(stderr, "screenshot: no renderer for this display\n");
        return;
    }

    int w = 0, h = 0;
    if (SDL_GetRendererOutputSize(renderer, &w, &h) != 0 || w <= 0 || h <= 0) {
        fprintf(stderr, "screenshot: SDL_GetRendererOutputSize failed: %s\n", SDL_GetError());
        return;
    }

    int pitch = w * 3;
    uint8_t *pixels = malloc((size_t)pitch * (size_t)h);
    if (!pixels) {
        fprintf(stderr, "screenshot: out of memory (%dx%d)\n", w, h);
        return;
    }

    if (SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_RGB24, pixels, pitch) != 0) {
        fprintf(stderr, "screenshot: SDL_RenderReadPixels failed: %s\n", SDL_GetError());
        free(pixels);
        return;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "screenshot: fopen(%s) failed\n", path);
        free(pixels);
        return;
    }
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(pixels, 1, (size_t)pitch * (size_t)h, f);
    fclose(f);
    free(pixels);

    printf("screenshot: wrote %s (%dx%d)\n", path, w, h);
    fflush(stdout);
}
