/*
 * status_bar.h - the small row of state icons shown on every nav-ring
 * screen (docs/application.md section 3). Shared between the firmware
 * (main/) and the host sim (sim/) - see main/screens/watch_face.c's header
 * comment for why this can be a genuinely shared file rather than a
 * hand-copied port.
 */
#pragma once

#include <stdbool.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    lv_obj_t *sd;
    lv_obj_t *gps;
    lv_obj_t *gpx;
    lv_obj_t *lora;
    lv_obj_t *bt;
    lv_obj_t *wifi;
    lv_obj_t *batt;
    lv_obj_t *chg;
} status_bar_t;

/* Builds one status bar instance into `parent` (a screen about to be shown
 * for the first time) and stores its objects in `out` for update_status_bar()
 * to refresh later. Only the watch face builds one - see status_bar.c. */
void build_status_bar_big(lv_obj_t *parent, status_bar_t *out);

/* Refreshes one status bar instance. Safe to call even if `bar->sd` (or any
 * field) is NULL - i.e. before that screen has been built. */
void update_status_bar(const status_bar_t *bar);

/* Shows/hides every icon in one status bar instance at once - used to drop
 * the whole row for Ultra-Sparmodus's minimal watch face. Safe to call
 * before the screen is built (same NULL guard as update_status_bar()). */
void status_bar_set_hidden(const status_bar_t *bar, bool hidden);

#ifdef __cplusplus
}
#endif
