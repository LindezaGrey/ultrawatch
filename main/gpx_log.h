/*
 * gpx_log.h - GPX track logging to the SD card.
 *
 * One session at a time: gpx_log_start() opens a new timestamped .gpx file
 * under /sdcard/log/gpx/ and a background task appends one <trkpt> every
 * 30 s (skipping ticks with no valid fix) until gpx_log_stop() closes it.
 * See docs/application.md section 6 ("GPX-Tracking, alle 30 Sekunden").
 *
 * Not the same feature as main/tracking.c (a step-gated pedometer/distance
 * tracker, currently disabled - see TRACKING_ENABLED in tracking.h) - this
 * module is purely time-based position logging and doesn't touch it.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One tick: called every 30s from the shared housekeeping task
 * (main/housekeeping.c), not its own task - see that file's header
 * comment. */
void gpx_log_tick(void);

/* Begins a new session: creates /sdcard/log/gpx (if needed) and a new
 * track_<YYYYMMDD_HHMMSS>.gpx file with the GPX header written. */
esp_err_t gpx_log_start(void);

/* Ends the current session, appending the closing tags. No-op if not active. */
esp_err_t gpx_log_stop(void);

bool gpx_log_is_active(void);

/* Trackpoints written so far in the current (or most recently finished)
 * session. For the GPS screen's live display. */
uint32_t gpx_log_point_count(void);

/* Path of the current (or most recently finished) session's file, or an
 * empty string if none yet this boot. For the gpxcat debug command. */
const char *gpx_log_current_path(void);

#ifdef __cplusplus
}
#endif
