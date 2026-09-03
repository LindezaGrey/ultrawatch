/*
 * services/housekeeping.h - one shared background task for the app's low-activity
 * periodic SD writers: daily_log.c (60s), gpx_log.c (30s), and
 * syslog_capture.c (10s, or sooner on its own buffer-full wake).
 *
 * These used to each own a dedicated always-running FreeRTOS task, despite
 * all three having the same shape (wake up periodically, do a bounded bit
 * of file I/O, sleep) and modest CPU needs - three separate task stacks
 * for that is wasteful on a board where internal DMA-capable RAM is
 * chronically tight (see docs/application.md section 12). Consolidating
 * into one task removes two whole task stacks (~6 KB) outright, which is
 * a safer way to reclaim memory than shrinking any one task's own stack -
 * that class of fix already caused a real live stack-overflow crash this
 * session (a task's real worst-case call depth wasn't what an idle
 * snapshot suggested).
 *
 * Each module keeps its own state/logic in its own file - daily_log_tick()
 * and gpx_log_tick() are just their old task loop bodies, now called on a
 * schedule from here instead of looping in their own task. Cadence is
 * tracked by elapsed ticks (xTaskGetTickCount()), not a loop-iteration
 * counter, so syslog's early buffer-full wake never accidentally speeds
 * up the 30s/60s schedules.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at boot, after syslog_capture_init() (this task calls
 * syslog_capture_service(), which needs that module's mutex/semaphore
 * already created). daily_log.c/gpx_log.c need no init of their own -
 * they're pure functions now, no task, no other state to set up. */
void housekeeping_init(void);

#ifdef __cplusplus
}
#endif
