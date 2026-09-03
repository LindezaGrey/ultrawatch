/*
 * services/daily_log.h - per-minute steps + activity logging to the SD card.
 *
 * A low-priority task wakes once per minute, samples the BHI260AP daily step
 * count, and appends one CSV row per sample. At each calendar-day rollover it
 * resets the daily step counter (bhi260ap_daily_sample()).
 *
 * Per-activity-class durations are NOT sampled here: bhi260ap.c tracks them
 * event-driven (bhi260ap_get_activity_ms()), crediting exact elapsed time on
 * every activity-change event the FIFO reports - including several in one
 * FIFO drain, none skipped, unlike periodic sampling of "the current
 * activity" would. daily_log_get_activity_seconds() just re-buckets that into
 * daily_log's own enum and converts to seconds; the day-rollover reset for it
 * lives in bhi260ap.c too (same ymd-change check that resets the step
 * counter).
 *
 * All writes are no-ops when the SD card is not mounted, and the module keeps
 * running (in-RAM counters) regardless, so the data is still available for the
 * BHI screen and the `dailylog` console command.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ordered activity classes tracked for the daily breakdown. Keep in sync with
 * the BHI260AP activity enum values. */
typedef enum {
    DAILY_ACT_STILL,
    DAILY_ACT_WALKING,
    DAILY_ACT_RUNNING,
    DAILY_ACT_CYCLING,
    DAILY_ACT_VEHICLE,
    DAILY_ACT_TILTING,
    DAILY_ACT_UNKNOWN,
    DAILY_ACT_COUNT,
} daily_activity_t;

/* One sample: called every 60s from the shared housekeeping task
 * (main/housekeeping.c), not its own task - see that file's header
 * comment. */
void daily_log_tick(void);

/* Today's step total (daily counter, resets at midnight). */
esp_err_t daily_log_get_steps(uint32_t *steps);

/* Seconds credited to each activity class today. */
esp_err_t daily_log_get_activity_seconds(const uint32_t *out[DAILY_ACT_COUNT]);

/* Flush any pending CSV writes now (also exposed via the dailylog command). */
void daily_log_flush(void);

#ifdef __cplusplus
}
#endif
