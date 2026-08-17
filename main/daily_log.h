/*
 * daily_log.h - per-minute steps + activity logging to the SD card.
 *
 * A low-priority task wakes once per minute, samples the BHI260AP daily step
 * count and the (sticky) activity class, and appends one CSV row per sample.
 * At each calendar-day rollover it resets the daily step counter and the
 * per-activity minute tallies.
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

/* Start the daily logging task. Call once after sd_log_mount(). */
void daily_log_init(void);

/* Today's step total (daily counter, resets at midnight). */
esp_err_t daily_log_get_steps(uint32_t *steps);

/* Minutes credited to each activity class today. */
esp_err_t daily_log_get_activity_minutes(const uint16_t *out[DAILY_ACT_COUNT]);

/* Flush any pending CSV writes now (also exposed via the dailylog command). */
void daily_log_flush(void);

#ifdef __cplusplus
}
#endif
