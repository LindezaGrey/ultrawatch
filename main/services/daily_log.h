/* Daily counters are reconciled against local RTC date once per second by
 * housekeeping and checkpointed with a persistent completed-day outbox.
 * CSV samples contain current activity plus accumulated per-class seconds;
 * completed days are written once per date, with retry after SD failure.
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
esp_err_t daily_log_get_activity_seconds(uint32_t out[DAILY_ACT_COUNT]);

/* Checkpoint current counters and retry completed summaries; also used before shutdown. */
void daily_log_flush(void);

#ifdef __cplusplus
}
#endif

void daily_log_init(void);
void daily_log_reconcile(void);
