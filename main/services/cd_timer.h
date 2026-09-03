/*
 * services/cd_timer.h - single countdown timer (kitchen-timer style), RAM-only.
 *
 * One active timer at a time (docs/application.md SS8.2 talks about "an
 * active countdown" in the singular) - starting a new one while one is
 * running replaces it. In-RAM only, not persisted: lost on reboot like a
 * physical countdown timer, deliberately (avoids per-tick NVS writes; see
 * the Phase 2 plan for the reasoning).
 *
 * On expiry, rings via alarm.c's shared ring engine (alarm_ring_now()) with
 * ALARM_RING_SOURCE_TIMER, ALARM_RING_BOTH - always both beep+vibration,
 * since there's no per-timer settings UI (presets-only creation: pick a
 * duration, it starts immediately).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts (or replaces) the active timer, counting down from `seconds`. */
esp_err_t cdtimer_start(uint32_t seconds);

/* Cancels the active timer without ringing. No-op if none is active. */
void cdtimer_cancel(void);

bool cdtimer_is_active(void);

/* Seconds remaining, 0 if not active. For the list screen's live countdown
 * display. */
uint32_t cdtimer_remaining_seconds(void);

/* Called on every 1 Hz UI tick (same cadence as alarm_check()): decrements
 * the active timer and starts the ring via alarm_ring_now() on expiry. */
void cdtimer_check(void);

#ifdef __cplusplus
}
#endif
