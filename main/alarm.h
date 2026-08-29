/*
 * alarm.h - multi-alarm clock + countdown-timer ring engine.
 *
 * Up to ALARM_MAX_COUNT independent alarms, each with an optional weekday
 * repeat mask. The PCF85063A has exactly one hardware alarm register ("one
 * hardware alarm; multiple alarms must be handled in software" - see
 * pcf85063a.h), so this module keeps the full alarm list in RAM/NVS and
 * always arms the RTC for whichever entry's next occurrence is soonest,
 * re-computing on every add/update/remove/enable change and after every
 * fire.
 *
 * Also owns the shared ring engine (beep via MAX98357A, vibration via
 * DRV2605, on a dedicated task) used both for alarms and for main/cd_timer.c's
 * countdown timer - see alarm_ring_now(). Rings by beep, vibration or both,
 * until dismissed or (alarms only) snoozed. Config is persisted in NVS.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ring modes. */
#define ALARM_RING_BEEP  0   /* beep only */
#define ALARM_RING_VIB   1   /* vibration only */
#define ALARM_RING_BOTH  2   /* beep + vibration */

#define ALARM_MAX_COUNT  8

/* bit0=Sunday .. bit6=Saturday (matches struct tm's tm_wday numbering).
 * All 7 bits set = fires every day. */
#define ALARM_WEEKDAY_ALL 0x7Fu

typedef struct {
    bool     in_use;       /* slot occupied */
    bool     enabled;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  ring_mode;    /* ALARM_RING_* */
    uint8_t  weekday_mask; /* bit0=Sun..bit6=Sat, see ALARM_WEEKDAY_ALL */
} alarm_entry_t;

/* Which kind of ring is in progress - lets the ring screen hide Snooze (no
 * "snoozing a timer" concept) and lets alarm_dismiss()/alarm_snooze() know
 * whether to touch the RTC alarm at all. */
typedef enum {
    ALARM_RING_SOURCE_ALARM = 0,
    ALARM_RING_SOURCE_TIMER,
} alarm_ring_source_t;

/* Load config, arm the RTC alarm for the earliest enabled entry, and prepare
 * the ring machinery. Call once after twatch_board_init (RTC + audio ready). */
esp_err_t alarm_init(void);

/* ---- multi-alarm list ---- */

/* Adds a new alarm, persists it, and re-arms the RTC for whichever entry is
 * now soonest. Returns the new slot index, or -1 if all ALARM_MAX_COUNT
 * slots are in use. */
int alarm_add(uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask);

/* Updates an existing slot in place (same index, e.g. after editing a row).
 * ESP_ERR_INVALID_ARG if idx is out of range or not in_use. */
esp_err_t alarm_update(int idx, uint8_t hour, uint8_t min, uint8_t ring_mode,
                       uint8_t weekday_mask);

/* Frees a slot entirely (distinct from disabling it). */
esp_err_t alarm_remove(int idx);

/* Enables/disables one slot without changing its time/mode/weekdays. */
esp_err_t alarm_set_enabled(int idx, bool enabled);

/* Copies up to `max` slots into `out` **in slot order, including empty
 * ones** (check `.in_use` per element) - so an element's position in `out`
 * IS its idx, safe to pass straight into alarm_update/remove/set_enabled.
 * Returns the number of slots written (min(max, ALARM_MAX_COUNT)). */
size_t alarm_get_all(alarm_entry_t *out, size_t max);

/* True if at least one alarm is currently enabled (RTC alarm armed). Used by
 * power_mgmt.c to decide whether the RTC INT line is worth waking on. */
bool alarm_is_armed(void);

/* Index of the alarm entry that triggered the current/most recent
 * alarm-sourced ring (valid while ringing - the scheduler only re-arms
 * *after* dismiss, so this stays stable for the whole ring). -1 if none.
 * For UI display only (e.g. the ring screen showing which time to stamp). */
int alarm_get_ringing_index(void);

bool alarm_is_ringing(void);
bool alarm_is_snoozing(void);

/* Called on every 1 Hz UI tick: starts the ring if the RTC alarm or snooze
 * timer fired. Safe to call from any task. */
esp_err_t alarm_check(void);

/* Called by power_mgmt when the watch wakes from light sleep with the RTC INT
 * line low (alarm or snooze expiry). Starts the ring if due. */
esp_err_t alarm_handle_wake(void);

/* Stop the ring now. For an alarm-sourced ring: clears the RTC alarm flag
 * and re-arms for the next earliest occurrence. For a timer-sourced ring:
 * just stops the ring (main/cd_timer.c already marked itself inactive at
 * expiry - the RTC was never involved). */
esp_err_t alarm_dismiss(void);

/* Stop the ring and start the 10 min snooze countdown timer. Alarms only -
 * if called while a timer-sourced ring is active, behaves like
 * alarm_dismiss() instead (timers have no snooze concept). */
esp_err_t alarm_snooze(void);

/* Fire the ring immediately with ALARM_RING_BOTH, source=ALARM (debug/test).
 * Does not touch any real alarm entry; dismissing it just re-arms whatever
 * was already the earliest real entry (a no-op if none are enabled). */
esp_err_t alarm_ring_test(void);

/* General ring entrypoint: starts the shared ring engine for `source` using
 * `ring_mode`. Called internally when the RTC alarm fires (ring_mode taken
 * from whichever entry was armed) and externally by main/cd_timer.c on
 * countdown expiry (always ALARM_RING_SOURCE_TIMER, ALARM_RING_BOTH). */
esp_err_t alarm_ring_now(alarm_ring_source_t source, uint8_t ring_mode);

/* Called by the UI when ringing starts/stops (so it can show the ring
 * screen, and hide Snooze when source is a timer). */
typedef void (*alarm_ring_cb_t)(bool ringing, alarm_ring_source_t source);
void alarm_register_ring_cb(alarm_ring_cb_t cb);

#ifdef __cplusplus
}
#endif
