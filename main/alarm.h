/*
 * alarm.h - daily alarm clock.
 *
 * Uses the PCF85063A hardware alarm (matched on hour+minute, fires daily) and
 * its countdown timer for snooze (10 min). Rings by beep (MAX98357A on BLDO2),
 * vibration (DRV2605) or both, on a dedicated task, until dismissed or
 * snoozed. Config is persisted in NVS.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ring modes. */
#define ALARM_RING_BEEP  0   /* beep only */
#define ALARM_RING_VIB   1   /* vibration only */
#define ALARM_RING_BOTH  2   /* beep + vibration */

typedef struct {
    bool     enabled;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  ring_mode;   /* ALARM_RING_* */
} alarm_config_t;

/* Load config, arm the RTC alarm if the config says so, and prepare the ring
 * machinery. Call once after twatch_board_init (RTC + audio ready). */
esp_err_t alarm_init(void);

/* Set the daily alarm time + mode and persist it. If enabled, arms the RTC
 * alarm interrupt immediately; if disabled, clears it. */
esp_err_t alarm_set(uint8_t hour, uint8_t min, bool enabled, uint8_t ring_mode);

/* Enable/disable the RTC alarm interrupt without touching the stored config. */
esp_err_t alarm_arm(void);
esp_err_t alarm_disarm(void);

bool alarm_is_armed(void);
bool alarm_is_ringing(void);
bool alarm_is_snoozing(void);
void alarm_get_config(alarm_config_t *cfg);

/* Called on every 1 Hz UI tick: starts the ring if the RTC alarm or snooze
 * timer fired. Safe to call from any task. */
esp_err_t alarm_check(void);

/* Called by power_mgmt when the watch wakes from light sleep with the RTC INT
 * line low (alarm or snooze expiry). Starts the ring if due. */
esp_err_t alarm_handle_wake(void);

/* Stop the ring now, clear the RTC alarm flag and re-arm for the next day. */
esp_err_t alarm_dismiss(void);

/* Stop the ring and start the 10 min snooze countdown timer. */
esp_err_t alarm_snooze(void);

/* Fire the ring immediately (debug/test). Does not touch the RTC flags. */
esp_err_t alarm_ring_test(void);

/* Called by the UI when ringing starts/stops (so it can show the ring screen). */
typedef void (*alarm_ring_cb_t)(bool ringing);
void alarm_register_ring_cb(alarm_ring_cb_t cb);

#ifdef __cplusplus
}
#endif
