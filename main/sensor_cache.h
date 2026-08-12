/*
 * sensor_cache.h - background telemetry cache for slow I2C peripherals.
 *
 * The AXP2101 PMU (battery %, mV, charge state, current, temp) and the
 * PCF85063A RTC are polled by a dedicated low-priority task into a RAM struct
 * so the LVGL/UI task never blocks on I2C reads. Screens read the cached
 * snapshot via sensor_cache_get().
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pcf85063a.h"
#include "axp2101.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cached snapshot of the slow I2C peripherals, refreshed once per second by
 * the background telemetry task. */
typedef struct {
    pcf85063a_time_t rtc;          /* wall-clock time from the RTC */
    bool             rtc_valid;
    uint8_t          batt_pct;     /* battery percentage 0..100 */
    uint16_t         batt_mv;      /* battery voltage in mV */
    axp2101_charge_state_t chg_state;
    bool             chg_enabled;  /* charge function enabled */
    uint16_t         chg_ma;       /* configured charge current */
    int16_t          batt_temp_c10;/* battery temperature, 0.1 C */
    bool             valid;        /* true when all reads succeeded */
} sensor_cache_t;

/* Start the background telemetry task. */
void sensor_cache_init(void);

/* Copy the latest cached snapshot (cheap: ~20 bytes, mutex-protected). */
void sensor_cache_get(sensor_cache_t *out);

/* Convenience: latest RTC time without the full struct copy. Returns true if
 * the cache holds a valid RTC reading. */
bool sensor_cache_get_rtc(pcf85063a_time_t *out);

/* Battery gauge rate estimate, derived from the % change over a rolling
 * ~5-minute window while the watch is awake. */
typedef struct {
    bool   estimate_valid;   /* enough stable movement to estimate */
    float  pct_per_hour;     /* signed: <0 discharging, >0 charging */
    float  runtime_h;        /* valid when discharging (hours until empty) */
    float  charge_h;         /* valid when charging (hours until full) */
} battery_estimate_t;

/* Latest battery rate estimate (computed in the background cache task). */
void battery_estimate_get(battery_estimate_t *out);

#ifdef __cplusplus
}
#endif
