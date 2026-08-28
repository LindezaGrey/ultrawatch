/*
 * pcf85063a_time.h - pure UTC calendar<->epoch conversion for pcf85063a_time_t.
 *
 * No ESP-IDF or hardware dependencies (just <stdint.h>/<time.h>): this is the
 * seam pcf85063a.c's I2C code calls into, and it's usable standalone (e.g.
 * host-side tests) without pulling in driver/i2c_master.h or FreeRTOS.
 */
#pragma once

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The hardware always stores UTC (never local wall time): callers convert to
 * local only at display or alarm-arming time, via pcf85063a_time_to_epoch()/
 * pcf85063a_epoch_to_time() below plus mktime()/localtime_r() and the
 * process TZ. Weekday follows the RTC convention: 1=Sunday .. 7=Saturday. */
typedef struct {
    uint8_t  sec;
    uint8_t  min;
    uint8_t  hour;
    uint8_t  day;
    uint8_t  weekday;
    uint8_t  month;   /* 1..12 */
    uint16_t year;    /* full year, e.g. 2026 */
} pcf85063a_time_t;

/* Convert between a pcf85063a_time_t (treated as a plain UTC calendar
 * breakdown - the RTC's own convention) and a UTC time_t epoch. Direct
 * civil-date<->days-since-epoch math (proleptic Gregorian, Howard Hinnant's
 * algorithm): unlike mktime()/localtime_r(), neither applies the process TZ
 * or DST, so these are safe to use before app_main() sets TZ and never
 * touch tm_isdst. pcf85063a_epoch_to_time() also fills weekday. */
time_t pcf85063a_time_to_epoch(const pcf85063a_time_t *t);
void pcf85063a_epoch_to_time(time_t epoch, pcf85063a_time_t *out);

#ifdef __cplusplus
}
#endif
