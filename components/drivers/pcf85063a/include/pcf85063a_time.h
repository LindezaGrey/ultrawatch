/*
 * pcf85063a_time.h - pure UTC calendar<->epoch conversion for pcf85063a_time_t.
 *
 * No ESP-IDF or hardware dependencies (just <stdint.h>/<time.h>): this is the
 * seam pcf85063a.c's I2C code calls into, and it's usable standalone (e.g.
 * host-side tests) without pulling in driver/i2c_master.h or FreeRTOS.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The hardware always stores UTC (never local wall time): callers convert to
 * local only at display or alarm-arming time, via pcf85063a_time_to_epoch()/
 * pcf85063a_epoch_to_time() below plus mktime()/localtime_r() and the
 * process TZ. Weekday is the raw Register_03 value and matches struct tm's
 * tm_wday: 0=Sunday .. 6=Saturday. */
typedef struct {
    uint8_t  sec;
    uint8_t  min;
    uint8_t  hour;
    uint8_t  day;
    uint8_t  weekday;
    uint8_t  month;   /* 1..12 */
    uint16_t year;    /* full year, e.g. 2026 */
} pcf85063a_time_t;

/* RTC calendar conversion is supported only for years 2000..2099. The chip
 * has no century bit and treats year 00 as leap; do not use this RTC calendar
 * for 2100-century dates.
 *
 * Convert between a pcf85063a_time_t (plain UTC calendar) and a UTC time_t
 * epoch using proleptic Gregorian civil-date arithmetic. Unlike libc
 * mktime()/localtime_r(), these helpers do not use process TZ/DST. The epoch
 * converter fills weekday. */
bool pcf85063a_time_is_valid(const pcf85063a_time_t *t);
/* Alarm fields are validated according to which fields participate in match. */
bool pcf85063a_alarm_fields_are_valid(uint8_t sec, uint8_t min, uint8_t hour,
                                      uint8_t day, uint8_t weekday,
                                      bool mask_day, bool mask_weekday);
/* Validate and decode one packed BCD field after masking its register bits. */
bool pcf85063a_decode_bcd_field(uint8_t raw, uint8_t tens_mask,
                                uint8_t max_value, uint8_t *value);
time_t pcf85063a_time_to_epoch(const pcf85063a_time_t *t);
void pcf85063a_epoch_to_time(time_t epoch, pcf85063a_time_t *out);

#ifdef __cplusplus
}
#endif
