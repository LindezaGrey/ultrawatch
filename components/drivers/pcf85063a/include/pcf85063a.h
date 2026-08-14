/*
 * pcf85063a.h - NXP PCF85063A RTC (I2C 0x51).
 *
 * Register map (verified against the Linux rtc-pcf85063 driver + datasheet):
 *   0x00 Control1 (STOP=bit5), 0x01 Control2 (AF=bit6, AIE=bit7, TIE=bit5),
 *   0x02 Offset, 0x03 RAM, 0x04..0x0A time (BCD), 0x0B..0x0F alarm,
 *   0x10..0x12 countdown timer. Seconds 0x04 bit7 = OS flag.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCF85063A_I2C_ADDR 0x51

/* Weekday follows the RTC convention: 1=Sunday .. 7=Saturday. */
typedef struct {
    uint8_t  sec;
    uint8_t  min;
    uint8_t  hour;
    uint8_t  day;
    uint8_t  weekday;
    uint8_t  month;   /* 1..12 */
    uint16_t year;    /* full year, e.g. 2026 */
} pcf85063a_time_t;

typedef struct {
    bool  enabled;                 /* enable the alarm interrupt (AIE) */
    bool  mask_sec;                /* AEN: true = don't match this field */
    bool  mask_min;
    bool  mask_hour;
    bool  mask_day;
    bool  mask_weekday;
    pcf85063a_time_t time;         /* sec/min/hour/day/weekday used */
} pcf85063a_alarm_t;

esp_err_t pcf85063a_init(i2c_master_dev_handle_t dev);
esp_err_t pcf85063a_get_time(i2c_master_dev_handle_t dev, pcf85063a_time_t *t);
esp_err_t pcf85063a_set_time(i2c_master_dev_handle_t dev, const pcf85063a_time_t *t);

/* Alarm. One hardware alarm; multiple alarms must be handled in software. */
esp_err_t pcf85063a_set_alarm(i2c_master_dev_handle_t dev, const pcf85063a_alarm_t *alarm);
esp_err_t pcf85063a_clear_alarm(i2c_master_dev_handle_t dev);
esp_err_t pcf85063a_alarm_triggered(i2c_master_dev_handle_t dev, bool *triggered);

/* Countdown timer (seconds; also 1/4s, 1min, 1hr sources available). */
esp_err_t pcf85063a_set_timer_seconds(i2c_master_dev_handle_t dev, uint16_t seconds, bool enable_int);
/* Countdown timer using the 1/min clock, so it can exceed 255 s (snooze). */
esp_err_t pcf85063a_set_timer_minutes(i2c_master_dev_handle_t dev, uint8_t minutes, bool enable_int);
esp_err_t pcf85063a_timer_stop(i2c_master_dev_handle_t dev);
esp_err_t pcf85063a_timer_triggered(i2c_master_dev_handle_t dev, bool *triggered);
esp_err_t pcf85063a_timer_flag_clear(i2c_master_dev_handle_t dev);

/* Frequency offset trimming (0x02). value is the raw 7-bit offset. */
esp_err_t pcf85063a_set_offset(i2c_master_dev_handle_t dev, uint8_t raw7);

/* Debug: dump Control_1/2 and the alarm + timer registers (0x00-0x11). */
void pcf85063a_debug_dump(i2c_master_dev_handle_t dev);

/* General-purpose RAM byte (0x03), survives backup battery. */
esp_err_t pcf85063a_write_ram(i2c_master_dev_handle_t dev, uint8_t val);
esp_err_t pcf85063a_read_ram(i2c_master_dev_handle_t dev, uint8_t *val);

#ifdef __cplusplus
}
#endif
