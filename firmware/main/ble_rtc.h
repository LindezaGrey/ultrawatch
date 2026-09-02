#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

typedef struct {
    int year;
    int month;
    int day;
    int weekday;
    int hour;
    int minute;
    int second;
} rtc_datetime_t;

typedef struct {
    uint8_t hour;
    uint8_t minute;
    bool enabled;
} alarm_config_t;

typedef enum {
    WATCH_ACTIVITY_STILL = 0,
    WATCH_ACTIVITY_WALKING,
    WATCH_ACTIVITY_RUNNING,
    WATCH_ACTIVITY_CYCLING,
    WATCH_ACTIVITY_IN_VEHICLE,
    WATCH_ACTIVITY_TILTING,
    WATCH_ACTIVITY_OTHER,
    WATCH_ACTIVITY_COUNT,
} watch_activity_class_t;

typedef struct {
    bool ready;
    uint32_t steps_today;
    watch_activity_class_t current_activity;
    uint32_t activity_seconds[WATCH_ACTIVITY_COUNT];
} watch_activity_snapshot_t;

/* Configure the PCF85063A for 24-hour mode without changing its time. */
esp_err_t ble_rtc_initialize(void);

/* Start the UltraWatch RTC GATT service and advertising. */
esp_err_t ble_rtc_start(void);

/* Format the RTC as YYYY-MM-DDTHH:MM:SS for BLE and the display. */
esp_err_t ble_rtc_get_time_payload(char *output, size_t output_size);

/* Read and validate the complete PCF85063A calendar representation. */
esp_err_t ble_rtc_get_datetime(rtc_datetime_t *datetime);

/* Read today's Bosch step count and activity-recognition durations. */
esp_err_t ble_activity_get_snapshot(watch_activity_snapshot_t *snapshot);

/* Format power as percent,millivolts,direction,vbus,present. */
esp_err_t ble_power_get_payload(char *output, size_t output_size);

/* Return whether new BLE connections are currently being advertised. */
bool ble_rtc_advertising_enabled(void);

/* Enable advertising, or disable it and terminate an active connection. */
esp_err_t ble_rtc_set_advertising_enabled(bool enabled);

/* Play one short DRV2605 click for an on-watch UI control. */
esp_err_t ble_haptic_click(void);

/* Temporarily keep GPS active without changing its stored sensor setting. */
esp_err_t ble_rtc_acquire_gps_lease(void);
void ble_rtc_release_gps_lease(void);

/* Read or set the one daily RTC alarm. Sound and vibration are always used. */
void ble_alarm_get_config(alarm_config_t *config);
esp_err_t ble_alarm_set(uint8_t hour, uint8_t minute, bool enabled);
bool ble_alarm_is_ringing(void);
esp_err_t ble_alarm_dismiss(void);
