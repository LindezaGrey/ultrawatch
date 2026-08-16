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

/* Configure the PCF85063A for 24-hour mode without changing its time. */
esp_err_t ble_rtc_initialize(void);

/* Start the UltraWatch RTC GATT service and advertising. */
esp_err_t ble_rtc_start(void);

/* Format the RTC as YYYY-MM-DDTHH:MM:SS for BLE and the display. */
esp_err_t ble_rtc_get_time_payload(char *output, size_t output_size);

/* Read and validate the complete PCF85063A calendar representation. */
esp_err_t ble_rtc_get_datetime(rtc_datetime_t *datetime);

/* Format power as percent,millivolts,direction,vbus,present. */
esp_err_t ble_power_get_payload(char *output, size_t output_size);

/* Return whether new BLE connections are currently being advertised. */
bool ble_rtc_advertising_enabled(void);

/* Enable or disable connectable advertising without dropping a connection. */
esp_err_t ble_rtc_set_advertising_enabled(bool enabled);

/* Play one short DRV2605 click for an on-watch UI control. */
esp_err_t ble_haptic_click(void);

/* Read or set the one daily RTC alarm. Sound and vibration are always used. */
void ble_alarm_get_config(alarm_config_t *config);
esp_err_t ble_alarm_set(uint8_t hour, uint8_t minute, bool enabled);
bool ble_alarm_is_ringing(void);
esp_err_t ble_alarm_dismiss(void);
