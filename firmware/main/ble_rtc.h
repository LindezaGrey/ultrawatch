#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

/* Configure the PCF85063A for 24-hour mode without changing its time. */
esp_err_t ble_rtc_initialize(void);

/* Start the UltraWatch RTC GATT service and advertising. */
esp_err_t ble_rtc_start(void);

/* Format the RTC as YYYY-MM-DDTHH:MM:SS for BLE and the display. */
esp_err_t ble_rtc_get_time_payload(char *output, size_t output_size);

/* Format power as percent,millivolts,direction,vbus,present. */
esp_err_t ble_power_get_payload(char *output, size_t output_size);

/* Return whether new BLE connections are currently being advertised. */
bool ble_rtc_advertising_enabled(void);
