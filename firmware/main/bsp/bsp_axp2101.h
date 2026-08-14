#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the PMU: enable ALDO2 (display power) and battery ADC measures */
esp_err_t bsp_axp2101_init(void);

bool bsp_axp2101_battery_connected(void);
/* Battery voltage in millivolts, 0 if not connected */
uint16_t bsp_axp2101_battery_voltage_mv(void);
/* Battery percent from voltage curve (3.2V..4.2V), 0..100, 0xFF if not connected */
uint8_t bsp_axp2101_battery_percent(void);

/* Enable/disable the ALDO1 rail (SD card power) */
esp_err_t bsp_axp2101_set_aldo1(bool enable);

#ifdef __cplusplus
}
#endif
