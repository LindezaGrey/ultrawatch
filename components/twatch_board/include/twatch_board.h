#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t status1, status2, adc_control, irq_status[3], poweroff_enable;
    bool usb_present, battery_present, charging, battery_voltage_available;
    uint8_t charger_state;
    uint16_t battery_mv;
} twatch_power_snapshot_t;

typedef struct {
    uint8_t control1;
    uint8_t seconds, minutes, hours, day, weekday, month, year;
    bool bcd_fields_plausible;
} twatch_rtc_snapshot_t;

typedef struct {
    uint8_t dcdc_control, dc1_voltage, ldo_control[2], aldo2_voltage;
    uint8_t expander_output[2], expander_polarity[2], expander_config[2];
    bool dc1_enabled, aldo2_enabled, dc1_setting_valid, aldo2_setting_valid;
    uint16_t dc1_configured_mv, aldo2_configured_mv;
    bool aldo4_enabled, aldo4_setting_valid;
    uint16_t aldo4_configured_mv;
    bool display_enable_output, display_enable_latch_high;
    bool touch_reset_output, touch_reset_latch_high;
} twatch_rail_snapshot_t;

// The app's main task is the sole caller. Board owns I2C0 and PMU handles
// for firmware lifetime. These functions do not write PMU configuration.
esp_err_t twatch_board_init(void);
// Output is replaced only after every register read succeeds.
esp_err_t twatch_board_read_power(twatch_power_snapshot_t *snapshot);
// Configured values/latches, not measured voltage or physical pin levels.
esp_err_t twatch_board_read_rails(twatch_rail_snapshot_t *snapshot);
// Verify required PMU settings, then drive only P07/P10 high. No PMU writes.
esp_err_t twatch_board_prepare_display_touch(void);

/* Probe documented CST9217 candidate addresses without writing registers. */
esp_err_t twatch_board_probe_touch(uint8_t *address, bool *present);
/* Read-only ACK probe for the schematic PCF85063 candidate at 0x51. */
esp_err_t twatch_board_probe_rtc(bool *present);
/* Read-only PCF85063 control/time registers; does not set or clear the clock. */
esp_err_t twatch_board_read_rtc(twatch_rtc_snapshot_t *snapshot);
/* Read-only ACK probe for the schematic BHI260AP candidate at 0x28. */
esp_err_t twatch_board_probe_bhi(bool *present);

/* Read the vendor CST92xx identity and geometry fields after address probing. */
esp_err_t twatch_board_read_touch_info(void);

typedef struct {
    bool pressed;
    uint16_t x;
    uint16_t y;
} twatch_touch_sample_t;

/* Poll one CST92xx packet; coordinates are native 410x502 panel coordinates. */
esp_err_t twatch_board_read_touch(twatch_touch_sample_t *sample);

#ifdef __cplusplus
}
#endif
