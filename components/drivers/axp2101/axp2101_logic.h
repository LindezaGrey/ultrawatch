#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Pure AXP2101 value conversions, kept separate so host tests can exercise
 * range checks and register encodings without ESP-IDF or hardware. */
bool axp2101_voltage_code(uint16_t mv, uint16_t min_mv, uint16_t max_mv,
                          uint16_t step_mv, uint8_t code_mask, uint8_t *code);
bool axp2101_charge_current_code(uint16_t ma, uint8_t *code);
bool axp2101_charge_current_ma(uint8_t code, uint16_t *ma);
bool axp2101_die_temperature_c10(uint16_t raw, int16_t *tenths_c);
