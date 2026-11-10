#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Resolve one linear 0..15 GPIO number to configuration/output registers and
 * its port bit. Pure helper for bounds and port-map tests. */
bool xl9555_pin_registers(uint8_t pin, uint8_t *config_reg,
                          uint8_t *output_reg, uint8_t *bit);
