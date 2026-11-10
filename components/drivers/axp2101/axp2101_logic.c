#include "axp2101_logic.h"

bool axp2101_voltage_code(uint16_t mv, uint16_t min_mv, uint16_t max_mv,
                          uint16_t step_mv, uint8_t code_mask, uint8_t *code)
{
    if (!code || !step_mv || mv < min_mv || mv > max_mv ||
        (mv - min_mv) % step_mv != 0) {
        return false;
    }
    uint16_t value = (mv - min_mv) / step_mv;
    if (value > code_mask) {
        return false;
    }
    *code = (uint8_t)value;
    return true;
}

static const uint16_t s_charge_currents_ma[] = {
    0, 0, 0, 0, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
};

bool axp2101_charge_current_ma(uint8_t code, uint16_t *ma)
{
    if (!ma || code >= sizeof(s_charge_currents_ma) / sizeof(s_charge_currents_ma[0])) {
        return false;
    }
    *ma = s_charge_currents_ma[code];
    return true;
}

bool axp2101_charge_current_code(uint16_t ma, uint8_t *code)
{
    if (!code) {
        return false;
    }

    /* The chip supports 0 mA and 100..1000 mA. Choose the nearest supported
     * setting; ties round upward. Requests above the supported maximum clamp. */
    uint8_t selected = 0;
    uint16_t best_delta = ma;
    for (uint8_t candidate = 4; candidate <= 16; ++candidate) {
        uint16_t supported = s_charge_currents_ma[candidate];
        uint16_t delta = ma > supported ? ma - supported : supported - ma;
        if (delta < best_delta || (delta == best_delta && candidate > selected)) {
            selected = candidate;
            best_delta = delta;
        }
    }
    if (ma > 1000) {
        selected = 16;
    }
    *code = selected;
    return true;
}

bool axp2101_die_temperature_c10(uint16_t raw, int16_t *tenths_c)
{
    if (!tenths_c || raw > 0x3fff) {
        return false;
    }
    /* T(C) = 22.0 + (7274 - raw) / 20.0. Round to nearest 0.1 C. */
    int32_t numerator = 440 + (7274 - (int32_t)raw);
    int32_t value = numerator >= 0 ? (numerator + 1) / 2 : (numerator - 1) / 2;
    if (value < INT16_MIN || value > INT16_MAX) {
        return false;
    }
    *tenths_c = (int16_t)value;
    return true;
}
