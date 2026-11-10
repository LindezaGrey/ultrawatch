#include "xl9555_logic.h"

#define XL_REG_OUTP0 0x02
#define XL_REG_OUTP1 0x03
#define XL_REG_CFG0  0x06
#define XL_REG_CFG1  0x07

bool xl9555_pin_registers(uint8_t pin, uint8_t *config_reg,
                          uint8_t *output_reg, uint8_t *bit)
{
    if (pin >= 16 || !config_reg || !output_reg || !bit) {
        return false;
    }
    bool port1 = pin >= 8;
    *config_reg = port1 ? XL_REG_CFG1 : XL_REG_CFG0;
    *output_reg = port1 ? XL_REG_OUTP1 : XL_REG_OUTP0;
    *bit = (uint8_t)(1u << (pin % 8));
    return true;
}
