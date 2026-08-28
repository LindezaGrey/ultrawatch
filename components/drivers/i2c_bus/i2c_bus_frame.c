#include "i2c_bus_frame.h"
#include <string.h>

bool i2c_bus_build_frame(uint8_t reg, const uint8_t *buf, size_t len, uint8_t *out)
{
    if (!buf && len > 0) {
        return false;
    }
    out[0] = reg;
    if (len > 0) {
        memcpy(out + 1, buf, len);
    }
    return true;
}
