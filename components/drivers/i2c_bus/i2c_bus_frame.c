#include "i2c_bus_frame.h"
#include <string.h>

bool i2c_bus_frame_size(size_t len, size_t *frame_len)
{
    if (!frame_len || len == SIZE_MAX) {
        return false;
    }
    *frame_len = len + 1;
    return true;
}

bool i2c_bus_build_frame(uint8_t reg, const uint8_t *buf, size_t len, uint8_t *out)
{
    size_t frame_len;
    if (!out || (!buf && len > 0) || !i2c_bus_frame_size(len, &frame_len)) {
        return false;
    }
    (void)frame_len;
    out[0] = reg;
    if (len > 0) {
        memcpy(out + 1, buf, len);
    }
    return true;
}
