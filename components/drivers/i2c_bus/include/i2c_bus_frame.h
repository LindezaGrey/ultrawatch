/*
 * i2c_bus_frame.h - pure register-write frame construction, no ESP-IDF deps.
 *
 * Builds the exact byte sequence i2c_bus_write() sends over the bus: the
 * register address followed by the payload. Split out from i2c_bus.h/.c so
 * this part is host-testable with zero I2C/hardware dependency -
 * i2c_bus_write() is a thin wrapper that picks where `out` points (a stack
 * buffer for small writes, a heap one for BHI260AP's larger transfers) and
 * then calls i2c_master_transmit(); none of that is exercised here.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Compute the full register-plus-payload frame length. Returns false if
 * frame_len is NULL or len + 1 cannot be represented by size_t. */
bool i2c_bus_frame_size(size_t len, size_t *frame_len);

/* Writes reg into out[0] and buf[0..len) into out[1..len+1) - out must
 * point at a buffer of at least len+1 bytes. Returns false (out left
 * untouched) if out is NULL, buf is NULL while len > 0, or len + 1
 * overflows size_t. len == 0 with buf == NULL is valid. */
bool i2c_bus_build_frame(uint8_t reg, const uint8_t *buf, size_t len, uint8_t *out);

#ifdef __cplusplus
}
#endif
