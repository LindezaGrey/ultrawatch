/*
 * i2c_bus.h - shared register read/write over i2c_master_dev_handle_t.
 *
 * Replaces the near-identical read_regs()/write_regs() statics duplicated
 * across the board's I2C drivers (axp2101.c, pcf85063a.c, drv2605.c,
 * xl9555.c, bhi260ap.c). Deliberately free functions over the plain
 * ESP-IDF handle, not a wrapper struct: several drivers' register-level
 * functions are called externally with the raw i2c_master_dev_handle_t
 * (e.g. main/debug_cmds.c's direct axp2101_read_reg/write_reg calls), so
 * keeping the handle type unchanged means only each driver's own .c file
 * needs to change - no public signature anywhere else moves.
 *
 * No retry logic: no driver today shows evidence of needing it, and
 * ESP-IDF's i2c_master driver already serializes bus access internally
 * (concurrent callers across FreeRTOS tasks are already correctness-safe).
 * Each call site passes its own timeout explicitly (most drivers use
 * ~100ms; BHI260AP's larger FIFO/firmware-upload transfers use 1000ms) so
 * the difference stays visible and named at the call site instead of a
 * silent copy-pasted magic number.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Read `len` bytes starting at register `reg` (8-bit register address,
 * write-then-read on one bus transaction). */
esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev, uint8_t reg,
                        uint8_t *buf, size_t len, uint32_t timeout_ms);

/* Write `len` bytes starting at register `reg` (register address + payload
 * in one bus transaction). Small writes (<=8 bytes - every driver except
 * BHI260AP) go on the stack; larger ones (BHI260AP's FIFO/firmware-upload
 * transfers, up to a few hundred bytes) heap-allocate the combined buffer,
 * same as bhi260ap.c's own write callback did before this extraction.
 * Returns ESP_ERR_NO_MEM if the heap allocation fails, ESP_ERR_INVALID_ARG
 * if buf is NULL while len > 0. The byte-layout logic itself is the pure,
 * host-tested i2c_bus_build_frame() in i2c_bus_frame.h. */
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev, uint8_t reg,
                         const uint8_t *buf, size_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
