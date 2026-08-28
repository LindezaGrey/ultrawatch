/*
 * test_i2c_bus_frame.c - host tests for i2c_bus_frame.c's pure write-frame
 * construction (no ESP-IDF dependency, no framework: plain asserts).
 *
 * Build/run: see README.md in this directory.
 */
#include "i2c_bus_frame.h"
#include <stdio.h>
#include <string.h>

static int g_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        g_failures++; \
    } \
} while (0)

/* One-byte payload: register address in out[0], payload byte in out[1]. */
static void test_single_byte_payload(void)
{
    uint8_t out[2] = { 0xAA, 0xAA };
    bool ok = i2c_bus_build_frame(0x10, (const uint8_t[]){ 0x42 }, 1, out);
    CHECK(ok);
    CHECK(out[0] == 0x10);
    CHECK(out[1] == 0x42);
}

/* Register-only write: len == 0, buf may legitimately be NULL. Only out[0]
 * (the register byte) gets written. */
static void test_zero_length_null_buf(void)
{
    uint8_t out[1] = { 0xAA };
    bool ok = i2c_bus_build_frame(0x2B, NULL, 0, out);
    CHECK(ok);
    CHECK(out[0] == 0x2B);
}

/* Invalid: NULL buf with len > 0 must be rejected, and out must be left
 * completely untouched (not even the register byte) - a caller checking the
 * return value should never mistake a partially-written buffer for success. */
static void test_null_buf_nonzero_len_rejected(void)
{
    uint8_t out[4] = { 0x11, 0x22, 0x33, 0x44 };
    bool ok = i2c_bus_build_frame(0x99, NULL, 3, out);
    CHECK(!ok);
    CHECK(out[0] == 0x11 && out[1] == 0x22 && out[2] == 0x33 && out[3] == 0x44);
}

/* Multi-byte payload matching a real caller's shape (pcf85063a.c's 7-byte
 * time write): every byte lands at the right offset, not just the ends. */
static void test_multibyte_payload_layout(void)
{
    const uint8_t payload[7] = { 0x00, 0x30, 0x21, 0x28, 0x03, 0x08, 0x26 };
    uint8_t out[8] = { 0 };
    bool ok = i2c_bus_build_frame(0x04, payload, sizeof(payload), out);
    CHECK(ok);
    CHECK(out[0] == 0x04);
    CHECK(memcmp(out + 1, payload, sizeof(payload)) == 0);
}

int main(void)
{
    test_single_byte_payload();
    test_zero_length_null_buf();
    test_null_buf_nonzero_len_rejected();
    test_multibyte_payload_layout();

    if (g_failures) {
        printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
