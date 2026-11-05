#include "xl9555_logic.h"
#include <stddef.h>
#include <stdio.h>

static int failures;
#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); failures++; \
} } while (0)

static void test_pin_mapping(void)
{
    static const uint8_t expected_cfg[] = { 0x06, 0x06, 0x07, 0x07 };
    static const uint8_t expected_out[] = { 0x02, 0x02, 0x03, 0x03 };
    static const uint8_t expected_bit[] = { 0x01, 0x80, 0x01, 0x80 };
    static const uint8_t pins[] = { 0, 7, 8, 15 };
    for (size_t i = 0; i < sizeof(pins); ++i) {
        uint8_t cfg = 0, out = 0, bit = 0;
        CHECK(xl9555_pin_registers(pins[i], &cfg, &out, &bit));
        CHECK(cfg == expected_cfg[i]);
        CHECK(out == expected_out[i]);
        CHECK(bit == expected_bit[i]);
    }
}

static void test_invalid_arguments(void)
{
    uint8_t cfg = 0xAA, out = 0xBB, bit = 0xCC;
    CHECK(!xl9555_pin_registers(16, &cfg, &out, &bit));
    CHECK(cfg == 0xAA && out == 0xBB && bit == 0xCC);
    CHECK(!xl9555_pin_registers(0, NULL, &out, &bit));
    CHECK(!xl9555_pin_registers(0, &cfg, NULL, &bit));
    CHECK(!xl9555_pin_registers(0, &cfg, &out, NULL));
}

int main(void)
{
    test_pin_mapping();
    test_invalid_arguments();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    puts("all checks passed");
    return 0;
}
