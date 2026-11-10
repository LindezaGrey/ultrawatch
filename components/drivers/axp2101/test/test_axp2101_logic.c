#include "axp2101_logic.h"
#include <stdio.h>

static int failures;
#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); failures++; \
} } while (0)

static void test_rail_voltage_codes(void)
{
    uint8_t code = 0xAA;
    CHECK(axp2101_voltage_code(500, 500, 3500, 100, 0x1F, &code) && code == 0);
    CHECK(axp2101_voltage_code(3500, 500, 3500, 100, 0x1F, &code) && code == 30);
    CHECK(!axp2101_voltage_code(3600, 500, 3500, 100, 0x1F, &code));
    CHECK(!axp2101_voltage_code(499, 500, 3500, 100, 0x1F, &code));
    CHECK(!axp2101_voltage_code(551, 500, 3500, 100, 0x1F, &code));
    CHECK(axp2101_voltage_code(3400, 500, 3400, 100, 0x1F, &code) && code == 29);
    CHECK(!axp2101_voltage_code(3500, 500, 3400, 100, 0x1F, &code));
    CHECK(!axp2101_voltage_code(4000, 500, 4000, 100, 0x1F, &code));
    CHECK(!axp2101_voltage_code(500, 500, 3500, 0, 0x1F, &code));
    CHECK(!axp2101_voltage_code(500, 500, 3500, 100, 0x1F, NULL));
}

static void test_charge_current_mapping(void)
{
    static const uint16_t expected_ma[] = {
        0, 0, 0, 0, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
    };
    uint16_t ma = 0xFFFF;
    for (uint8_t code = 0; code < sizeof(expected_ma) / sizeof(expected_ma[0]); ++code) {
        CHECK(axp2101_charge_current_ma(code, &ma) && ma == expected_ma[code]);
    }
    CHECK(!axp2101_charge_current_ma(17, &ma));
    CHECK(!axp2101_charge_current_ma(0, NULL));

    uint8_t code = 0xFF;
    CHECK(axp2101_charge_current_code(0, &code) && code == 0);
    CHECK(axp2101_charge_current_code(25, &code) && code == 0);
    CHECK(axp2101_charge_current_code(49, &code) && code == 0);
    CHECK(axp2101_charge_current_code(50, &code) && code == 4);
    CHECK(axp2101_charge_current_code(75, &code) && code == 4);
    CHECK(axp2101_charge_current_code(100, &code) && code == 4);
    CHECK(axp2101_charge_current_code(112, &code) && code == 4);
    CHECK(axp2101_charge_current_code(113, &code) && code == 5);
    CHECK(axp2101_charge_current_code(250, &code) && code == 9);
    CHECK(axp2101_charge_current_code(350, &code) && code == 10);
    CHECK(axp2101_charge_current_code(500, &code) && code == 11);
    CHECK(axp2101_charge_current_code(1000, &code) && code == 16);
    CHECK(axp2101_charge_current_code(1500, &code) && code == 16);
    CHECK(!axp2101_charge_current_code(500, NULL));
}

static void test_die_temperature_conversion(void)
{
    int16_t temp = 0;
    CHECK(axp2101_die_temperature_c10(7274, &temp) && temp == 220);
    CHECK(axp2101_die_temperature_c10(7272, &temp) && temp == 221);
    CHECK(axp2101_die_temperature_c10(7276, &temp) && temp == 219);
    CHECK(!axp2101_die_temperature_c10(0x4000, &temp));
    CHECK(!axp2101_die_temperature_c10(0, NULL));
}

int main(void)
{
    test_rail_voltage_codes();
    test_charge_current_mapping();
    test_die_temperature_conversion();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    puts("all checks passed");
    return 0;
}
