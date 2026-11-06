#include "cst9217_logic.h"
#include <stddef.h>
#include <stdio.h>

static int failures;
#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); failures++; \
} } while (0)

static void test_coordinate_bounds(void)
{
    CHECK(cst9217_resolution_is_valid(410, 502));
    CHECK(!cst9217_resolution_is_valid(0, 502));
    CHECK(!cst9217_resolution_is_valid(410, 0));
    CHECK(!cst9217_resolution_is_valid(480, 480));
    CHECK(!cst9217_resolution_is_valid(UINT16_MAX, UINT16_MAX));
    CHECK(cst9217_point_is_valid(0, 0, 410, 502));
    CHECK(cst9217_point_is_valid(409, 501, 410, 502));
    CHECK(!cst9217_point_is_valid(410, 0, 410, 502));
    CHECK(!cst9217_point_is_valid(0, 502, 410, 502));
    CHECK(!cst9217_point_is_valid(0, 0, 0, 502));
    CHECK(!cst9217_point_is_valid(0, 0, 410, 0));
}

int main(void)
{
    test_coordinate_bounds();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    puts("all checks passed");
    return 0;
}
