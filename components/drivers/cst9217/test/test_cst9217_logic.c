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
    const cst9217_point_t valid_points[] = {
        { .track_id = 1, .x = 0, .y = 0 },
        { .track_id = 2, .x = 409, .y = 501 },
    };
    CHECK(cst9217_validate_points(valid_points, 2, 410, 502));
    CHECK(cst9217_validate_points(NULL, 0, 410, 502));
    CHECK(!cst9217_validate_points(NULL, 1, 410, 502));
    const cst9217_point_t bad_point[] = {{ .x = 410, .y = 0 }};
    CHECK(!cst9217_validate_points(bad_point, 1, 410, 502));
    CHECK(!cst9217_validate_points(valid_points, 2, 0, 502));
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
