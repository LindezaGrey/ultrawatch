#include "cst9217_logic.h"

bool cst9217_resolution_is_valid(uint16_t x, uint16_t y)
{
    return x == 410 && y == 502;
}

bool cst9217_validate_points(const cst9217_point_t *points, uint8_t count,
                             uint16_t max_x, uint16_t max_y)
{
    if ((!points && count > 0) || !max_x || !max_y) {
        return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (points[i].x >= max_x || points[i].y >= max_y) {
            return false;
        }
    }
    return true;
}
