#include "cst9217_logic.h"

bool cst9217_resolution_is_valid(uint16_t x, uint16_t y)
{
    return x == 410 && y == 502;
}

bool cst9217_point_is_valid(uint16_t x, uint16_t y, uint16_t max_x, uint16_t max_y)
{
    return max_x > 0 && max_y > 0 && x < max_x && y < max_y;
}
