#pragma once

#include <stdbool.h>
#include <stdint.h>

bool cst9217_resolution_is_valid(uint16_t x, uint16_t y);
bool cst9217_point_is_valid(uint16_t x, uint16_t y, uint16_t max_x, uint16_t max_y);
