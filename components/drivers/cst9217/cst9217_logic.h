#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t track_id;
    uint16_t x;
    uint16_t y;
    uint16_t strength;
} cst9217_point_t;

bool cst9217_resolution_is_valid(uint16_t x, uint16_t y);
/* Reject null arrays with nonzero count, zero geometry, or any out-of-range
 * coordinate. The application can adapt vendor touch points to this type. */
bool cst9217_validate_points(const cst9217_point_t *points, uint8_t count,
                             uint16_t max_x, uint16_t max_y);
