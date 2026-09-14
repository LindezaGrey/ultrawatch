#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "twatch_board.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Acquire both verified display interfaces once to validate registration. */
bool twatch_brookesia_bridge_ready(void);
bool twatch_brookesia_bridge_start(void);
bool twatch_brookesia_bridge_draw(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2,
                                  const uint8_t *data);
bool twatch_brookesia_bridge_read_touch(twatch_touch_sample_t *sample);

#ifdef __cplusplus
}
#endif
