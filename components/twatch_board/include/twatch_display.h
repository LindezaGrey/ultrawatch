#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/* Initialize the V1.0 CO5300 over the schematic QSPI pins. */
esp_err_t twatch_display_init(void);

/* Draw a bounded four-quadrant RGB565 pattern for physical bring-up. */
esp_err_t twatch_display_test_pattern(void);

/* Draw a deterministic tile/grid image for photographing rounded-edge cutoff. */
esp_err_t twatch_display_geometry_test(void);

/* Fill the complete active window with one RGB565 color for transfer diagnosis. */
esp_err_t twatch_display_test_solid(uint16_t color);

/* Flush an RGB565 rectangle using the initialized CO5300 QSPI path. */
esp_err_t twatch_display_flush(uint16_t x, uint16_t y, uint16_t width,
                               uint16_t height, const void *pixels);

/* Cycle MADCTL orientations over the already-rendered frame for bring-up. */
esp_err_t twatch_display_rotation_demo(void);

/* Exercise panel sleep/wake while leaving PMU rails enabled. */
esp_err_t twatch_display_sleep_wake_demo(void);
