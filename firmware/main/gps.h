#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool ready;
    bool fix_valid;
    uint8_t fix_type;
    uint8_t satellites;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    uint32_t horizontal_accuracy_mm;
    uint32_t ground_speed_mm_s;
    int32_t heading_e5;
    uint16_t agc;
} gps_status_t;

/* Probe and configure the powered MIA-M10Q, then start 1 Hz NAV-PVT parsing. */
esp_err_t gps_initialize(void);

/* Abort a probe/configuration that is still in progress. */
void gps_cancel_initialize(void);

/* Stop parsing, release UART1, and clear the latest receiver state. */
esp_err_t gps_deinitialize(void);

/* Copy the latest receiver/fix status. Returns false until the driver starts. */
bool gps_get_status(gps_status_t *status);
