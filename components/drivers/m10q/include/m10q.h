/*
 * m10q.h - u-blox MIA-M10Q GNSS receiver (UART, NMEA-0183).
 *
 * Uses the Espressif UART driver; NMEA parsing is implemented in this
 * component (GGA/RMC). TX 43 -> GNSS RX, RX 44 <- GNSS TX, PPS 13.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M10Q_PIN_TX  43
#define M10Q_PIN_RX  44
#define M10Q_PIN_PPS 13

typedef struct {
    bool     valid;
    double   lat;
    double   lon;
    double   alt_m;
    uint16_t sat_count;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} m10q_fix_t;

esp_err_t m10q_init(void);
esp_err_t m10q_get_fix(m10q_fix_t *fix);

#ifdef __cplusplus
}
#endif
