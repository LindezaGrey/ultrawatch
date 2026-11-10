#pragma once
#include "m10q_types.h"
#define GPS_NO_FIX_TIMEOUT_MS 300000u
#define GPS_ACQUISITION_LIMIT_MS 600000u
typedef enum { GPS_RECEPTION_OK, GPS_POOR_SIGNAL, GPS_NO_TELEMETRY, GPS_NO_FIX } gps_reception_reason_t;
typedef struct {
    uint32_t last_fix_ms, last_reception_ms, last_sats_ms;
    uint16_t moderate[5], strong[5];
    unsigned count, next;
    bool seen, good;
} gps_reception_t;
void gps_reception_start(gps_reception_t *state, uint32_t now_ms);
gps_reception_reason_t gps_reception_update(gps_reception_t *state, uint32_t now_ms, const m10q_fix_t *fix);
