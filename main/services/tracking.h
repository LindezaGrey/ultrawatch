/*
 * services/tracking.h - step-gated distance tracking (lifetime distance + steps).
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* TEMPORARILY DEACTIVATED while GNSS is being debugged (the step-gated pulsing
 * would fight the "GNSS always on" mode). Set back to 1 to re-enable. */
#ifndef TRACKING_ENABLED
#define TRACKING_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Lifetime tracking totals (persisted in NVS, namespace "track"). */
typedef struct {
    uint32_t dist_cm;   /* lifetime distance travelled in centimetres */
    uint32_t steps;     /* lifetime steps counted during tracking sessions */
} tracking_totals_t;

/* Init: load persisted totals and create the background tracking task.
 * No-op while TRACKING_ENABLED is 0. */
void tracking_init(void);

/* Start/stop a tracking session. Session steps are measured from the step
 * counter at start; GNSS is pulsed every TRACK_STEPS_PER_FIX steps while the
 * BHI activity class is walking or running (vehicle/cycling/still gates the
 * pulses). Return ESP_ERR_NOT_SUPPORTED while TRACKING_ENABLED is 0. */
esp_err_t tracking_start(void);
esp_err_t tracking_stop(void);
bool tracking_is_active(void);

/* True when tracking is active AND the current BHI activity is walking or
 * running (i.e. GNSS pulses + distance are being gated on). */
bool tracking_is_gated_active(void);

/* Estimated current position (degrees): the last session fix projected by the
 * steps taken since it, along the last GNSS course, using the lifetime average
 * step length. Returns false if no session fix exists yet. */
bool tracking_get_estimated_position(double *lat, double *lon);

/* Called by the GNSS control task when a tracking fix arrives (lat/lon deg).
 * Accumulates haversine distance and updates the persisted last position. */
void tracking_on_fix(double lat, double lon);

/* True when the step threshold since the last fix has been reached and a GNSS
 * position fix is due. Polled by the GNSS control task each loop. Always false
 * while TRACKING_ENABLED is 0. */
bool tracking_fix_due(void);

/* Clear the fix-due flag without recording a position (used when the GNSS
 * timed out before a fix; distance is not accumulated for that interval). */
void tracking_fix_clear(void);

/* Current session + lifetime stats for the UI. */
void tracking_get_totals(tracking_totals_t *totals);

/* Steps counted in the current session so far (0 when inactive). */
uint32_t tracking_get_session_steps(void);

/* Average step length in centimetres (lifetime dist/steps), 0 if no steps. */
uint32_t tracking_get_avg_step_cm(void);

#ifdef __cplusplus
}
#endif
