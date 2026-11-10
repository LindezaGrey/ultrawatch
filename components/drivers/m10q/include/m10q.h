/*
 * m10q.h - u-blox MIA-M10Q GNSS receiver (UART, UBX via ubxlib).
 *
 * ubxlib owns UART RX and parses the configured UBX messages. TX 43 -> GNSS
 * RX, RX 44 <- GNSS TX, PPS 13.
 *
 * Power: the receiver runs on AXP2101 BLDO1. It is normally powered off; the
 * always-on VRTC backup rail keeps the receiver's RTC + ephemeris alive at
 * ~28 uA, so a later power-up is a warm/hot start (fast TTFF).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "u_gnss_type.h"
#include "u_gnss_info.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M10Q_PIN_TX  43
#define M10Q_PIN_RX  44
#define M10Q_PIN_PPS 13

#include "m10q_types.h"

/* Cumulative fix statistics (persisted in NVS, namespace "m10q"). */
typedef struct {
    uint32_t total_fixes;    /* total number of fixes since first use */
    uint32_t fixes_today;    /* fixes recorded today (local date) */
    uint32_t ttf_avg_ms;     /* average time-to-first-fix */
    uint32_t ttf_best_ms;    /* best (shortest) time-to-first-fix */
} m10q_stats_t;

/* Initialize the receiver state (no UART, rail off). pmu is the AXP2101 I2C
 * device handle for the BLDO1 rail; rtc is the PCF85063A device handle used
 * to sync the RTC from GPS time. */
esp_err_t m10q_init(i2c_master_dev_handle_t pmu, i2c_master_dev_handle_t rtc);

/* Power the receiver on (BLDO1 rail + UART) or off (UBX soft-standby, then
 * rail off). Off keeps the VRTC backup alive for a fast warm/hot start. */
esp_err_t m10q_power(bool on);

/* Latest fix + satellites. Returns ESP_OK (fix->valid may be false). */
esp_err_t m10q_get_fix(m10q_fix_t *fix);

/* Last known position persisted in NVS (from a previous fix), degrees. */
void m10q_get_last_position(double *lat, double *lon);

/* Persist the given position as the last-known position (degrees). Used by
 * tracking to update it on every fix (bypasses the 50 m gate). */
void m10q_update_last_position(double lat, double lon);

/* Great-circle distance in metres between two WGS84 points (degrees), using
 * the haversine formula (the same one used by GPS libraries such as
 * TinyGPSPlus). */
double m10q_distance_m(double lat1, double lon1, double lat2, double lon2);

/* Current receiver power/fix state. */
m10q_state_t m10q_get_state(void);

/* Cumulative fix statistics. */
esp_err_t m10q_get_stats(m10q_stats_t *stats);

/* MON-RF is not supported by the MIA-M10Q; returns ESP_ERR_NOT_SUPPORTED. */
esp_err_t m10q_get_agc(uint16_t *agc);

/* Deprecated no-op: MON-RF diagnostics are not supported by this receiver. */
void m10q_poll_agc(void);

/* Seed the receiver with an approximate position (degrees) via MGA-INI
 * POS_LLH for faster acquisition when no ephemeris is stored. */
void m10q_seed_position(double lat, double lon);

/* Deprecated debug counters; currently return zero (raw UART access belongs to ubxlib). */
void m10q_get_dbg(uint32_t *rx_bytes, uint32_t *nmea_lines);

/* Debug: number of GSV sentences seen since power-on. */
uint32_t m10q_get_gsv_count(void);

/* Receiver navigation status (UBX-NAV-STATUS): fix type + time/position
 * validity flags. updated=false until the first message arrives. */
esp_err_t m10q_get_nav_status(m10q_nav_status_t *nav);

/* Deprecated no-op: ubxlib owns the UART RX stream, so raw dumping is unavailable. */
void m10q_set_raw_dump(bool on);

/* Force a PPS-based RTC drift calibration now (re-runs even if a previous
 * calibration already applied an offset this session). Needs a valid GNSS fix
 * so the MIA-M10Q emits its 1PPS signal on GPIO 13. */
void m10q_rtc_calibrate(void);

/* Read version info cached at power-on; never changes receiver power. */
esp_err_t m10q_get_versions(uGnssVersionType_t *ver);

#ifdef __cplusplus
}
#endif
