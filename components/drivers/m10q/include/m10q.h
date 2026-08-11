/*
 * m10q.h - u-blox MIA-M10Q GNSS receiver (UART, NMEA-0183).
 *
 * Uses the Espressif UART driver; NMEA parsing (GGA/RMC/GSV) is implemented
 * in this component. TX 43 -> GNSS RX, RX 44 <- GNSS TX, PPS 13.
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

#ifdef __cplusplus
extern "C" {
#endif

#define M10Q_PIN_TX  43
#define M10Q_PIN_RX  44
#define M10Q_PIN_PPS 13

#define M10Q_MAX_SATS 16

/* L1 user-equivalent range error for HDOP-derived horizontal accuracy. */
#define M10Q_UERE_M   5

typedef enum {
    M10Q_STATE_OFF = 0,      /* powered down (rail off) */
    M10Q_STATE_ACQUIRING,    /* powered on, no valid fix yet */
    M10Q_STATE_FIXED,        /* valid fix */
} m10q_state_t;

typedef struct {
    uint8_t prn;             /* satellite PRN */
    int16_t elevation_deg;   /* 0..90 */
    int16_t azimuth_deg;     /* 0..359 */
    int16_t snr_db;          /* C/N0, -1 = not tracked */
    bool    used;            /* used in the current fix */
} m10q_sat_t;

typedef struct {
    bool     valid;
    double   lat;            /* degrees, N+ */
    double   lon;            /* degrees, E+ */
    double   alt_m;
    uint16_t sat_count;      /* satellites used in the fix */
    uint16_t sat_in_view;
    uint16_t speed_kmh;      /* ground speed from RMC, knots -> km/h */
    uint16_t course_deg;     /* track angle 0..359 */
    uint16_t hdop;           /* HDOP * 10 */
    uint16_t hacc_m;         /* estimated horizontal accuracy = HDOP * UERE */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    m10q_sat_t sats[M10Q_MAX_SATS];
} m10q_fix_t;

/* Initialize the receiver state (no UART, rail off). pmu is the AXP2101 I2C
 * device handle used to control the BLDO1 rail. */
esp_err_t m10q_init(i2c_master_dev_handle_t pmu);

/* Power the receiver on (BLDO1 rail + UART) or off (UBX soft-standby, then
 * rail off). Off keeps the VRTC backup alive for a fast warm/hot start. */
esp_err_t m10q_power(bool on);

/* Latest fix + satellites. Returns ESP_OK (fix->valid may be false). */
esp_err_t m10q_get_fix(m10q_fix_t *fix);

/* Current receiver power/fix state. */
m10q_state_t m10q_get_state(void);

/* Debug counters: bytes received and NMEA lines parsed since power-on. */
void m10q_get_dbg(uint32_t *rx_bytes, uint32_t *nmea_lines);

/* Debug: number of GSV sentences seen since power-on. */
uint32_t m10q_get_gsv_count(void);

#ifdef __cplusplus
}
#endif
