#pragma once
#include <stdbool.h>
#include <stdint.h>

#define M10Q_FRESH_MS 3000u

#define M10Q_MAX_SATS 24

/* L1 user-equivalent range error for HDOP-derived horizontal accuracy. */
#define M10Q_UERE_M   5

typedef enum {
    M10Q_STATE_OFF = 0,      /* powered down (rail off) */
    M10Q_STATE_ACQUIRING,    /* powered on, no valid fix yet */
    M10Q_STATE_FIXED,        /* valid fix */
} m10q_state_t;

typedef struct {
    uint8_t gnss_id;         /* UBX constellation identifier; PRNs are not globally unique. */
    uint8_t quality;
    bool healthy;
    uint8_t prn;             /* satellite PRN */
    int16_t elevation_deg;   /* 0..90 */
    int16_t azimuth_deg;     /* 0..359 */
    int16_t snr_db;          /* C/N0, -1 = not tracked */
    bool    used;            /* used in the current fix */
} m10q_sat_t;

typedef struct {
    bool     valid;
    uint32_t updated_ms;     /* Monotonic timestamp of the latest position callback. */
    bool     heading_valid;
    bool     time_valid;
    uint16_t year;
    uint8_t month, day;
    uint32_t pvt_itow;
    bool pvt_received;
    uint32_t sats_updated_ms, sats_itow;
    bool sats_valid;
    uint16_t sats_total, sats_above_25, sats_above_30;
    uint8_t best_cno;
    uint32_t dop_updated_ms, dop_itow;
    bool dop_valid;
    bool     fix_3d;            /* true when the fix is 3D (not just 2D) */
    double   lat;            /* degrees, N+ */
    double   lon;            /* degrees, E+ */
    double   alt_m;          /* NAN without a 3D fix. */
    uint16_t sat_count;      /* satellites used in the fix */
    uint16_t sat_in_view;
    uint16_t speed_kmh;      /* NAV-PVT ground speed, km/h */
    uint16_t course_deg;     /* track angle 0..359 */
    uint16_t hdop;           /* HDOP * 10; UINT16_MAX when unavailable */
    uint16_t hacc_m;         /* receiver horizontal accuracy estimate */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    int32_t  rtc_offset_s;     /* GPS time - RTC time (seconds), 0 if unknown */
    m10q_sat_t sats[M10Q_MAX_SATS];
} m10q_fix_t;

/* Receiver navigation status from UBX-NAV-STATUS (0x01 0x03), updated
 * asynchronously while powered on. updated=false until the first message. */
typedef struct {
    uint32_t updated_ms;
    bool     updated;
    uint8_t  gps_fix;         /* 0=no fix, 2=2D, 3=3D */
    bool     gps_fix_ok;      /* flags bit0 (valid fix available) */
    bool     wkns_set;        /* flags bit2 (GPS week number valid) */
    bool     tow_set;         /* flags bit3 (GPS time-of-week valid) */
    uint32_t ttff_ms;         /* module-reported last time-to-first-fix */
} m10q_nav_status_t;
