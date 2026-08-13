/*
 * m10q.c - u-blox MIA-M10Q GNSS receiver (UART, NMEA-0183).
 *
 * A UART RX task parses NMEA sentences: GGA (position/altitude/sats/HDOP),
 * RMC (validity/speed/course/time) and GSV (per-satellite azimuth/elevation/
 * SNR, multi-sentence per constellation). The receiver is powered via the
 * AXP2101 BLDO1 rail only on demand; the always-on VRTC backup keeps its RTC
 * and ephemeris alive so power-ups are warm/hot starts.
 */
#include "m10q.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "axp2101.h"
#include "pcf85063a.h"

static const char *TAG = "m10q";

#define M10Q_UART_NUM     UART_NUM_1
#define M10Q_BAUD_INIT    38400   /* factory default of the MIA-M10Q */
#define M10Q_BAUD_RUN     115200  /* raised after probing (RAM-only) */
#define M10Q_RX_BUF       512
#define M10Q_TX_BUF       0
#define M10Q_EVT_QUEUE    16
#define M10Q_RX_TASK_STACK 4096
#define M10Q_LINE_MAX     128

/* Only persist the last-known position when the fix moved this far from the
 * stored one (avoids NVS writes every second while stationary). */
#define LKP_UPDATE_MIN_M  50

/* NVS persistence (stats + last position for aiding). */
#define M10Q_NVS_NS       "m10q"
#define NVS_KEY_TOTAL     "gps_total"
#define NVS_KEY_TFSUM     "gps_tfsum"
#define NVS_KEY_TFBEST    "gps_tfbest"
#define NVS_KEY_TODAY     "gps_today"
#define NVS_KEY_DAY       "gps_day"
#define NVS_KEY_LAT       "gps_lat"
#define NVS_KEY_LON       "gps_lon"

/* M10 Generation-9 config keys (UBX-21035062). */
#define CFG_UART1_BAUDRATE     0x40520001u
#define CFG_SIGNAL_GPS_ENA     0x1031001Fu
#define CFG_SIGNAL_GAL_ENA     0x10310021u
#define CFG_SIGNAL_BDS_ENA     0x10310022u
#define CFG_SIGNAL_BDS_B1_ENA  0x1031000Du
#define CFG_SIGNAL_BDS_B1C_ENA 0x1031000Fu
#define CFG_SIGNAL_QZSS_ENA    0x10310024u
#define CFG_SIGNAL_SBAS_ENA    0x10310020u
#define CFG_SIGNAL_GLO_ENA     0x10310025u
#define CFG_ANA_USE_ANA        0x10230001u
#define CFG_ANA_ORBMAXERR      0x30230002u
#define CFG_NAVSPG_FIXMODE     0x20110011u
#define CFG_NAVSPG_INFIL_MINSVS 0x201100A1u
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007u

static i2c_master_dev_handle_t s_pmu;
static i2c_master_dev_handle_t s_rtc;
static bool s_powered;
static bool s_uart_installed;
static volatile bool s_probing;   /* set during baud probe; RX task idles */
static m10q_state_t s_state = M10Q_STATE_OFF;
static m10q_fix_t s_fix;
static QueueHandle_t s_uart_queue;
static uint32_t s_rx_bytes;      /* debug: bytes received since power-on */
static uint32_t s_nmea_lines;   /* debug: NMEA lines parsed since power-on */
static uint32_t s_gsv_count;    /* debug: GSV sentences seen */
static volatile bool s_dump_raw;   /* debug: print every NMEA line received */

/* Stats + TTFF tracking. */
static m10q_stats_t s_stats;
static uint32_t s_ttf_sum_ms;
static uint32_t s_ttf_start_ms;
static bool s_ttf_running;
static bool s_had_fix;
static bool s_rtc_synced;    /* RTC synced from GPS once per power-on */
static uint16_t s_agc;          /* last MON-RF AGC counter */

static void note_fix(void);     /* defined below (stats/TTFF) */
static void note_fix_if_first(void);
static void stats_load(void);
static void send_utc_time_aid(void);

/* ---- Distance / last-known-position ---- */

/* Great-circle distance in metres between two WGS84 points (haversine). The
 * same formula used by GPS libraries (e.g. TinyGPSPlus); see docs note. */
double m10q_distance_m(double lat1, double lon1, double lat2, double lon2)
{
    const double rad = 0.017453292519943295;   /* pi / 180 */
    double dlat = (lat2 - lat1) * rad;
    double dlon = (lon2 - lon1) * rad;
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1 * rad) * cos(lat2 * rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    return 6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

/* Stored last-known position (NVS), 1e7 fixed point. */
static void lkp_load(int32_t *lat, int32_t *lon)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, NVS_KEY_LAT, lat);
        nvs_get_i32(h, NVS_KEY_LON, lon);
        nvs_close(h);
    }
}

/* Persist the given fix as the last-known position, but only when it moved
 * more than LKP_UPDATE_MIN_M from the stored one, so a stationary receiver
 * does not rewrite NVS on every fix (wear) while still tracking movement.
 * No-op when the fix is not valid. */
static void lkp_persist_if_moved(double lat, double lon)
{
    nvs_handle_t wh;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &wh) != ESP_OK) {
        return;
    }
    int32_t old_lat = 0, old_lon = 0;
    lkp_load(&old_lat, &old_lon);
    double d = m10q_distance_m(old_lat / 1e7, old_lon / 1e7, lat, lon);
    if (d >= LKP_UPDATE_MIN_M) {
        nvs_set_i32(wh, NVS_KEY_LAT, (int32_t)(lat * 1e7));
        nvs_set_i32(wh, NVS_KEY_LON, (int32_t)(lon * 1e7));
        ESP_LOGI(TAG, "last position updated (moved %.0f m)", d);
    }
    nvs_commit(wh);
    nvs_close(wh);
}

/* ---- UBX RX parser (for NAV-PVT accuracy) ----
 * Byte-fed state machine for u-blox binary frames. Only UBX-NAV-PVT is
 * consumed (for the measured horizontal accuracy); everything else is
 * ignored. */
#define UBX_MAX_PAYLOAD 128

typedef struct {
    uint8_t state;      /* 0=idle,1=sync2,2=class,3=id,4=lenL,5=lenH,6=payload,7=ckA,8=ckB */
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint16_t idx;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t ck_a;
    uint8_t ck_b;
} ubx_rx_t;

static ubx_rx_t s_ubx_rx;

static void ubx_rx_reset(void)
{
    memset(&s_ubx_rx, 0, sizeof(s_ubx_rx));
}

/* True while the byte stream is inside (or starting) a UBX frame (sync seen,
 * not yet past the final checksum). Used to keep UBX binary bytes out of the
 * NMEA line buffer. */
static bool ubx_rx_in_frame(void)
{
    return s_ubx_rx.state >= 1;
}

/* Feed one byte; returns true when a complete frame has been delivered. */
static bool ubx_rx_feed(uint8_t b)
{
    switch (s_ubx_rx.state) {
    case 0:
        if (b == 0xB5) s_ubx_rx.state = 1;
        break;
    case 1:
        if (b == 0x62) {
            s_ubx_rx.state = 2;
        } else {
            s_ubx_rx.state = (b == 0xB5) ? 1 : 0;
        }
        break;
    case 2:
        s_ubx_rx.cls = b;
        s_ubx_rx.ck_a = 0;
        s_ubx_rx.ck_b = 0;
        s_ubx_rx.ck_a += b;
        s_ubx_rx.ck_b += s_ubx_rx.ck_a;
        s_ubx_rx.state = 3;
        break;
    case 3:
        s_ubx_rx.id = b;
        s_ubx_rx.ck_a += b;
        s_ubx_rx.ck_b += s_ubx_rx.ck_a;
        s_ubx_rx.state = 4;
        break;
    case 4:
        s_ubx_rx.len = b;
        s_ubx_rx.ck_a += b;
        s_ubx_rx.ck_b += s_ubx_rx.ck_a;
        s_ubx_rx.state = 5;
        break;
    case 5:
        s_ubx_rx.len |= (uint16_t)b << 8;
        s_ubx_rx.idx = 0;
        s_ubx_rx.ck_a += b;
        s_ubx_rx.ck_b += s_ubx_rx.ck_a;
        s_ubx_rx.state = (s_ubx_rx.len == 0) ? 7 : 6;
        break;
    case 6:
        if (s_ubx_rx.idx < UBX_MAX_PAYLOAD) {
            s_ubx_rx.payload[s_ubx_rx.idx] = b;
        }
        s_ubx_rx.ck_a += b;
        s_ubx_rx.ck_b += s_ubx_rx.ck_a;
        s_ubx_rx.idx++;
        if (s_ubx_rx.idx >= s_ubx_rx.len) s_ubx_rx.state = 7;
        break;
    case 7:
        if (b == s_ubx_rx.ck_a) {
            s_ubx_rx.state = 8;
        } else {
            s_ubx_rx.state = (b == 0xB5) ? 1 : 0;
        }
        break;
    case 8:
        if (b == s_ubx_rx.ck_b) {
            ubx_rx_reset();
            return true;
        }
        s_ubx_rx.state = 0;
        break;
    default:
        s_ubx_rx.state = 0;
        break;
    }
    return false;
}

/* UBX-NAV-PVT (class 0x01, id 0x07): the authoritative fix. Extracts hAcc
 * (bytes 40..43, mm), fix validity, position, SVs, and UTC time. On the first
 * valid fix after power-on, syncs the RTC and records TTFF. */
static void ubx_handle_pvt(void)
{
    if (s_ubx_rx.len < 92) {
        return;
    }
    uint8_t *p = s_ubx_rx.payload;

    uint32_t hacc_mm = (uint32_t)p[40] | ((uint32_t)p[41] << 8) |
                       ((uint32_t)p[42] << 16) | ((uint32_t)p[43] << 24);
    s_fix.hacc_m = (uint16_t)((hacc_mm + 500) / 1000);

    uint8_t fix_type = p[20];   /* 0=none, 1=DR, 2=2D, 3=3D */
    uint8_t flags = p[21];      /* bit0 = gnssFixOK */
    bool fix_ok = (flags & 0x01) && (fix_type == 2 || fix_type == 3);

    if (fix_ok) {
        int32_t lon = (int32_t)((uint32_t)p[24] | ((uint32_t)p[25] << 8) |
                                ((uint32_t)p[26] << 16) | ((uint32_t)p[27] << 24));
        int32_t lat = (int32_t)((uint32_t)p[28] | ((uint32_t)p[29] << 8) |
                                ((uint32_t)p[30] << 16) | ((uint32_t)p[31] << 24));
        int32_t hmsl = (int32_t)((uint32_t)p[36] | ((uint32_t)p[37] << 8) |
                                 ((uint32_t)p[38] << 16) | ((uint32_t)p[39] << 24));
        s_fix.lat = (double)lat / 1e7;
        s_fix.lon = (double)lon / 1e7;
        s_fix.alt_m = (double)hmsl / 1000.0;
        s_fix.sat_count = p[23];
        s_fix.hour = p[8];
        s_fix.minute = p[9];
        s_fix.second = p[10];
        s_fix.valid = true;
        s_fix.fix_3d = (fix_type == 3);
        s_state = M10Q_STATE_FIXED;

        /* First valid fix since power-on: record stats + persist LKP. */
        note_fix_if_first();

        /* Sync the RTC from the first PVT frame that carries a valid UTC date
         * (independent of which sentence reported the first fix). */
        if (!s_rtc_synced && (p[11] & 0x03)) {   /* validDate | validTime */
            /* PVT time is UTC; the RTC stores local wall time. Convert
             * using the configured TZ (set in app_main). */
            struct tm utc = { 0 };
            utc.tm_year = (int)((uint16_t)p[4] | ((uint16_t)p[5] << 8)) - 1900;
            utc.tm_mon = p[6] - 1;
            utc.tm_mday = p[7];
            utc.tm_hour = p[8];
            utc.tm_min = p[9];
            utc.tm_sec = p[10];
            utc.tm_isdst = 0;
            time_t epoch = timegm(&utc);
            struct tm local;
            localtime_r(&epoch, &local);
            pcf85063a_time_t t;
            memset(&t, 0, sizeof(t));
            t.year = (uint16_t)(local.tm_year + 1900);
            t.month = (uint8_t)(local.tm_mon + 1);
            t.day = (uint8_t)local.tm_mday;
            t.hour = (uint8_t)local.tm_hour;
            t.min = (uint8_t)local.tm_min;
            t.sec = (uint8_t)local.tm_sec;
            if (s_rtc && pcf85063a_set_time(s_rtc, &t) == ESP_OK) {
                s_rtc_synced = true;
                ESP_LOGI(TAG, "RTC synced from GPS: %04u-%02u-%02u %02u:%02u:%02u local",
                         (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
                         (unsigned)t.hour, (unsigned)t.min, (unsigned)t.sec);
            }
        }
    } else {
        /* Losing a fix: allow a new TTFF measurement on the next lock. */
        if (s_had_fix) {
            s_had_fix = false;
            s_ttf_running = true;
            s_ttf_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        }
    }
}

static void ubx_handle_mon_rf(void)
{
    if (s_ubx_rx.len >= 20) {
        /* Block 0: ... noisePerMS(2) agcCnt(2) at payload[18..19]. */
        s_agc = (uint16_t)(s_ubx_rx.payload[18] | (s_ubx_rx.payload[19] << 8));
    }
}

/* NOTE: the MIA-M10Q does not answer UBX-MON-RF (neither the poll nor the
 * CFG-MSGOUT periodic output; it returns no frame at all). s_agc therefore
 * stays 0 and AGC is not a usable RF-health signal on this module. The MON-RF
 * poll checksum was wrong (0x76 vs 0xD0) and has been fixed, but the module
 * still does not respond, so RF diagnostics are unavailable. */

static void ubx_rx_dispatch(void)
{
    if (s_ubx_rx.cls == 0x01 && s_ubx_rx.id == 0x07) {
        ubx_handle_pvt();
    } else if (s_ubx_rx.cls == 0x0A && s_ubx_rx.id == 0x38) {
        ubx_handle_mon_rf();
    }
}

/* GSV sentence merging: GSV frames arrive as N sentences per constellation;
 * a full sky scan is a cycle GP->GA->GB->GQ->GP... The M10 emits the GPS
 * constellation as TWO consecutive GSV frames per scan (tracked sats with SNR,
 * then in-view sats without), so a same-talker msg_num==1 is NOT a new scan.
 * Only reset when the first talker reappears after a different talker. */
static uint16_t s_gsv_sat_count;
static char s_gsv_first_talker;   /* first constellation talker of the scan */
static char s_gsv_last_talker;    /* talker of the previous GSV sentence */

/* ---- UBX helpers (for soft-standby on power-down) ---- */

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, size_t len)
{
    if (!s_powered) {
        return;
    }
    uint8_t frame[M10Q_LINE_MAX + 8];
    size_t n = 0;
    frame[n++] = 0xB5;
    frame[n++] = 0x62;
    frame[n++] = cls;
    frame[n++] = id;
    frame[n++] = (uint8_t)(len & 0xFF);
    frame[n++] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(frame + n, payload, len);
    n += len;
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < n; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    frame[n++] = ck_a;
    frame[n++] = ck_b;
    uart_write_bytes(M10Q_UART_NUM, frame, n);
}

/* Probe the UART baud rate. The MIA-M10Q answers at 38400 (factory), 115200
 * (if a previous RAM config raised it and the module kept power) or 9600.
 * We detect a live receiver by watching for the UBX sync pair (0xB5 0x62) in
 * response to a UBX-MON-VER request. Returns the found baud or 0. */
static uint32_t m10q_probe_baud(void)
{
    static const uint32_t probe_bauds[] = { M10Q_BAUD_INIT, M10Q_BAUD_RUN, 9600 };
    s_probing = true;
    for (size_t pb = 0; pb < sizeof(probe_bauds) / sizeof(probe_bauds[0]); pb++) {
        uart_set_baudrate(M10Q_UART_NUM, probe_bauds[pb]);
        vTaskDelay(pdMS_TO_TICKS(200));
        /* Ask for the module version; a live module answers with a UBX
         * frame. NMEA chatter is ignored. */
        for (int retry = 0; retry < 3; retry++) {
            uart_flush_input(M10Q_UART_NUM);
            uint8_t req[] = { 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };
            uart_write_bytes(M10Q_UART_NUM, req, sizeof(req));
            uint32_t deadline = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) + 1500;
            bool seen_sync = false;
            /* Require the full UBX sync pair: a single stray 0xB5 in a baud-
             * garbled stream must not be mistaken for a live receiver (that
             * locked the UART to the wrong speed and silenced the module). */
            uint8_t prev = 0;
            while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) < deadline) {
                uint8_t b = 0;
                if (uart_read_bytes(M10Q_UART_NUM, &b, 1, pdMS_TO_TICKS(50)) > 0) {
                    if (prev == 0xB5 && b == 0x62) {
                        seen_sync = true;
                        break;
                    }
                    prev = b;
                }
            }
            if (seen_sync) {
                ESP_LOGI(TAG, "module responds at %lu baud", (unsigned long)probe_bauds[pb]);
                s_probing = false;
                return probe_bauds[pb];
            }
        }
    }
    s_probing = false;
    return 0;
}

/* UBX-CFG-PM2 (0x06,0x3B): cyclic power save (on/off with acquisition
 * tracking). */
static void ubx_cfg_pm2(void)
{
    uint8_t pm2[44];
    memset(pm2, 0, sizeof(pm2));
    pm2[0] = 0x00;            /* version */
    pm2[1] = 0x00;
    pm2[4] = 0x01;            /* power setup request */
    pm2[6] = 0x00;            /* no flags */
    /* update/search periods, grid offset, on time, min acq time = 0 (default) */
    ubx_send(0x06, 0x3B, pm2, sizeof(pm2));
}

/* UBX-CFG-VALSET (0x06,0x8A): enable UBX-NAV-PVT output on UART1 (1 Hz) so we
 * get the receiver's measured horizontal accuracy (hAcc). RAM-only config. */
static void ubx_cfg_enable_pvt(void)
{
    /* CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007, value 1 (u8). */
    uint8_t payload[9] = {
        0x00,                   /* version */
        0x01,                   /* layer: RAM */
        0x00, 0x00,             /* position */
        0x07, 0x00, 0x91, 0x20, /* key (LE) */
        0x01,                   /* value: enabled */
    };
    ubx_send(0x06, 0x8A, payload, sizeof(payload));
}

/* UBX-RXM-PMREQ (0x06,0x41): request software standby (keeps backup RAM). */
static void ubx_rxm_pmreq(void)
{
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    payload[4] = 0x02;        /* flags: backup */
    ubx_send(0x06, 0x41, payload, sizeof(payload));
}

/* ---- Config (CFG-VALSET/GET) ---- */

static esp_err_t ubx_cfg_set_u8(uint32_t key, uint8_t value)
{
    uint8_t payload[9] = { 0x00, 0x01, 0x00, 0x00,
                           (uint8_t)key, (uint8_t)(key >> 8),
                           (uint8_t)(key >> 16), (uint8_t)(key >> 24),
                           value };
    ubx_send(0x06, 0x8A, payload, sizeof(payload));
    return ESP_OK;
}

static esp_err_t ubx_cfg_set_u16(uint32_t key, uint16_t value)
{
    uint8_t payload[10] = { 0x00, 0x01, 0x00, 0x00,
                            (uint8_t)key, (uint8_t)(key >> 8),
                            (uint8_t)(key >> 16), (uint8_t)(key >> 24),
                            (uint8_t)value, (uint8_t)(value >> 8) };
    ubx_send(0x06, 0x8A, payload, sizeof(payload));
    return ESP_OK;
}

/* ---- MGA-INI aiding (time + position for fast TTFF) ---- */

/* Send UBX-MGA-INI TIME_UTC (type 0x10, 32 bytes) from the RTC (local time). */
static void send_utc_time_aid(void)
{
    pcf85063a_time_t t;
    if (s_rtc && pcf85063a_get_time(s_rtc, &t) == ESP_OK) {
        struct tm local = { 0 };
        local.tm_year = (int)t.year - 1900;
        local.tm_mon = (int)t.month - 1;
        local.tm_mday = (int)t.day;
        local.tm_hour = (int)t.hour;
        local.tm_min = (int)t.min;
        local.tm_sec = (int)t.sec;
        local.tm_isdst = -1;
        time_t epoch = mktime(&local);
        if (epoch == (time_t)-1) {
            return;
        }
        struct tm utc;
        gmtime_r(&epoch, &utc);

        /* UBX-MGA-INI-TIME_UTC (type 0x10), 24-byte payload:
         * 0 type, 1 version, 2 ref, 3 leapSecs, 4-5 year (U2), 6 month,
         * 7 day, 8 hour, 9 minute, 10 second, 11 bitfield0,
         * 12-15 ns (U4), 16-17 tAccS (U2), 18-19 reserved, 20-23 tAccNs (U4). */
        uint8_t payload[24] = { 0 };
        payload[0] = 0x10;                 /* type: TIME_UTC */
        payload[1] = 0x00;                 /* version */
        payload[2] = 0x00;                 /* ref: apply on receipt */
        payload[3] = 18;                   /* leap seconds since 1980 (18 since 2017) */
        payload[4] = (uint8_t)(utc.tm_year + 1900);
        payload[5] = (uint8_t)((utc.tm_year + 1900) >> 8);
        payload[6] = (uint8_t)(utc.tm_mon + 1);
        payload[7] = (uint8_t)utc.tm_mday;
        payload[8] = (uint8_t)utc.tm_hour;
        payload[9] = (uint8_t)utc.tm_min;
        payload[10] = (uint8_t)utc.tm_sec;
        payload[16] = 1;                   /* tAccS: time accurate to 1 s */
        ubx_send(0x13, 0x40, payload, sizeof(payload));
        ESP_LOGI(TAG, "MGA-INI TIME_UTC sent (%04d-%02u-%02u %02u:%02u:%02u)",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec);
    }
}

/* Send UBX-MGA-INI POS_LLH (type 0x01, 20 bytes) from the last known fix. */
static void send_pos_llh_aid(void)
{
    nvs_handle_t h;
    int32_t lat = 0, lon = 0;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, NVS_KEY_LAT, &lat);
        nvs_get_i32(h, NVS_KEY_LON, &lon);
        nvs_close(h);
    }
    if (lat == 0 && lon == 0) {
        return;
    }
    uint8_t payload[20] = { 0 };
    payload[0] = 0x01;   /* type: POS_LLH */
    payload[1] = 0x00;
    payload[4] = lat & 0xFF;
    payload[5] = (lat >> 8) & 0xFF;
    payload[6] = (lat >> 16) & 0xFF;
    payload[7] = (lat >> 24) & 0xFF;
    payload[8] = lon & 0xFF;
    payload[9] = (lon >> 8) & 0xFF;
    payload[10] = (lon >> 16) & 0xFF;
    payload[11] = (lon >> 24) & 0xFF;
    ubx_send(0x13, 0x40, payload, sizeof(payload));
    ESP_LOGI(TAG, "MGA-INI POS_LLH sent");
}

/* Seed the receiver with an approximate position (degrees) via MGA-INI
 * POS_LLH, so it can compute satellite positions and acquire faster even
 * without stored ephemeris. Only needs to be within a few hundred km. */
void m10q_seed_position(double lat, double lon)
{
    if (!s_powered || !s_uart_installed) {
        return;
    }
    int32_t ilat = (int32_t)(lat * 1e7);
    int32_t ilon = (int32_t)(lon * 1e7);
    uint8_t payload[20] = { 0 };
    payload[0] = 0x01;   /* type: POS_LLH */
    payload[4] = ilat & 0xFF;
    payload[5] = (ilat >> 8) & 0xFF;
    payload[6] = (ilat >> 16) & 0xFF;
    payload[7] = (ilat >> 24) & 0xFF;
    payload[8] = ilon & 0xFF;
    payload[9] = (ilon >> 8) & 0xFF;
    payload[10] = (ilon >> 16) & 0xFF;
    payload[11] = (ilon >> 24) & 0xFF;
    ubx_send(0x13, 0x40, payload, sizeof(payload));
    ESP_LOGI(TAG, "MGA-INI POS_LLH seeded (%.5f, %.5f)", lat, lon);
}

/* ---- NVS stats persistence ---- */

static void stats_load(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_TOTAL, &s_stats.total_fixes);
        nvs_get_u32(h, NVS_KEY_TFSUM, &s_ttf_sum_ms);
        nvs_get_u32(h, NVS_KEY_TFBEST, &s_stats.ttf_best_ms);
        nvs_get_u32(h, NVS_KEY_TODAY, &s_stats.fixes_today);
        nvs_close(h);
    }
    if (s_stats.total_fixes > 0) {
        s_stats.ttf_avg_ms = s_ttf_sum_ms / s_stats.total_fixes;
    }
}

static void stats_save(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_TOTAL, s_stats.total_fixes);
        nvs_set_u32(h, NVS_KEY_TFSUM, s_ttf_sum_ms);
        nvs_set_u32(h, NVS_KEY_TFBEST, s_stats.ttf_best_ms);
        nvs_set_u32(h, NVS_KEY_TODAY, s_stats.fixes_today);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* Record a fix: TTFF, counters, today's date, last position. */
/* Record stats + persist LKP on the first valid fix since power-on, regardless
 * of which sentence reported it (GGA or PVT). The PVT handler already does this
 * in its own !s_had_fix block; this catches the case where GGA locks before
 * UBX-NAV-PVT is received, so the last-known position / stats are never lost. */
static void note_fix_if_first(void)
{
    if (!s_had_fix) {
        s_had_fix = true;
        note_fix();
    }
}

static void note_fix(void)
{
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t ttf_ms = 0;    if (s_ttf_running && now_ms >= s_ttf_start_ms) {
        ttf_ms = now_ms - s_ttf_start_ms;
    }
    s_ttf_running = false;

    uint32_t today = 0;
    pcf85063a_time_t t;
    if (s_rtc && pcf85063a_get_time(s_rtc, &t) == ESP_OK) {
        today = (uint32_t)(t.year) * 10000 + (uint32_t)(t.month) * 100 + t.day;
    }
    uint32_t stored_day = 0;
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_DAY, &stored_day);
        nvs_close(h);
    }

    s_stats.total_fixes++;
    if (today != 0 && today == stored_day) {
        s_stats.fixes_today++;
    } else if (today != 0) {
        s_stats.fixes_today = 1;
    }
    if (ttf_ms > 0) {
        s_ttf_sum_ms += ttf_ms;
        s_stats.ttf_avg_ms = s_ttf_sum_ms / s_stats.total_fixes;
        if (s_stats.ttf_best_ms == 0 || ttf_ms < s_stats.ttf_best_ms) {
            s_stats.ttf_best_ms = ttf_ms;
        }
    }
    stats_save();

    nvs_handle_t wh;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &wh) == ESP_OK) {
        lkp_persist_if_moved(s_fix.lat, s_fix.lon);
        if (today != 0) {
            nvs_set_u32(wh, NVS_KEY_DAY, today);
        }
        nvs_commit(wh);
        nvs_close(wh);
    }

    ESP_LOGI(TAG, "FIX: %d sats, TTFF %u ms (avg %lu, best %lu)",
             (int)s_fix.sat_count, ttf_ms,
             (unsigned long)s_stats.ttf_avg_ms, (unsigned long)s_stats.ttf_best_ms);
}

/* ---- NMEA line parsing ---- */

#define NMEA_MAX_FIELDS 24

/* Split a NMEA line into comma-separated fields in-place (commas become NUL),
 * returning the number of fields. field[0] is the talker+type token. */
static int nmea_split(char *line, char **field, int max)
{
    int n = 0;
    char *p = line;
    while (n < max) {
        field[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

static int nmea_int(char *s, int *out)
{
    if (!s || !*s) {
        return -1;
    }
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) {
        return -1;
    }
    *out = (int)v;
    return 0;
}

static int nmea_double(char *s, double *out)
{
    if (!s || !*s) {
        return -1;
    }
    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s) {
        return -1;
    }
    *out = v;
    return 0;
}

/* Convert ddmm.mmmm -> decimal degrees. */
static double nmea_latlon(double v, char hemi)
{
    double deg = (int)(v / 100.0);
    double min = v - deg * 100.0;
    double d = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') {
        d = -d;
    }
    return d;
}

static void parse_gga(char *line)
{
    /* $GxGGA,hhmmss.ss,lat,N,lon,E,quality,sats,hdop,alt,M,... */
    char *f[NMEA_MAX_FIELDS];
    int nf = nmea_split(line, f, NMEA_MAX_FIELDS);
    if (nf < 10) return;
    double t, lat, lon, alt, hdop;
    int quality = 0, sats = 0;
    if (nmea_double(f[1], &t) < 0) return;
    if (nmea_double(f[2], &lat) < 0) return;
    if (nmea_double(f[4], &lon) < 0) return;
    if (nmea_int(f[6], &quality) < 0) return;
    if (nmea_int(f[7], &sats) < 0) return;
    if (nmea_double(f[8], &hdop) < 0) return;
    if (nmea_double(f[9], &alt) < 0) return;
    if (!f[3] || !f[5]) return;

    s_fix.hdop = (uint16_t)(hdop * 10.0);
    s_fix.lat = nmea_latlon(lat, f[3][0]);
    s_fix.lon = nmea_latlon(lon, f[5][0]);
    s_fix.alt_m = alt;
    s_fix.sat_count = (uint16_t)sats;
    s_fix.hacc_m = (uint16_t)(hdop * M10Q_UERE_M);
    int hms = (int)t;
    s_fix.hour = (uint8_t)(hms / 10000);
    s_fix.minute = (uint8_t)((hms / 100) % 100);
    s_fix.second = (uint8_t)(hms % 100);
    s_fix.valid = (quality > 0);
    s_state = s_fix.valid ? M10Q_STATE_FIXED : M10Q_STATE_ACQUIRING;
    if (s_fix.valid) {
        note_fix_if_first();
    }
}

static void parse_rmc(char *line)
{
    /* $GxRMC,hhmmss.ss,A,lat,N,lon,E,speed,course,date,... */
    char *f[NMEA_MAX_FIELDS];
    int nf = nmea_split(line, f, NMEA_MAX_FIELDS);
    if (nf < 9 || !f[2] || f[2][0] != 'A') {
        return;   /* not valid */
    }
    double speed = 0, course = 0;
    if (nmea_double(f[7], &speed) < 0) return;
    nmea_double(f[8], &course);
    /* knots -> km/h */
    s_fix.speed_kmh = (uint16_t)(speed * 1.852 + 0.5);
    s_fix.course_deg = (uint16_t)(course + 0.5) % 360;
    s_fix.valid = true;
    s_state = M10Q_STATE_FIXED;
}

static void parse_gsv(char *line)
{
    /* $GxGSV,num_msgs,msg_num,sats_in_view,{prn,el,az,snr}*4
     * The receiver emits a separate multi-sentence GSV frame per constellation
     * (GP/GL/GA/GB/GQ), each starting with msg_num==1. A full sky scan is a
     * sequence GP->GA->GB->GQ->GP...; reset the accumulator when a new cycle
     * starts (talker wraps back to the first seen). */
    char *f[NMEA_MAX_FIELDS];
    int nf = nmea_split(line, f, NMEA_MAX_FIELDS);
    if (nf < 4) return;
    /* f[1]=num_msgs, f[2]=msg_num, f[3]=sats_in_view (may be "00"). */
    int msg_num = 0;
    if (nmea_int(f[2], &msg_num) < 0) return;
    /* Talker is f[0] = "$GPGSV" -> char index 2 (P/A/B/L/Q). Index 1 is the
     * leading 'G' shared by all GNSS constellations. */
    char talker = (f[0][0] == '$' && f[0][2] != '\0') ? f[0][2] : '\0';

    if (msg_num == 1) {
        if (s_gsv_first_talker == '\0') {
            s_gsv_first_talker = talker;
            s_gsv_sat_count = 0;
            s_fix.sat_in_view = 0;
        } else if (talker == s_gsv_first_talker && s_gsv_last_talker != talker) {
            /* Talker wrapped to the first after a different constellation:
             * a new sky-scan cycle. A same-talker msg_num==1 right after the
             * previous frame is NOT a new scan (the M10 sends two GPGSV frames
             * per scan), so it must not reset the accumulator. */
            s_gsv_sat_count = 0;
            s_fix.sat_in_view = 0;
        }
    }
    s_gsv_last_talker = talker;

    for (int i = 0; i < 4; i++) {
        int base = 4 + i * 4;
        if (base + 3 >= nf) {
            break;
        }
        /* A satellite block may be empty (e.g. "00" constellation frames or
         * short blocks with missing az/el): only accept blocks with a valid
         * PRN, and only count ones that carry elevation/azimuth (used for the
         * skyplot). Otherwise they would inflate sat_in_view with phantom sats
         * and plot dots at the skyplot centre. */
        int prn = 0;
        if (nmea_int(f[base], &prn) < 0 || prn == 0) {
            break;
        }
        int el = 0, az = 0, snr = 0;
        nmea_int(f[base + 1], &el);
        nmea_int(f[base + 2], &az);
        if (nmea_int(f[base + 3], &snr) < 0) {
            snr = -1;
        }
        if (s_gsv_sat_count < M10Q_MAX_SATS) {
            m10q_sat_t *s = &s_fix.sats[s_gsv_sat_count];
            s->prn = (uint8_t)prn;
            s->elevation_deg = (int16_t)el;
            s->azimuth_deg = (int16_t)az;
            s->snr_db = (int16_t)snr;
            s->used = false;
            s_gsv_sat_count++;
            s_fix.sat_in_view = s_gsv_sat_count;
        }
    }
}

static void parse_line(char *line)
{
    if (line[0] != '$') {
        return;
    }
    if (strstr(line, "GGA,")) {
        parse_gga(line);
    } else if (strstr(line, "RMC,")) {
        parse_rmc(line);
    } else if (strstr(line, "GSV,")) {
        s_gsv_count++;
        parse_gsv(line);
    }
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uart_event_t evt;
    char line[M10Q_LINE_MAX];
    size_t line_len = 0;
    for (;;) {
        if (!s_powered || !s_uart_queue) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (s_probing) {
            /* The baud probe is reading the UART directly; do not steal its
             * bytes (would miss the UBX sync and fall back to the wrong
             * baud). */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xQueueReceive(s_uart_queue, &evt, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }
        if (evt.type == UART_DATA) {
            uint8_t buf[256];
            int n = uart_read_bytes(M10Q_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
            if (n > 0) {
                s_rx_bytes += (uint32_t)n;
            }
            for (int i = 0; i < n; i++) {
                uint8_t b = buf[i];
                /* Feed the UBX binary parser (NAV-PVT for hAcc). UBX frames
                 * start with 0xB5 0x62; NMEA lines start with '$'. While a
                 * UBX frame is being consumed (sync seen through final
                 * checksum), its bytes must NOT enter the NMEA line buffer. */
                bool frame_done = ubx_rx_feed(b);
                if (frame_done) {
                    ubx_rx_dispatch();
                    continue;   /* final checksum byte consumed */
                }
                if (ubx_rx_in_frame()) {
                    continue;
                }
                char c = (char)b;
                if (c == '\n') {
                    if (line_len > 0) {
                        line[line_len] = '\0';
                        s_nmea_lines++;
                        parse_line(line);
                        line_len = 0;
                    }
                } else if (c != '\r' && line_len < M10Q_LINE_MAX - 1) {
                    line[line_len++] = c;
                }
            }
            if (s_dump_raw && n > 0) {
                /* Debug: print the raw chunk so we can inspect what the receiver
                 * actually sends (NMEA + UBX bytes). */
                printf("[m10q raw] ");
                for (int i = 0; i < n; i++) {
                    uint8_t b = buf[i];
                    if (b == '\n') {
                        printf("\\n\n[m10q raw] ");
                    } else if (b == '\r') {
                        printf("\\r");
                    } else {
                        putchar(b);
                    }
                }
                printf("\n");
            }
        } else if (evt.type == UART_FIFO_OVF || evt.type == UART_BUFFER_FULL) {
            uart_flush_input(M10Q_UART_NUM);
        }
    }
}

esp_err_t m10q_init(i2c_master_dev_handle_t pmu, i2c_master_dev_handle_t rtc)
{
    s_pmu = pmu;
    s_rtc = rtc;
    s_powered = false;
    s_state = M10Q_STATE_OFF;
    memset(&s_fix, 0, sizeof(s_fix));
    memset(&s_stats, 0, sizeof(s_stats));
    /* Persistent RX task; it blocks on the UART event queue (created when the
     * UART is installed on power-on) and survives power-off. */
    xTaskCreate(uart_rx_task, "m10q_rx", M10Q_RX_TASK_STACK, NULL, 6, NULL);
    return ESP_OK;
}

esp_err_t m10q_power(bool on)
{
    if (on == s_powered) {
        return ESP_OK;
    }
    if (on) {
        /* Install the UART driver only once. The persistent RX task blocks on
         * its event queue for the whole lifetime; deleting the driver on a
         * later power-off would free that queue under the running task and
         * crash (use-after-free). So the UART stays installed and only the
         * BLDO1 rail is toggled. */
        if (!s_uart_installed) {
            uart_config_t cfg = {
                .baud_rate = M10Q_BAUD_INIT,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
            };
            ESP_RETURN_ON_ERROR(uart_param_config(M10Q_UART_NUM, &cfg), TAG, "uart param");
            ESP_RETURN_ON_ERROR(uart_set_pin(M10Q_UART_NUM, M10Q_PIN_TX, M10Q_PIN_RX, -1, -1),
                                TAG, "uart pin");
            ESP_RETURN_ON_ERROR(uart_driver_install(M10Q_UART_NUM, M10Q_RX_BUF, M10Q_TX_BUF,
                                                    M10Q_EVT_QUEUE, &s_uart_queue, 0),
                                TAG, "uart install");
            s_uart_installed = true;
        }

        /* Power the receiver rail and let it boot. A cold M10 boot can take
         * up to ~1 s to start outputting; probing too early finds nothing and
         * leaves the UART silent until the next power cycle. */
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, true);
        vTaskDelay(pdMS_TO_TICKS(1000));

        s_powered = true;
        uint32_t baud = m10q_probe_baud();
        if (baud == 0) {
            /* Cold-boot race: the module may still be starting. Wait and probe
             * again once before falling back to the factory baud. */
            ESP_LOGW(TAG, "no UBX answer on first probe; retrying after 1 s");
            vTaskDelay(pdMS_TO_TICKS(1000));
            baud = m10q_probe_baud();
        }
        if (baud == 0) {
            ESP_LOGW(TAG, "no UBX answer at 38400/115200/9600; using NMEA at 38400");
            baud = M10Q_BAUD_INIT;
        } else if (baud != M10Q_BAUD_RUN) {
            /* Raise the UART speed (RAM-only config, re-applied each boot).
             * The module switches after the ACK. */
            uint8_t valset[] = {
                0x00, 0x01, 0x00, 0x00,                       /* version, layer RAM */
                0x01, 0x00, 0x52, 0x40,                       /* CFG_UART1_BAUDRATE */
                M10Q_BAUD_RUN & 0xFF, (M10Q_BAUD_RUN >> 8) & 0xFF,
                (M10Q_BAUD_RUN >> 16) & 0xFF, (M10Q_BAUD_RUN >> 24) & 0xFF,
            };
            ubx_send(0x06, 0x8A, valset, sizeof(valset));   /* CFG-VALSET */
            vTaskDelay(pdMS_TO_TICKS(100));
            uart_flush_input(M10Q_UART_NUM);
            uart_set_baudrate(M10Q_UART_NUM, M10Q_BAUD_RUN);
            ESP_LOGI(TAG, "UART raised to %lu baud", (unsigned long)M10Q_BAUD_RUN);
        }

        s_rx_bytes = 0;
        s_nmea_lines = 0;
        s_state = M10Q_STATE_ACQUIRING;
        memset(&s_fix, 0, sizeof(s_fix));
        s_gsv_sat_count = 0;
        s_gsv_first_talker = '\0';
        s_gsv_last_talker = '\0';
        ubx_cfg_pm2();
        ubx_cfg_enable_pvt();

        /* Configure constellations: GPS + GAL + BDS B1I + QZSS + SBAS.
         * GLONASS and BDS B1C stay off (B1I cannot run with B1C/GLO, and it
         * keeps AssistNow Autonomous usable). RAM-only. */
        struct { uint32_t key; uint8_t want; const char *name; } sig[] = {
            { CFG_SIGNAL_BDS_B1C_ENA, 0, "BDS_B1C" },
            { CFG_SIGNAL_GLO_ENA,     0, "GLO" },
            { CFG_SIGNAL_BDS_ENA,     1, "BDS" },
            { CFG_SIGNAL_BDS_B1_ENA,  1, "BDS_B1I" },
            { CFG_SIGNAL_GPS_ENA,     1, "GPS" },
            { CFG_SIGNAL_GAL_ENA,     1, "GAL" },
            { CFG_SIGNAL_QZSS_ENA,    1, "QZSS" },
            { CFG_SIGNAL_SBAS_ENA,    1, "SBAS" },
        };
        for (size_t i = 0; i < sizeof(sig) / sizeof(sig[0]); i++) {
            ubx_cfg_set_u8(sig[i].key, sig[i].want);
            vTaskDelay(pdMS_TO_TICKS(200));   /* GNSS restart settle */
        }
        ubx_cfg_set_u8(CFG_ANA_USE_ANA, 1);
        /* AssistNow Autonomous builds its predicted-ephemeris database from
         * stored time/position observations (M10 manages these internally). */
        ubx_cfg_set_u16(CFG_ANA_ORBMAXERR, 50);   /* 50 m max modelled orbit error */
        ubx_cfg_set_u8(CFG_NAVSPG_FIXMODE, 2);       /* auto 2D/3D */
        ubx_cfg_set_u8(CFG_NAVSPG_INFIL_MINSVS, 3);
        vTaskDelay(pdMS_TO_TICKS(300));

        /* RF diagnostics: poll AGC once. */
        uint8_t monrf[] = { 0xB5, 0x62, 0x0A, 0x38, 0x00, 0x00, 0x42, 0xD0 };
        uart_write_bytes(M10Q_UART_NUM, monrf, sizeof(monrf));

        /* Load stats and start a TTFF measurement. */
        stats_load();
        s_had_fix = false;
        s_rtc_synced = false;
        s_ttf_running = true;
        s_ttf_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* Time + position aiding for a fast first fix. */
        send_utc_time_aid();
        send_pos_llh_aid();

        ESP_LOGI(TAG, "powered on (BLDO1) at %lu baud", (unsigned long)baud);
    } else {
        /* Persist the freshest position before the rail is cut, so the next
         * power-on's position aiding (send_pos_llh_aid) seeds the right area.
         * lkp_persist_if_moved keeps the 50 m wear gate (no rewrite if the
         * watch has not moved since the stored position). */
        if (s_fix.valid) {
            lkp_persist_if_moved(s_fix.lat, s_fix.lon);
        }
        ubx_rxm_pmreq();   /* soft standby keeps backup RAM */
        vTaskDelay(pdMS_TO_TICKS(50));
        /* UART stays installed (see power-on); only cut the rail. */
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
        s_powered = false;
        s_state = M10Q_STATE_OFF;
        ESP_LOGI(TAG, "powered off (backup RAM kept)");
    }
    return ESP_OK;
}

esp_err_t m10q_get_fix(m10q_fix_t *fix)
{
    if (!fix) {
        return ESP_ERR_INVALID_ARG;
    }
    *fix = s_fix;
    return ESP_OK;
}

void m10q_get_last_position(double *lat, double *lon)
{
    int32_t ilat = 0, ilon = 0;
    lkp_load(&ilat, &ilon);
    if (lat) {
        *lat = ilat / 1e7;
    }
    if (lon) {
        *lon = ilon / 1e7;
    }
}

void m10q_update_last_position(double lat, double lon)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, NVS_KEY_LAT, (int32_t)(lat * 1e7));
        nvs_set_i32(h, NVS_KEY_LON, (int32_t)(lon * 1e7));
        nvs_commit(h);
        nvs_close(h);
    }
}

m10q_state_t m10q_get_state(void)
{
    return s_state;
}

void m10q_get_dbg(uint32_t *rx_bytes, uint32_t *nmea_lines)
{
    if (rx_bytes) {
        *rx_bytes = s_rx_bytes;
    }
    if (nmea_lines) {
        *nmea_lines = s_nmea_lines;
    }
}

uint32_t m10q_get_gsv_count(void)
{
    return s_gsv_count;
}

void m10q_set_raw_dump(bool on)
{
    s_dump_raw = on;
}

esp_err_t m10q_get_stats(m10q_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_stats.total_fixes == 0 && s_ttf_sum_ms == 0) {
        stats_load();
    }
    *stats = s_stats;
    return ESP_OK;
}

esp_err_t m10q_get_agc(uint16_t *agc)
{
    if (!agc) {
        return ESP_ERR_INVALID_ARG;
    }
    *agc = s_agc;
    return ESP_OK;
}

void m10q_poll_agc(void)
{
    if (!s_powered || !s_uart_installed) {
        return;
    }
    uint8_t monrf[] = { 0xB5, 0x62, 0x0A, 0x38, 0x00, 0x00, 0x42, 0xD0 };
    uart_write_bytes(M10Q_UART_NUM, monrf, sizeof(monrf));
}
