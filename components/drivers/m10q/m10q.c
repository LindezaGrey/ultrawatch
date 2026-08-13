/*
 * m10q.c - u-blox MIA-M10Q GNSS receiver (UART, via ubxlib).
 *
 * Thin driver that wraps the u-blox ubxlib GNSS API (uDeviceOpen +
 * uGnssPwrOn + uGnssPosGet) and exposes a small cached fix/satellite API to
 * the rest of the firmware. TX 43 -> GNSS RX, RX 44 <- GNSS TX, PPS 13.
 *
 * Power: the receiver runs on AXP2101 BLDO1 (controlled via the pmu I2C
 * handle). The always-on VRTC backup rail keeps the receiver's RTC +
 * ephemeris alive at ~28 uA, so a later power-up is a warm/hot start.
 *
 * The skyplot satellite list is fed from UBX-NAV-SAT (class 0x01, id 0x35),
 * received asynchronously via uGnssMsgReceiveStart().
 */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ubxlib.h"
#include "u_gnss_module_type.h"
#include "u_gnss_cfg_val_key.h"
#include "m10q.h"
#include "axp2101.h"
#include "pcf85063a.h"

#define TAG "m10q"

#define M10Q_UART_NUM    1       /* ESP32 UART1 for the GNSS receiver */
#define M10Q_BAUD        115200  /* target UART speed (raised at init) */

/* ---- static state ---- */
static i2c_master_dev_handle_t s_pmu;
static i2c_master_dev_handle_t s_rtc;
static bool s_powered;
static bool s_opened;            /* uDeviceOpen done */
static uDeviceHandle_t s_gnss;   /* ubxlib GNSS device handle */

static m10q_fix_t s_fix;
static m10q_stats_t s_stats;
static uint32_t s_rx_bytes;      /* debug */
static uint32_t s_nmea_lines;    /* debug */
static uint32_t s_gsv_count;     /* debug */
static uint16_t s_agc;           /* always 0 (MIA-M10Q does not answer MON-RF) */
static volatile bool s_dump_raw;

static bool s_had_fix;
static uint32_t s_ttf_start_ms;
static bool s_ttf_running;
static uint32_t s_ttf_sum_ms;

/* ubxlib init-once guard. */
static bool s_ubxlib_init;

/* ---- NVS stats persistence ---- */
#define M10Q_NVS_NS       "m10q"
#define NVS_KEY_DAY       "day"
#define NVS_KEY_LAT       "gps_lat"
#define NVS_KEY_LON       "gps_lon"

static void lkp_load(int32_t *lat, int32_t *lon)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, NVS_KEY_LAT, lat);
        nvs_get_i32(h, NVS_KEY_LON, lon);
        nvs_close(h);
    }
}

static void lkp_persist(double lat, double lon)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, NVS_KEY_LAT, (int32_t)(lat * 1e7));
        nvs_set_i32(h, NVS_KEY_LON, (int32_t)(lon * 1e7));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void stats_save(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "fixes", s_stats.total_fixes);
        nvs_set_u32(h, "today", s_stats.fixes_today);
        nvs_set_u32(h, "ttfa", s_stats.ttf_avg_ms);
        nvs_set_u32(h, "ttfb", s_stats.ttf_best_ms);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void stats_load(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "fixes", &s_stats.total_fixes);
        nvs_get_u32(h, "today", &s_stats.fixes_today);
        nvs_get_u32(h, "ttfa", &s_stats.ttf_avg_ms);
        nvs_get_u32(h, "ttfb", &s_stats.ttf_best_ms);
        nvs_close(h);
    }
}

/* ---- Great-circle distance (haversine) ---- */
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

/* ---- UBX-NAV-SAT parser for the skyplot ----
 * Frame: class 0x01 id 0x35, payload:
 *   iTOW(4) version(1) numSvs(1) reserved(2), then per SV (12 bytes):
 *   gnssId(1) svId(1) cno(1) elev(1 i1) azim(2 i2) prRes(2) flags(4)
 * flags bit0 = used-in-solution. */
static void parse_nav_sat(const uint8_t *payload, size_t len)
{
    if (len < 8) {
        return;
    }
    uint8_t numSvs = payload[5];
    size_t off = 8;
    uint16_t count = 0;
    for (uint8_t i = 0; i < numSvs && off + 12 <= len && count < M10Q_MAX_SATS; i++, off += 12) {
        uint8_t svId = payload[off + 1];
        uint8_t cno = payload[off + 2];
        int8_t elev = (int8_t)payload[off + 3];
        int16_t azim = (int16_t)(payload[off + 4] | (payload[off + 5] << 8));
        uint32_t flags = (uint32_t)payload[off + 8] |
                         ((uint32_t)payload[off + 9] << 8) |
                         ((uint32_t)payload[off + 10] << 16) |
                         ((uint32_t)payload[off + 11] << 24);

        if (cno == 0 && elev == 0) {
            continue;   /* not visible/tracked */
        }
        m10q_sat_t *s = &s_fix.sats[count];
        s->prn = svId;
        s->elevation_deg = elev;
        s->azimuth_deg = azim;
        s->snr_db = (cno > 0) ? (int16_t)cno : -1;
        s->used = (flags & 0x01) != 0;
        count++;
    }
    s_fix.sat_in_view = count;
}

/* NAV-SAT message callback (ubxlib task context). */
static void nav_sat_cb(uDeviceHandle_t handle, const uGnssMessageId_t *pId,
                       int32_t errorCodeOrLength, void *pParam)
{
    (void)pParam;
    if (errorCodeOrLength <= 0 || pId == NULL ||
        pId->type != U_GNSS_PROTOCOL_UBX || pId->id.ubx != 0x0135) {
        return;
    }
    int32_t size = errorCodeOrLength;
    char *buf = (char *)heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (buf) {
        if (uGnssMsgReceiveCallbackRead(handle, buf, (size_t)size) == size) {
            /* payload starts after 6-byte header (sync2+class+id+len2) */
            parse_nav_sat((const uint8_t *)buf + 6, (size_t)size - 8);
        }
        heap_caps_free(buf);
    }
    s_gsv_count++;
}

/* Streamed position callback: updates s_fix on every PVT. Runs in the
 * ubxlib receiver task. */
static void pos_cb(uDeviceHandle_t gnssHandle, int32_t errorCode,
                   int32_t lat1e7, int32_t lon1e7, int32_t altMm,
                   int32_t radiusMm, int32_t speedMmS, int32_t svs,
                   int64_t timeUtc)
{
    (void)gnssHandle;
    if (errorCode != 0 || lat1e7 == 0 || lon1e7 == 0) {
        if (s_had_fix) {
            /* lost the fix: allow a new TTFF measurement next lock */
            s_had_fix = false;
            s_ttf_running = true;
            s_ttf_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        }
        s_fix.valid = false;
        return;
    }

    s_fix.lat = lat1e7 / 1e7;
    s_fix.lon = lon1e7 / 1e7;
    s_fix.alt_m = altMm / 1000.0;
    s_fix.sat_count = (svs > 0) ? (uint16_t)svs : 0;
    s_fix.hacc_m = (radiusMm > 0) ? (uint16_t)((radiusMm + 500) / 1000) : 0;
    s_fix.speed_kmh = (speedMmS > 0) ? (uint16_t)((speedMmS / 1000.0) * 3.6 + 0.5) : 0;
    s_fix.course_deg = 0;
    s_fix.valid = true;
    s_fix.fix_3d = (svs >= 3);
    s_fix.hdop = 0;

    if (timeUtc > 0) {
        time_t e = (time_t)(timeUtc / 1000);
        struct tm tm;
        gmtime_r(&e, &tm);
        s_fix.hour = (uint8_t)tm.tm_hour;
        s_fix.minute = (uint8_t)tm.tm_min;
        s_fix.second = (uint8_t)tm.tm_sec;
    }

    if (!s_had_fix) {
        s_had_fix = true;
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint32_t ttf_ms = (s_ttf_running && now_ms >= s_ttf_start_ms)
                          ? (now_ms - s_ttf_start_ms) : 0;
        s_ttf_running = false;
        s_stats.total_fixes++;
        if (ttf_ms > 0) {
            s_ttf_sum_ms += ttf_ms;
            s_stats.ttf_avg_ms = s_ttf_sum_ms / s_stats.total_fixes;
            if (s_stats.ttf_best_ms == 0 || ttf_ms < s_stats.ttf_best_ms) {
                s_stats.ttf_best_ms = ttf_ms;
            }
        }
        stats_save();
        ESP_LOGI(TAG, "FIX: %d sats, TTFF %u ms (avg %lu, best %lu)",
                 (int)s_fix.sat_count, ttf_ms,
                 (unsigned long)s_stats.ttf_avg_ms,
                 (unsigned long)s_stats.ttf_best_ms);
        int32_t olat = 0, olon = 0;
        lkp_load(&olat, &olon);
        if (m10q_distance_m(olat / 1e7, olon / 1e7, s_fix.lat, s_fix.lon) >= 50) {
            lkp_persist(s_fix.lat, s_fix.lon);
        }
    }
}

/* ---- Public API ---- */

esp_err_t m10q_init(i2c_master_dev_handle_t pmu, i2c_master_dev_handle_t rtc)
{
    s_pmu = pmu;
    s_rtc = rtc;
    s_powered = false;
    s_opened = false;
    memset(&s_fix, 0, sizeof(s_fix));
    stats_load();
    return ESP_OK;
}

esp_err_t m10q_power(bool on)
{
    if (on == s_powered) {
        return ESP_OK;
    }
    if (on) {
        if (!s_ubxlib_init) {
            /* Init the ubxlib port layer + device/GNSS APIs once. */
            if (uPortInit() != 0 || uDeviceInit() != 0 || uGnssInit() != 0) {
                ESP_LOGE(TAG, "ubxlib init failed");
                return ESP_ERR_INVALID_STATE;
            }
            s_ubxlib_init = true;
        }

        /* Power the receiver rail and let it boot. */
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, true);
        vTaskDelay(pdMS_TO_TICKS(1000));
        s_powered = true;

        if (!s_opened) {
            uDeviceCfg_t cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.deviceType = U_DEVICE_TYPE_GNSS;
            cfg.deviceCfg.cfgGnss.moduleType = U_GNSS_MODULE_TYPE_M10;
            cfg.deviceCfg.cfgGnss.pinEnablePower = -1;   /* rail via AXP */
            cfg.transportType = U_DEVICE_TRANSPORT_TYPE_UART;
            cfg.transportCfg.cfgUart.uart = M10Q_UART_NUM;
            cfg.transportCfg.cfgUart.baudRate = 38400;   /* factory baud; module reverts on power cycle */
            cfg.transportCfg.cfgUart.pinTxd = M10Q_PIN_TX;
            cfg.transportCfg.cfgUart.pinRxd = M10Q_PIN_RX;
            cfg.transportCfg.cfgUart.pinCts = -1;
            cfg.transportCfg.cfgUart.pinRts = -1;
            int32_t err = uDeviceOpen(&cfg, &s_gnss);
            if (err != 0) {
                /* The module may be mid-restart or at an unexpected baud
                 * (RAM/BBRAM config persists via VRTC). Power-cycle the rail
                 * and retry once before giving up. */
                ESP_LOGW(TAG, "uDeviceOpen failed (%d); power-cycling and retrying", (int)err);
                axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
                vTaskDelay(pdMS_TO_TICKS(500));
                axp2101_enable_rail(s_pmu, AXP2101_BLDO1, true);
                vTaskDelay(pdMS_TO_TICKS(1500));
                err = uDeviceOpen(&cfg, &s_gnss);
                if (err != 0) {
                    ESP_LOGE(TAG, "uDeviceOpen failed again: %d", (int)err);
                    s_powered = false;
                    axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
                    return ESP_ERR_INVALID_STATE;
                }
            }
            s_opened = true;
        }

        if (uGnssPwrOn(s_gnss) != 0) {
            ESP_LOGW(TAG, "uGnssPwrOn failed");
        }

        /* Enable UBX-NAV-SAT output on UART1 for the skyplot. */
        U_GNSS_CFG_SET_VAL_RAM(s_gnss, MSGOUT_UBX_NAV_SAT_UART1_U1, 1);

        /* Start UBX-NAV-SAT reception for the skyplot. */
        uGnssMessageId_t navSat = { .type = U_GNSS_PROTOCOL_UBX, .id.ubx = 0x0135 };
        uGnssMsgReceiveStart(s_gnss, &navSat, nav_sat_cb, NULL);

        /* Streamed position: callback fires on every PVT. */
        int32_t sret = uGnssPosGetStreamedStart(s_gnss, 1000, pos_cb);
        if (sret != 0) {
            ESP_LOGE(TAG, "uGnssPosGetStreamedStart failed: %d", (int)sret);
        }

        /* Reset TTFF state. */
        s_had_fix = false;
        s_ttf_running = true;
        s_ttf_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        s_fix.valid = false;
        ESP_LOGI(TAG, "powered on (BLDO1) at %u baud", (unsigned)M10Q_BAUD);
    } else {
        if (s_opened) {
            uGnssPosGetStreamedStop(s_gnss);
            uGnssMsgReceiveStopAll(s_gnss);
            uGnssPwrOff(s_gnss);
            uDeviceClose(s_gnss, false);
            s_opened = false;
        }
        /* Persist LKP if we have a fresh fix. */
        if (s_fix.valid) {
            int32_t olat = 0, olon = 0;
            lkp_load(&olat, &olon);
            if (m10q_distance_m(olat / 1e7, olon / 1e7, s_fix.lat, s_fix.lon) >= 50) {
                lkp_persist(s_fix.lat, s_fix.lon);
            }
        }
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
        s_powered = false;
        s_fix.valid = false;
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
    lkp_persist(lat, lon);
}

m10q_state_t m10q_get_state(void)
{
    if (!s_powered) {
        return M10Q_STATE_OFF;
    }
    return s_fix.valid ? M10Q_STATE_FIXED : M10Q_STATE_ACQUIRING;
}

esp_err_t m10q_get_stats(m10q_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
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
    /* MIA-M10Q does not answer UBX-MON-RF; s_agc stays 0. */
}

void m10q_seed_position(double lat, double lon)
{
    if (!s_opened) {
        return;
    }
    /* UBX-MGA-INI POS_LLH (class 0x13, id 0x40): seed an approximate position
     * so acquisition is faster. lat/lon in 1e-7 deg fixed point. */
    uint8_t payload[20] = { 0 };
    payload[0] = 0x01;   /* type: POS_LLH */
    int32_t ilat = (int32_t)(lat * 1e7);
    int32_t ilon = (int32_t)(lon * 1e7);
    payload[4] = ilat & 0xFF;
    payload[5] = (ilat >> 8) & 0xFF;
    payload[6] = (ilat >> 16) & 0xFF;
    payload[7] = (ilat >> 24) & 0xFF;
    payload[8] = ilon & 0xFF;
    payload[9] = (ilon >> 8) & 0xFF;
    payload[10] = (ilon >> 16) & 0xFF;
    payload[11] = (ilon >> 24) & 0xFF;
    uGnssMsgSend(s_gnss, (const char *)payload, sizeof(payload));
    ESP_LOGI(TAG, "MGA-INI POS_LLH seeded (%.5f, %.5f)", lat, lon);
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
