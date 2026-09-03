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
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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
static m10q_nav_status_t s_nav_status;
static uint32_t s_rx_bytes;      /* debug */
static uint32_t s_nmea_lines;    /* debug */
static uint32_t s_gsv_count;     /* debug */
static uint16_t s_agc;           /* always 0 (MIA-M10Q does not answer MON-RF) */
static volatile bool s_dump_raw;

static bool s_had_fix;
static uint32_t s_ttf_start_ms;
static bool s_ttf_running;
static uint32_t s_ttf_sum_ms;

/* Mutex protecting callback-owned data shared with UI/tracking/BLE tasks. */
static SemaphoreHandle_t s_data_mux;

static void m10q_data_lock(void)
{
    if (s_data_mux) {
        xSemaphoreTake(s_data_mux, portMAX_DELAY);
    }
}

static void m10q_data_unlock(void)
{
    if (s_data_mux) {
        xSemaphoreGive(s_data_mux);
    }
}

/* ---- RTC sync + PPS drift calibration ---- */
#define RTC_SYNC_MIN_DELTA_S  1         /* only set the RTC if |delta| > this */
#define PPS_WINDOW_S          120       /* measure drift over this many PPS pulses */
#define PPS_OFFSET_STEP_PPM   4.340     /* PCF85063A normal-mode LSB */
#define PPS_MIN_EDGES         10        /* abort calibration below this */
static bool s_cal_active;
static int16_t s_rtc_offset = -1;     /* last calibrated OFFSET value, or -1 if unset */
static TaskHandle_t s_cal_task;
static volatile int64_t s_pps_time_us;   /* esp_timer at last PPS rising edge */
static volatile uint32_t s_pps_count;    /* PPS edge counter */
static time_t s_pending_utc;             /* UTC epoch from last fix; 0 = none */

static void rtc_sync_from_utc(time_t utc);
static void cal_run(void);
static void cal_task(void *arg);
static void offset_save(void);
static void offset_load(void);
static void mga_ini_seed(void);

/* ---- RTC sync + PPS drift calibration ----
 * GNSS time is UTC (from UBX-NAV-PVT timeUtc); the RTC stores UTC too, so
 * comparing them is a direct epoch diff. rtc_sync_from_utc() rewrites the
 * RTC from GPS when they disagree by >= 1 s. cal_run() then measures the
 * RTC's rate against the MIA-M10Q 1PPS
 * on GPIO 13 and maps the drift into the PCF85063A OFFSET register.
 *
 * Drift measurement: the PPS edge marks a GPS second boundary; the RTC
 * seconds register rolls over at its own (slightly off) second boundary.
 * Between PPS edges we time the RTC rollover, so each PPS cycle yields one
 * rollover instant. Over the window, the mean RTC interval vs the mean PPS
 * interval (both on the esp_timer clock, so its own ~ppm error cancels)
 * gives the RTC rate error.
 */

#define RTC_CAL_START_BIT   (1UL << 0)

/* PPS ISR (POSEDGE on GPIO 13): timestamp the GPS second boundary. */
static void IRAM_ATTR pps_isr(void *arg)
{
    (void)arg;
    s_pps_time_us = esp_timer_get_time();
    s_pps_count++;
}

static int64_t rollover_detect(uint8_t *prev);

static void cal_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t bits = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!(bits & RTC_CAL_START_BIT)) {
            continue;
        }
        s_cal_active = true;
        if (s_rtc) {
            if (s_pending_utc) {
                rtc_sync_from_utc(s_pending_utc);
            }
            cal_run();
        }
        s_cal_active = false;
    }
}

/* ubxlib init-once guard. */
static bool s_ubxlib_init;

/* Set the RTC from GPS UTC time. The RTC stores UTC directly, so this is a
 * plain epoch diff/write - no TZ conversion. Updates s_fix.rtc_offset_s. */
static void rtc_sync_from_utc(time_t utc)
{
    pcf85063a_time_t rt;
    if (pcf85063a_get_time(s_rtc, &rt) != ESP_OK) {
        return;
    }
    time_t rtc_epoch = pcf85063a_time_to_epoch(&rt);
    int32_t delta = (int32_t)(utc - rtc_epoch);
    s_fix.rtc_offset_s = delta;

    if (delta < -RTC_SYNC_MIN_DELTA_S || delta > RTC_SYNC_MIN_DELTA_S) {
        pcf85063a_time_t nt;
        pcf85063a_epoch_to_time(utc, &nt);
        if (pcf85063a_set_time(s_rtc, &nt) == ESP_OK) {
            s_fix.rtc_offset_s = 0;    /* just applied; now in sync */
            ESP_LOGI(TAG, "RTC set from GPS: %04d-%02d-%02d %02d:%02d:%02d (was %+ld s)",
                     (int)nt.year, nt.month, nt.day, nt.hour, nt.min, nt.sec,
                     (long)delta);
        }
    } else {
        ESP_LOGI(TAG, "RTC already matched GPS (delta %+ld s)", (long)delta);
    }
}

/* Poll the RTC seconds register until it changes; return the esp_timer
 * instant (us) of that rollover, updating *prev. Returns 0 on timeout. */
static int64_t rollover_detect(uint8_t *prev)
{
    int64_t t0 = esp_timer_get_time();
    for (;;) {
        uint8_t sec;
        if (pcf85063a_get_second(s_rtc, &sec) == ESP_OK) {
            uint8_t b = sec & 0x7f;
            if (*prev != 0xFF && b != *prev) {
                *prev = b;
                return esp_timer_get_time();
            }
            *prev = b;
        } else {
            break;
        }
        if (esp_timer_get_time() - t0 > 1500000LL) {
            break;    /* no rollover within 1.5 s: RTC stopped or PPS early */
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return 0;
}

/* Run the PPS-rate measurement and apply the PCF85063A offset. */
static void cal_run(void)
{
    ESP_LOGI(TAG, "RTC PPS calibration: %d s window", PPS_WINDOW_S);

    /* The ISR service is installed by power_mgmt_init; be safe if not yet. */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "PPS cal: ISR service unavailable (%s)", esp_err_to_name(isr_err));
        return;
    }

    gpio_config_t pps_cfg = {
        .pin_bit_mask = (1ULL << M10Q_PIN_PPS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&pps_cfg);
    gpio_isr_handler_add(M10Q_PIN_PPS, pps_isr, NULL);

    int64_t p_first = 0, r_first = 0, p_last = 0, r_last = 0;
    int n = 0;
    uint8_t prev_sec = 0xFF;
    uint32_t last_cnt = s_pps_count;
    while (n < PPS_WINDOW_S && s_cal_active) {
        /* wait for a PPS edge, up to 3 s */
        uint32_t waited = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        while (s_pps_count == last_cnt) {
            if (!s_cal_active ||
                (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - waited > 3000) {
                break;    /* PPS stopped (fix lost / GNSS off) */
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_pps_count == last_cnt || !s_cal_active) {
            break;
        }
        last_cnt = s_pps_count;
        int64_t pps = s_pps_time_us;
        int64_t roll = rollover_detect(&prev_sec);
        if (roll == 0) {
            break;
        }
        if (n == 0) {
            p_first = pps;
            r_first = roll;
        }
        p_last = pps;
        r_last = roll;
        n++;
    }
    gpio_isr_handler_remove(M10Q_PIN_PPS);
    gpio_set_intr_type(M10Q_PIN_PPS, GPIO_INTR_DISABLE);

    if (n < PPS_MIN_EDGES) {
        ESP_LOGW(TAG, "PPS cal aborted: only %d edges (need %d)", n, PPS_MIN_EDGES);
        return;
    }
    /* mean RTC second interval vs mean PPS (GPS) interval, same clock.
     * drift_ppm > 0: RTC seconds are longer -> RTC runs SLOW (loses time).
     * Register value > 0 slows the clock (Linux offset convention), so we
     * negate: a slow RTC gets a negative offset (sped up), a fast RTC
     * (drift_ppm < 0) gets a positive offset (slowed down). */
    double mean_pps = (double)(p_last - p_first) / (n - 1);
    double mean_rtc = (double)(r_last - r_first) / (n - 1);
    double drift_ppm = (mean_rtc / mean_pps - 1.0) * 1e6;
    int16_t off = (int16_t)lround(-drift_ppm / PPS_OFFSET_STEP_PPM);
    if (off > 63) {
        off = 63;
    }
    if (off < -63) {
        off = -63;
    }
    if (pcf85063a_set_offset(s_rtc, (uint8_t)(off & 0x7f)) != ESP_OK) {
        ESP_LOGE(TAG, "PPS cal: failed to write OFFSET");
        return;
    }
    s_rtc_offset = off;
    offset_save();
    ESP_LOGI(TAG, "PPS cal done: drift %+.1f ppm, OFFSET %+d (%d edges)",
             drift_ppm, (int)off, n);
}

/* ---- NVS stats persistence ---- */
#define M10Q_NVS_NS       "m10q"
#define NVS_KEY_DAY       "day"
#define NVS_KEY_LAT       "gps_lat"
#define NVS_KEY_LON       "gps_lon"
#define NVS_KEY_POWERONS  "powerons"
#define NVS_KEY_RTCOFF    "rtcoff"

static uint32_t s_power_on_count;   /* cumulative GNSS power-ons (persisted) */

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

static void stats_save(const m10q_stats_t *stats, uint32_t ttf_sum_ms)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "fixes", stats->total_fixes);
        nvs_set_u32(h, "today", stats->fixes_today);
        nvs_set_u32(h, "ttfa", stats->ttf_avg_ms);
        nvs_set_u32(h, "ttfb", stats->ttf_best_ms);
        nvs_set_u32(h, "ttfs", ttf_sum_ms);
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
        nvs_get_u32(h, "ttfs", &s_ttf_sum_ms);
        nvs_get_u32(h, NVS_KEY_POWERONS, &s_power_on_count);
        nvs_close(h);
    }
}

static void power_on_count_save(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_POWERONS, s_power_on_count);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void offset_save(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i16(h, NVS_KEY_RTCOFF, s_rtc_offset);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void offset_load(void)
{
    nvs_handle_t h;
    if (nvs_open(M10Q_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int16_t v = -1;
        nvs_get_i16(h, NVS_KEY_RTCOFF, &v);
        if (v >= -63 && v <= 63) {
            s_rtc_offset = v;
        }
        nvs_close(h);
    }
    if (s_rtc_offset >= -63 && s_rtc_offset <= 63) {
        /* Re-apply the last good offset on boot. */
        pcf85063a_set_offset(s_rtc, (uint8_t)(s_rtc_offset & 0x7f));
        ESP_LOGI(TAG, "RTC offset restored: %+d", (int)s_rtc_offset);
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
    m10q_data_lock();
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
    m10q_data_unlock();
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

/* UBX-NAV-STATUS (class 0x01, id 0x03) callback. Payload:
 *   iTOW(4) gpsFix(1) flags(1) fixStat(1) flags2(1) ttff(4) msss(4)
 * flags bit0 gpsFixOk, bit2 wknsSet (GPS week valid), bit3 towSet
 * (GPS time-of-week valid). towSet/wknsSet tell us whether the receiver
 * has a valid time base at all (the no-fix root cause when they are 0). */
static void nav_status_cb(uDeviceHandle_t handle, const uGnssMessageId_t *pId,
                          int32_t errorCodeOrLength, void *pParam)
{
    (void)pParam;
    if (errorCodeOrLength <= 0 || pId == NULL ||
        pId->type != U_GNSS_PROTOCOL_UBX || pId->id.ubx != 0x0103) {
        return;
    }
    int32_t size = errorCodeOrLength;
    char *buf = (char *)heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (buf) {
        if (uGnssMsgReceiveCallbackRead(handle, buf, (size_t)size) == size) {
            const uint8_t *p = (const uint8_t *)buf + 6;   /* past 6-byte header */
            m10q_data_lock();
            s_nav_status.gps_fix    = p[4];
            s_nav_status.gps_fix_ok = (p[5] & 0x01) != 0;
            s_nav_status.wkns_set   = (p[5] & 0x04) != 0;
            s_nav_status.tow_set    = (p[5] & 0x08) != 0;
            s_nav_status.ttff_ms    = (uint32_t)p[8] | ((uint32_t)p[9] << 8) |
                                      ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
            s_nav_status.updated    = true;
            m10q_data_unlock();
        }
        heap_caps_free(buf);
    }
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
        m10q_data_lock();
        s_fix.valid = false;
        m10q_data_unlock();
        return;
    }

    m10q_data_lock();
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
        time_t e = (time_t)timeUtc;
        struct tm tm;
        gmtime_r(&e, &tm);
        s_fix.hour = (uint8_t)tm.tm_hour;
        s_fix.minute = (uint8_t)tm.tm_min;
        s_fix.second = (uint8_t)tm.tm_sec;
    }
    m10q_data_unlock();

    if (!s_had_fix) {
        s_had_fix = true;
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint32_t ttf_ms = (s_ttf_running && now_ms >= s_ttf_start_ms)
                          ? (now_ms - s_ttf_start_ms) : 0;
        s_ttf_running = false;
        m10q_data_lock();
        s_stats.total_fixes++;
        if (ttf_ms > 0) {
            s_ttf_sum_ms += ttf_ms;
            s_stats.ttf_avg_ms = s_ttf_sum_ms / s_stats.total_fixes;
            if (s_stats.ttf_best_ms == 0 || ttf_ms < s_stats.ttf_best_ms) {
                s_stats.ttf_best_ms = ttf_ms;
            }
        }
        m10q_stats_t stats = s_stats;
        uint32_t ttf_sum_ms = s_ttf_sum_ms;
        m10q_data_unlock();
        stats_save(&stats, ttf_sum_ms);
        ESP_LOGI(TAG, "FIX (power-on #%lu): %d sats, TTFF %u ms (avg %lu, best %lu)",
                 (unsigned long)s_power_on_count,
                 (int)s_fix.sat_count, ttf_ms,
                 (unsigned long)stats.ttf_avg_ms,
                 (unsigned long)stats.ttf_best_ms);
        int32_t olat = 0, olon = 0;
        lkp_load(&olat, &olon);
        if (m10q_distance_m(olat / 1e7, olon / 1e7, s_fix.lat, s_fix.lon) >= 50) {
            lkp_persist(s_fix.lat, s_fix.lon);
        }
        /* First fix with a time tag: sync the RTC and start PPS calibration. */
        if (timeUtc > 0 && s_cal_task) {
            s_pending_utc = (time_t)timeUtc;
            xTaskNotify(s_cal_task, RTC_CAL_START_BIT, eSetBits);
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
    s_pending_utc = 0;
    memset(&s_fix, 0, sizeof(s_fix));
    if (!s_data_mux) {
        s_data_mux = xSemaphoreCreateMutex();
    }
    stats_load();
    offset_load();
    if (xTaskCreate(cal_task, "rtccal", 4096, NULL, 3, &s_cal_task) != pdPASS) {
        s_cal_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t m10q_power(bool on)
{
    if (on == s_powered) {
        return ESP_OK;
    }
    if (on) {
        uint32_t pwr_no = s_power_on_count + 1;
        ESP_LOGI(TAG, "POWER-ON #%lu (ttf_avg %lu ms)", (unsigned long)pwr_no,
                 (unsigned long)s_stats.ttf_avg_ms);
        if (!s_ubxlib_init) {
            /* Init the ubxlib port layer + device/GNSS APIs once. */
            if (uPortInit() != 0 || uDeviceInit() != 0 || uGnssInit() != 0) {
                ESP_LOGE(TAG, "ubxlib init failed (power-on #%lu)", (unsigned long)pwr_no);
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
                 * (RAM/BBRAM config persists via VRTC). A failed open can leave
                 * the UART driver holding a power-management lock; delete it so
                 * the retry starts clean and light-sleep stays possible. */
                ESP_LOGW(TAG, "uDeviceOpen failed (%d) on power-on #%lu; power-cycling and retrying",
                         (int)err, (unsigned long)pwr_no);
                uart_driver_delete(M10Q_UART_NUM);
                axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
                vTaskDelay(pdMS_TO_TICKS(500));
                axp2101_enable_rail(s_pmu, AXP2101_BLDO1, true);
                vTaskDelay(pdMS_TO_TICKS(1500));
                err = uDeviceOpen(&cfg, &s_gnss);
                if (err != 0) {
                    ESP_LOGE(TAG, "uDeviceOpen failed again (power-on #%lu): %d",
                             (unsigned long)pwr_no, (int)err);
                    s_powered = false;
                    uart_driver_delete(M10Q_UART_NUM);
                    axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
                    return ESP_ERR_INVALID_STATE;
                }
            }
            s_opened = true;
        }

        int32_t pwr_err = uGnssPwrOn(s_gnss);
        if (pwr_err != 0) {
            ESP_LOGE(TAG, "uGnssPwrOn failed (power-on #%lu): %d",
                     (unsigned long)pwr_no, (int)pwr_err);
            m10q_power(false);
            return ESP_FAIL;
        }

        /* Skyplot/status telemetry is optional: log its failures but keep the
         * receiver usable for position acquisition. */
        int32_t nav_sat_cfg = U_GNSS_CFG_SET_VAL_RAM(s_gnss, MSGOUT_UBX_NAV_SAT_UART1_U1, 1);
        if (nav_sat_cfg != 0) {
            ESP_LOGW(TAG, "UBX-NAV-SAT config failed: %d", (int)nav_sat_cfg);
        }

        /* Start UBX-NAV-SAT reception for the skyplot. */
        uGnssMessageId_t navSat = { .type = U_GNSS_PROTOCOL_UBX, .id.ubx = 0x0135 };
        int32_t nav_sat_rx = uGnssMsgReceiveStart(s_gnss, &navSat, nav_sat_cb, NULL);
        if (nav_sat_rx != 0) {
            ESP_LOGW(TAG, "UBX-NAV-SAT receive start failed: %d", (int)nav_sat_rx);
        }

        /* UBX-NAV-STATUS (0x01 0x03): fix type + gpsFixOk/wknsSet/towSet so we
         * can see whether the receiver has a valid time base while acquiring. */
        int32_t nav_stat_cfg = U_GNSS_CFG_SET_VAL_RAM(s_gnss, MSGOUT_UBX_NAV_STATUS_UART1_U1, 1);
        if (nav_stat_cfg != 0) {
            ESP_LOGW(TAG, "UBX-NAV-STATUS config failed: %d", (int)nav_stat_cfg);
        }
        uGnssMessageId_t navStat = { .type = U_GNSS_PROTOCOL_UBX, .id.ubx = 0x0103 };
        int32_t nav_stat_rx = uGnssMsgReceiveStart(s_gnss, &navStat, nav_status_cb, NULL);
        if (nav_stat_rx != 0) {
            ESP_LOGW(TAG, "UBX-NAV-STATUS receive start failed: %d", (int)nav_stat_rx);
        }

        /* Warm-start aiding (time + position) so the receiver can fix fast even
         * if its own VRTC-backed time went stale. */
        mga_ini_seed();

        /* Identify the module once per power-on (UBX-MON-VER): a genuine
         * MIA-M10Q reports its MOD= string; clones usually differ/omit it. */
        uGnssVersionType_t ver;
        memset(&ver, 0, sizeof(ver));
        if (uGnssInfoGetVersions(s_gnss, &ver) == 0) {
            ESP_LOGI(TAG, "MON-VER: sw=%s hw=%s mod=%s fw=%s prot=%s",
                     ver.ver, ver.hw, ver.mod, ver.fw, ver.prot);
        }

        /* Streamed position: callback fires on every PVT. */
        int32_t sret = uGnssPosGetStreamedStart(s_gnss, 1000, pos_cb);
        if (sret != 0) {
            ESP_LOGE(TAG, "uGnssPosGetStreamedStart failed (power-on #%lu): %d",
                     (unsigned long)pwr_no, (int)sret);
            m10q_power(false);
            return ESP_FAIL;
        }

        /* Reset TTFF state. */
        s_had_fix = false;
        s_ttf_running = true;
        s_ttf_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        s_fix.valid = false;
        s_power_on_count = pwr_no;
        power_on_count_save();
        ESP_LOGI(TAG, "POWER-ON #%lu complete (BLDO1 at %u baud)", (unsigned long)pwr_no,
                 (unsigned)M10Q_BAUD);
    } else {
        if (s_opened) {
            uGnssPosGetStreamedStop(s_gnss);
            uGnssMsgReceiveStopAll(s_gnss);
            uGnssPwrOff(s_gnss);
            uDeviceClose(s_gnss, false);
            s_opened = false;
        }
        /* Ensure the UART driver is fully removed so it does not hold a power
         * lock that would block light sleep. */
        uart_driver_delete(M10Q_UART_NUM);
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
        s_pending_utc = 0;
        /* Abort an in-flight PPS calibration (GNSS rail is now off). */
        if (s_cal_active) {
            s_cal_active = false;
            gpio_set_intr_type(M10Q_PIN_PPS, GPIO_INTR_DISABLE);
            gpio_isr_handler_remove(M10Q_PIN_PPS);
        }
        ESP_LOGI(TAG, "POWER-OFF #%lu (backup RAM kept)", (unsigned long)s_power_on_count);
    }
    return ESP_OK;
}

esp_err_t m10q_get_fix(m10q_fix_t *fix)
{
    if (!fix) {
        return ESP_ERR_INVALID_ARG;
    }
    m10q_data_lock();
    *fix = s_fix;
    m10q_data_unlock();
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
    m10q_data_lock();
    m10q_state_t st = s_fix.valid ? M10Q_STATE_FIXED : M10Q_STATE_ACQUIRING;
    m10q_data_unlock();
    return st;
}

esp_err_t m10q_get_stats(m10q_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    m10q_data_lock();
    *stats = s_stats;
    m10q_data_unlock();
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
    uGnssMgaPos_t pos = {
        .latitudeX1e7  = (int32_t)(lat * 1e7),
        .longitudeX1e7 = (int32_t)(lon * 1e7),
        .altitudeMillimetres = 0,
        .radiusMillimetres   = 1000000,   /* ~1 km radius so it is never rejected */
    };
    int32_t err = uGnssMgaIniPosSend(s_gnss, &pos);
    ESP_LOGI(TAG, "MGA-INI POS_LLH %s (%.5f, %.5f)",
             err == 0 ? "acked" : "NACK/FAILED", lat, lon);
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

esp_err_t m10q_get_nav_status(m10q_nav_status_t *nav)
{
    if (!nav) {
        return ESP_ERR_INVALID_ARG;
    }
    m10q_data_lock();
    *nav = s_nav_status;
    m10q_data_unlock();
    return ESP_OK;
}

esp_err_t m10q_get_versions(uGnssVersionType_t *ver)
{
    if (!ver || !s_opened) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(ver, 0, sizeof(*ver));
    return uGnssInfoGetVersions(s_gnss, ver);
}

/* Plausibility check on the local RTC (PCF85063A) before we ever inject its
 * time into the GNSS module: guards against a garbage/stale wall clock (e.g.
 * the year-2070 bug) being sent as aiding data. */
static bool rtc_fields_plausible(const pcf85063a_time_t *rt)
{
    if (rt->year < 2000 || rt->year > 2100 ||
        rt->month < 1 || rt->month > 12 ||
        rt->day < 1 || rt->day > 31 ||
        rt->hour > 23 || rt->min > 59 || rt->sec > 60 ||
        rt->weekday < 1 || rt->weekday > 7) {
        return false;
    }
    return true;
}

/* Inject MGA-INI aiding from the RTC (UTC calendar fields -> UTC epoch) and
 * the NVS last-known position, but only when the module actually needs it:
 *
 *  - Time: the module keeps its own RTC on the VBACKUP rail. If that is alive
 *    (uGnssInfoGetTimeUtcRaw() returns a plausible epoch), injection is
 *    unnecessary and is skipped. Only a missing/stale module time is seeded.
 *  - Position: sent only if the NVS last-known position is in range.
 *
 * Uses the ubxlib helpers that wait for and check the module's MGA-ACK, so a
 * FAILED result tells us the module rejected the data. */
static void mga_ini_seed(void)
{
    if (!s_opened || s_rtc == NULL) {
        return;
    }

    pcf85063a_time_t rt;
    time_t epoch = 0;
    if (pcf85063a_get_time(s_rtc, &rt) == ESP_OK && rtc_fields_plausible(&rt)) {
        epoch = pcf85063a_time_to_epoch(&rt);
    }

    /* Check the module's own VBACKUP-backed time first. */
    int64_t mod_epoch = uGnssInfoGetTimeUtcRaw(s_gnss);
    if (epoch > 0 && mod_epoch > 0 &&
        (mod_epoch - (int64_t)epoch) > -3600 && (mod_epoch - (int64_t)epoch) < 3600) {
        ESP_LOGI(TAG, "MGA-INI TIME skipped: module time valid (diff %+lld s)",
                 (long long)(mod_epoch - (int64_t)epoch));
    } else if (epoch > 0) {
        int32_t err = uGnssMgaIniTimeSend(s_gnss, (int64_t)epoch * 1000000000LL,
                                          1000000000LL, NULL);   /* ~1 s */
        ESP_LOGI(TAG, "MGA-INI TIME %s (epoch %lld, module had %lld)",
                 err == 0 ? "acked" : "NACK/FAILED",
                 (long long)epoch, (long long)mod_epoch);
    } else {
        ESP_LOGW(TAG, "MGA-INI TIME skipped: local RTC not plausible");
    }

    int32_t ilat = 0, ilon = 0;
    lkp_load(&ilat, &ilon);
    if (ilat >= -90 * 10000000 && ilat <= 90 * 10000000 &&
        ilon >= -180 * 10000000 && ilon <= 180 * 10000000 &&
        (ilat != 0 || ilon != 0)) {
        uGnssMgaPos_t pos = {
            .latitudeX1e7  = ilat,
            .longitudeX1e7 = ilon,
            .altitudeMillimetres = 0,
            .radiusMillimetres   = 1000000,   /* ~1 km */
        };
        int32_t err = uGnssMgaIniPosSend(s_gnss, &pos);
        ESP_LOGI(TAG, "MGA-INI POS %s (%.5f, %.5f)",
                 err == 0 ? "acked" : "NACK/FAILED", ilat / 1e7, ilon / 1e7);
    }
}

void m10q_set_raw_dump(bool on)
{
    s_dump_raw = on;
}

void m10q_rtc_calibrate(void)
{
    if (!s_cal_task || s_cal_active) {
        return;
    }
    m10q_data_lock();
    bool ok = s_fix.valid && s_fix.sat_count >= 3;
    m10q_data_unlock();
    if (ok) {
        xTaskNotify(s_cal_task, RTC_CAL_START_BIT, eSetBits);
    } else {
        ESP_LOGW(TAG, "PPS cal: no valid fix, skipping");
    }
}
