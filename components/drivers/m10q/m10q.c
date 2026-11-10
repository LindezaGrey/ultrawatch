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
#include <stdint.h>
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
#include "freertos/queue.h"

#include "ubxlib.h"
#include "u_gnss_module_type.h"
#include "u_gnss_cfg_val_key.h"
#include "m10q.h"
#include "m10q_decode.h"
#include "axp2101.h"
#include "pcf85063a.h"

#define TAG "m10q"

#define M10Q_UART_NUM    1       /* ESP32 UART1 for the GNSS receiver */
#define M10Q_BAUD        38400   /* factory UART speed used by this driver */

/* ---- static state ---- */
static i2c_master_dev_handle_t s_pmu;
static i2c_master_dev_handle_t s_rtc;
static bool s_powered;            /* receiver fully configured and running */
static bool s_rail_on;            /* last observed BLDO1 state */
static bool s_rail_state_known;
static bool s_opened;             /* uDeviceOpen done */
static uGnssVersionType_t s_versions;
static bool s_versions_valid;
static bool s_session_time_synced;
static uDeviceHandle_t s_gnss;   /* ubxlib GNSS device handle */

static m10q_fix_t s_fix;
static m10q_stats_t s_stats;
static m10q_nav_status_t s_nav_status;
typedef struct {
    uint32_t generation;
    bool valid;
    double lat, lon;
    uint16_t sats;
    time_t utc;
} m10q_fix_event_t;

static uint32_t s_power_generation;
static TaskHandle_t s_fix_task;
static QueueHandle_t s_fix_queue;
static SemaphoreHandle_t s_fix_work_mux;
static uint32_t s_gsv_count;     /* NAV-SAT frames received */

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
static int16_t s_rtc_offset = INT16_MIN;  /* last calibrated OFFSET value, or INT16_MIN if unset */
static TaskHandle_t s_cal_task;
static volatile int64_t s_pps_time_us;   /* esp_timer at last PPS rising edge */
static volatile uint32_t s_pps_count;    /* PPS edge counter */
static volatile uint32_t s_pps_seq;      /* seqlock guarding s_pps_time_us */
static time_t s_pending_utc;             /* UTC epoch from last fix; 0 = none */

static void rtc_sync_from_utc(time_t utc);
static void cal_run(void);
static void cal_task(void *arg);
static void fix_task(void *arg);
static void position_updated(bool valid, double lat, double lon, uint16_t sats, time_t utc);
static void offset_save(void);
static void offset_load(void);
static void mga_ini_seed(void);

/* Verify BLDO1 changes before recording the observed hardware state. */
static esp_err_t m10q_set_rail(bool enable)
{
    if (s_pmu == NULL) return ESP_ERR_INVALID_STATE;
    esp_err_t set_err = axp2101_enable_rail(s_pmu, AXP2101_BLDO1, enable);
    if (set_err == ESP_OK) {
        s_rail_on = enable;
        s_rail_state_known = true;
        return ESP_OK;
    }
    bool enabled = false;
    esp_err_t read_err = axp2101_is_rail_enabled(s_pmu, AXP2101_BLDO1, &enabled);
    if (read_err != ESP_OK) {
        s_rail_state_known = false;
        return read_err;
    }
    s_rail_on = enabled;
    s_rail_state_known = true;
    if (enabled != enable) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

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
    s_pps_seq++;                    /* odd: write in progress */
    s_pps_time_us = esp_timer_get_time();
    s_pps_seq++;                    /* even: timestamp committed */
    s_pps_count++;
}

/* A 64-bit volatile read is not atomic on Xtensa: an edge landing between the
 * two halves yields a timestamp from neither. s_pps_seq is the seqlock the ISR
 * brackets the write with, so a changed (or odd) value means retry. Handing
 * back the edge count with it keeps the caller's edge bookkeeping and the
 * timestamp describing the same edge. */
static void pps_snapshot(uint32_t *edge, int64_t *us)
{
    for (;;) {
        uint32_t seq = s_pps_seq;
        if (seq & 1u) {
            continue;    /* ISR is mid-write */
        }
        int64_t t = s_pps_time_us;
        uint32_t cnt = s_pps_count;
        if (s_pps_seq == seq) {
            *edge = cnt;
            *us = t;
            return;
        }
    }
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

static void fix_task(void *arg)
{
    (void)arg;
    m10q_fix_event_t event;
    for (;;) {
        if (xQueueReceive(s_fix_queue, &event, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(s_fix_work_mux, portMAX_DELAY);
            m10q_data_lock();
            bool current = event.generation == s_power_generation && s_powered;
            m10q_data_unlock();
            if (current) {
                position_updated(event.valid, event.lat, event.lon, event.sats, event.utc);
            }
            xSemaphoreGive(s_fix_work_mux);
        }
    }
}

/* ubxlib init-once guard. */
static bool s_ubxlib_init;

/* Set the RTC from GPS UTC time. The RTC stores UTC directly, so this is a
 * plain epoch diff/write - no TZ conversion. Updates s_fix.rtc_offset_s. */
static void rtc_sync_from_utc(time_t utc)
{
    pcf85063a_time_t rt;
    bool rtc_readable = (pcf85063a_get_time(s_rtc, &rt) == ESP_OK);
    if (!rtc_readable) {
        /* Usually the sticky OS bit in Register_00 (oscillator stopped, e.g.
         * after a supply glitch). Its time is meaningless, and only
         * pcf85063a_set_time() clears OS - so write unconditionally instead of
         * bailing, otherwise the RTC could never recover from GPS. */
        ESP_LOGW(TAG, "RTC time unreadable (oscillator stopped) - forcing sync");
    }
    time_t rtc_epoch = rtc_readable ? pcf85063a_time_to_epoch(&rt) : 0;
    int32_t delta = (int32_t)(utc - rtc_epoch);
    if (rtc_readable) {
        m10q_data_lock();
        s_fix.rtc_offset_s = delta;
        m10q_data_unlock();
    }

    if (!rtc_readable || delta < -RTC_SYNC_MIN_DELTA_S || delta > RTC_SYNC_MIN_DELTA_S) {
        pcf85063a_time_t nt;
        pcf85063a_epoch_to_time(utc, &nt);
        if (pcf85063a_set_time(s_rtc, &nt) == ESP_OK) {
            m10q_data_lock();
            s_fix.rtc_offset_s = 0;
            m10q_data_unlock();
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
        int64_t pps;
        uint32_t edge;
        pps_snapshot(&edge, &pps);
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
        /* Count edges, not loop passes: a stalled poll can miss one, and the
         * first/last span below then covers that many more seconds. */
        n += (int)(edge - last_cnt);
        last_cnt = edge;
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
        int16_t v = INT16_MIN;
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

/* NAV-SAT message callback (ubxlib task context). */
static void nav_sat_cb(uDeviceHandle_t handle, const uGnssMessageId_t *pId,
                       int32_t errorCodeOrLength, void *pParam)
{
    (void)pParam;
    if (errorCodeOrLength < 16 || pId == NULL ||
        pId->type != U_GNSS_PROTOCOL_UBX || pId->id.ubx != 0x0135) {
        return;
    }
    int32_t size = errorCodeOrLength;
    char *buf = (char *)heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (buf) {
        if (uGnssMsgReceiveCallbackRead(handle, buf, (size_t)size) == size) {
            /* payload starts after 6-byte header (sync2+class+id+len2) */
            m10q_data_lock();
            m10q_decode_sat(&s_fix, (const uint8_t *)buf + 6, (size_t)size - 8,
                            (uint32_t)(esp_timer_get_time() / 1000));
            m10q_data_unlock();
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
    if (errorCodeOrLength != 24 || pId == NULL ||
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
            s_nav_status.updated_ms = (uint32_t)(esp_timer_get_time() / 1000);
            s_nav_status.updated    = true;
            m10q_data_unlock();
        }
        heap_caps_free(buf);
    }
}

/* Preserve statistics and RTC synchronization after a decoded PVT update. */
static void position_updated(bool valid, double lat, double lon, uint16_t sats, time_t utc)
{
    if (!valid) {
        if (s_had_fix) {
            s_had_fix = false;
            s_ttf_running = true;
            s_ttf_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        }
        return;
    }
    if (!s_had_fix) {
        s_had_fix = true;
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t ttf_ms = s_ttf_running
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
                 (int)sats, ttf_ms,
                 (unsigned long)stats.ttf_avg_ms,
                 (unsigned long)stats.ttf_best_ms);
        int32_t olat = 0, olon = 0;
        lkp_load(&olat, &olon);
        if (m10q_distance_m(olat / 1e7, olon / 1e7, lat, lon) >= 50) {
            lkp_persist(lat, lon);
        }
    }
    if (utc > 0 && !s_session_time_synced && s_cal_task) {
        s_session_time_synced = true;
        s_pending_utc = utc;
        xTaskNotify(s_cal_task, RTC_CAL_START_BIT, eSetBits);
    }
}

/* UTC epoch from a decoded NAV-PVT time, 0 if the time is not valid. */
static time_t fix_utc_epoch(const m10q_fix_t *f)
{
    if (!f->time_valid) {
        return 0;
    }
    pcf85063a_time_t date = { .year = f->year, .month = f->month, .day = f->day,
        .hour = f->hour, .min = f->minute, .sec = f->second };
    return pcf85063a_time_to_epoch(&date);
}

/* Read the complete NAV-PVT epoch, including real fix type and heading. */
static void nav_pvt_cb(uDeviceHandle_t handle, const uGnssMessageId_t *id,
                       int32_t length, void *arg)
{
    (void)arg;
    if (length != 100 || !id || id->type != U_GNSS_PROTOCOL_UBX || id->id.ubx != 0x0107) return;
    m10q_data_lock();
    uint32_t generation = s_power_generation;
    m10q_data_unlock();
    uint8_t frame[100];
    if (uGnssMsgReceiveCallbackRead(handle, (char *)frame, sizeof(frame)) != sizeof(frame)) return;
    m10q_data_lock();
    bool decoded = m10q_decode_pvt(&s_fix, frame + 6, 92, (uint32_t)(esp_timer_get_time() / 1000));
    bool valid = s_fix.valid;
    double lat = s_fix.lat, lon = s_fix.lon;
    uint16_t sats = s_fix.sat_count;
    time_t utc = fix_utc_epoch(&s_fix);
    m10q_data_unlock();
    if (decoded && s_fix_queue != NULL) {
        m10q_fix_event_t event;
        event.generation = generation;
        event.valid = valid;
        event.lat = lat;
        event.lon = lon;
        event.sats = sats;
        event.utc = utc;
        if (xQueueSend(s_fix_queue, &event, 0) != pdTRUE) {
            ESP_LOGW(TAG, "GNSS fix event queue full; dropped event");
        }
    }
}

static void nav_dop_cb(uDeviceHandle_t handle, const uGnssMessageId_t *id,
                       int32_t length, void *arg)
{
    (void)arg;
    if (length != 26 || !id || id->type != U_GNSS_PROTOCOL_UBX || id->id.ubx != 0x0104) return;
    uint8_t frame[26];
    if (uGnssMsgReceiveCallbackRead(handle, (char *)frame, sizeof(frame)) != sizeof(frame)) return;
    m10q_data_lock();
    m10q_decode_dop(&s_fix, frame + 6, 18, (uint32_t)(esp_timer_get_time() / 1000));
    m10q_data_unlock();
}

/* ---- Public API ---- */

/* Caller holds s_fix_work_mux to exclude pending first-fix persistence. */
static void m10q_clear_runtime_state(void)
{
    m10q_data_lock();
    s_powered = false;
    s_had_fix = false;
    s_power_generation++;
    memset(&s_fix, 0, sizeof(s_fix));
    s_fix.hdop = UINT16_MAX;
    memset(&s_nav_status, 0, sizeof(s_nav_status));
    m10q_data_unlock();
    s_pending_utc = 0;
    if (s_cal_active) {
        s_cal_active = false;
        gpio_set_intr_type(M10Q_PIN_PPS, GPIO_INTR_DISABLE);
        gpio_isr_handler_remove(M10Q_PIN_PPS);
    }
    if (s_fix_queue) {
        xQueueReset(s_fix_queue);
    }
}

esp_err_t m10q_init(i2c_master_dev_handle_t pmu, i2c_master_dev_handle_t rtc)
{
    s_pmu = pmu;
    s_rtc = rtc;
    s_powered = false;
    s_rail_state_known = false;
    s_opened = false;
    s_pending_utc = 0;
    memset(&s_fix, 0, sizeof(s_fix));
    if (!s_data_mux) {
        s_data_mux = xSemaphoreCreateMutex();
        if (!s_data_mux) {
            return ESP_ERR_NO_MEM;
        }
    }
    stats_load();
    offset_load();
    if (xTaskCreate(cal_task, "rtccal", 4096, NULL, 3, &s_cal_task) != pdPASS) {
        s_cal_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_fix_queue = xQueueCreate(4, sizeof(m10q_fix_event_t));
    if (!s_fix_queue) {
        vTaskDelete(s_cal_task);
        s_cal_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_fix_work_mux = xSemaphoreCreateMutex();
    if (!s_fix_work_mux) {
        vQueueDelete(s_fix_queue);
        s_fix_queue = NULL;
        vTaskDelete(s_cal_task);
        s_cal_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    m10q_data_lock();
    s_power_generation = 0;
    m10q_data_unlock();
    if (xTaskCreate(fix_task, "gnss_fix", 4096, NULL, 3, &s_fix_task) != pdPASS) {
        s_fix_task = NULL;
        vSemaphoreDelete(s_fix_work_mux);
        s_fix_work_mux = NULL;
        vQueueDelete(s_fix_queue);
        s_fix_queue = NULL;
        vTaskDelete(s_cal_task);
        s_cal_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t m10q_power(bool on)
{
    if (!s_data_mux || !s_fix_queue || !s_fix_work_mux) {
        return ESP_ERR_INVALID_STATE;
    }
    if (on && s_powered && s_opened && s_rail_state_known && s_rail_on) {
        return ESP_OK;
    }
    if (on && s_powered && (!s_opened || !s_rail_state_known || !s_rail_on)) {
        if (!s_fix_work_mux) return ESP_ERR_INVALID_STATE;
        xSemaphoreTake(s_fix_work_mux, portMAX_DELAY);
        if (s_opened) {
            int32_t close_err = uDeviceClose(s_gnss, false);
            if (close_err != 0) {
                m10q_data_lock();
                s_powered = false;
                m10q_data_unlock();
                xSemaphoreGive(s_fix_work_mux);
                return ESP_FAIL;
            }
            s_opened = false;
            s_gnss = NULL;
            uart_driver_delete(M10Q_UART_NUM);
        }
        esp_err_t rail_err = m10q_set_rail(false);
        if (rail_err != ESP_OK) {
            xSemaphoreGive(s_fix_work_mux);
            return rail_err;
        }
        m10q_clear_runtime_state();
        xSemaphoreGive(s_fix_work_mux);
    }
    if (!on && !s_powered && !s_opened && s_rail_state_known && !s_rail_on) {
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

        /* Start a distinct session so queued callback work from an older
         * power cycle cannot persist stale fix statistics. */
        xSemaphoreTake(s_fix_work_mux, portMAX_DELAY);
        m10q_data_lock();
        s_power_generation++;
        s_powered = false;
        memset(&s_fix, 0, sizeof(s_fix));
        s_fix.hdop = UINT16_MAX;
        memset(&s_nav_status, 0, sizeof(s_nav_status));
        m10q_data_unlock();
        xQueueReset(s_fix_queue);

        /* Power the receiver rail and let it boot. */
        esp_err_t rail_err = m10q_set_rail(true);
        if (rail_err != ESP_OK) {
            ESP_LOGE(TAG, "BLDO1 enable failed: %s", esp_err_to_name(rail_err));
            xSemaphoreGive(s_fix_work_mux);
            return rail_err;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (!s_opened) {
            uDeviceCfg_t cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.deviceType = U_DEVICE_TYPE_GNSS;
            cfg.deviceCfg.cfgGnss.moduleType = U_GNSS_MODULE_TYPE_M10;
            cfg.deviceCfg.cfgGnss.pinEnablePower = -1;   /* rail via AXP */
            cfg.transportType = U_DEVICE_TRANSPORT_TYPE_UART;
            cfg.transportCfg.cfgUart.uart = M10Q_UART_NUM;
            cfg.transportCfg.cfgUart.baudRate = M10Q_BAUD; /* factory baud; module reverts on power cycle */
            cfg.transportCfg.cfgUart.pinTxd = M10Q_PIN_TX;
            cfg.transportCfg.cfgUart.pinRxd = M10Q_PIN_RX;
            cfg.transportCfg.cfgUart.pinCts = -1;
            cfg.transportCfg.cfgUart.pinRts = -1;
            int32_t err = uDeviceOpen(&cfg, &s_gnss);
            if (err != 0) {
                s_gnss = NULL;
                /* A failed open can leave the UART driver holding a power-
                 * management lock; delete it so retry starts clean and
                 * light-sleep remains possible. */
                ESP_LOGW(TAG, "uDeviceOpen failed (%d) on power-on #%lu; power-cycling and retrying",
                         (int)err, (unsigned long)pwr_no);
                uart_driver_delete(M10Q_UART_NUM);
                esp_err_t off_err = m10q_set_rail(false);
                if (off_err != ESP_OK) {
                    ESP_LOGE(TAG, "BLDO1 retry power-off failed: %s", esp_err_to_name(off_err));
                    xSemaphoreGive(s_fix_work_mux);
                    return off_err;
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_err_t on_err = m10q_set_rail(true);
                if (on_err != ESP_OK) {
                    ESP_LOGE(TAG, "BLDO1 retry power-on failed: %s", esp_err_to_name(on_err));
                    xSemaphoreGive(s_fix_work_mux);
                    return on_err;
                }
                vTaskDelay(pdMS_TO_TICKS(1500));
                err = uDeviceOpen(&cfg, &s_gnss);
                if (err != 0) {
                    s_gnss = NULL;
                    ESP_LOGE(TAG, "uDeviceOpen failed again (power-on #%lu): %d",
                             (unsigned long)pwr_no, (int)err);
                    uart_driver_delete(M10Q_UART_NUM);
                    esp_err_t cleanup_err = m10q_set_rail(false);
                    if (cleanup_err != ESP_OK) {
                        ESP_LOGE(TAG, "BLDO1 cleanup after open failure failed: %s",
                                 esp_err_to_name(cleanup_err));
                        xSemaphoreGive(s_fix_work_mux);
                        return cleanup_err;
                    }
                    m10q_data_lock();
                    s_powered = false;
                    m10q_data_unlock();
                    xSemaphoreGive(s_fix_work_mux);
                    return ESP_ERR_INVALID_STATE;
                }
            }
            s_opened = true;
        }

        int32_t pwr_err = uGnssPwrOn(s_gnss);
        if (pwr_err != 0) {
            ESP_LOGE(TAG, "uGnssPwrOn failed (power-on #%lu): %d",
                     (unsigned long)pwr_no, (int)pwr_err);
            int32_t close_err = uDeviceClose(s_gnss, false);
            if (close_err == 0) {
                s_opened = false;
                s_gnss = NULL;
            }
            uart_driver_delete(M10Q_UART_NUM);
            esp_err_t rail_err = m10q_set_rail(false);
            if (close_err != 0 || rail_err != ESP_OK) {
                xSemaphoreGive(s_fix_work_mux);
                return close_err != 0 ? ESP_FAIL : rail_err;
            }
            m10q_data_lock();
            s_powered = false;
            m10q_data_unlock();
            xSemaphoreGive(s_fix_work_mux);
            return ESP_FAIL;
        }
        m10q_data_lock();
        s_powered = true;
        m10q_data_unlock();

        m10q_data_lock();
        memset(&s_fix, 0, sizeof(s_fix));
        s_fix.hdop = UINT16_MAX;
        memset(&s_nav_status, 0, sizeof(s_nav_status));
        m10q_data_unlock();
        s_had_fix = false;
        s_session_time_synced = false;
        s_ttf_running = true;
        s_ttf_start_ms = (uint32_t)(esp_timer_get_time() / 1000);

        /* Skyplot/status telemetry is optional: log its failures but keep the
         * receiver usable for position acquisition. */
        int32_t nav_sat_cfg = U_GNSS_CFG_SET_VAL_RAM(s_gnss, MSGOUT_UBX_NAV_SAT_UART1_U1, 1);
        if (nav_sat_cfg != 0) {
            ESP_LOGW(TAG, "UBX-NAV-SAT config failed: %d", (int)nav_sat_cfg);
        }

        /* Start UBX-NAV-SAT reception for the skyplot. */
        uGnssMessageId_t navSat = { .type = U_GNSS_PROTOCOL_UBX, .id.ubx = 0x0135 };
        int32_t nav_sat_rx = uGnssMsgReceiveStart(s_gnss, &navSat, nav_sat_cb, NULL);
        if (nav_sat_rx < 0) {
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
        if (nav_stat_rx < 0) {
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
            m10q_data_lock();
            s_versions = ver;
            s_versions_valid = true;
            m10q_data_unlock();
            ESP_LOGI(TAG, "MON-VER: sw=%s hw=%s mod=%s fw=%s prot=%s",
                     ver.ver, ver.hw, ver.mod, ver.fw, ver.prot);
        }

        int32_t nav_rate = uGnssCfgSetRate(s_gnss, 1000, 1, -1);
        if (nav_rate != 0) ESP_LOGW(TAG, "GNSS 1Hz configuration failed: %ld", (long)nav_rate);
        int32_t rate_err = U_GNSS_CFG_SET_VAL_RAM(s_gnss, MSGOUT_UBX_NAV_PVT_UART1_U1, 1);
        uGnssMessageId_t pvt = { .type = U_GNSS_PROTOCOL_UBX, .id.ubx = 0x0107 };
        int32_t rx = rate_err == 0 ? uGnssMsgReceiveStart(s_gnss, &pvt, nav_pvt_cb, NULL) : rate_err;
        if (rx < 0 || rate_err != 0) {
            ESP_LOGE(TAG, "NAV-PVT startup failed: %ld", (long)rx);
            m10q_data_lock();
            s_powered = false;
            m10q_data_unlock();
            int32_t stop_err = uGnssMsgReceiveStopAll(s_gnss);
            int32_t pwr_off_err = uGnssPwrOff(s_gnss);
            int32_t close_err = uDeviceClose(s_gnss, false);
            if (close_err == 0) {
                s_opened = false;
                s_gnss = NULL;
            }
            if (stop_err != 0 || pwr_off_err != 0 || close_err != 0) {
                ESP_LOGW(TAG, "NAV-PVT rollback: stop=%ld power-off=%ld close=%ld",
                         (long)stop_err, (long)pwr_off_err, (long)close_err);
            }
            uart_driver_delete(M10Q_UART_NUM);
            if (pwr_off_err != 0 || close_err != 0) {
                m10q_data_lock();
                s_powered = true;
                m10q_data_unlock();
                xSemaphoreGive(s_fix_work_mux);
                return ESP_FAIL;
            }
            esp_err_t rail_err = m10q_set_rail(false);
            if (rail_err != ESP_OK) {
                xSemaphoreGive(s_fix_work_mux);
                return rail_err;
            }
            m10q_clear_runtime_state();
            xSemaphoreGive(s_fix_work_mux);
            return ESP_FAIL;
        }
        int32_t dop_cfg = U_GNSS_CFG_SET_VAL_RAM(s_gnss, MSGOUT_UBX_NAV_DOP_UART1_U1, 1);
        uGnssMessageId_t dop = { .type = U_GNSS_PROTOCOL_UBX, .id.ubx = 0x0104 };
        int32_t dop_rx = dop_cfg == 0 ? uGnssMsgReceiveStart(s_gnss, &dop, nav_dop_cb, NULL) : dop_cfg;
        if (dop_cfg != 0 || dop_rx < 0) ESP_LOGW(TAG, "NAV-DOP unavailable: %ld", (long)dop_rx);

        s_power_on_count = pwr_no;
        power_on_count_save();
        xSemaphoreGive(s_fix_work_mux);
        ESP_LOGI(TAG, "POWER-ON #%lu complete (BLDO1 at %u baud)", (unsigned long)pwr_no,
                 (unsigned)M10Q_BAUD);
    } else {
        xSemaphoreTake(s_fix_work_mux, portMAX_DELAY);
        int32_t close_err = 0;
        if (s_opened) {
            int32_t stop_err = uGnssMsgReceiveStopAll(s_gnss);
            int32_t pwr_off_err = uGnssPwrOff(s_gnss);
            if (pwr_off_err != 0) {
                ESP_LOGW(TAG, "uGnssPwrOff failed (%ld); keeping receiver rail on", (long)pwr_off_err);
                m10q_data_lock();
                s_powered = true;
                m10q_data_unlock();
                xSemaphoreGive(s_fix_work_mux);
                return ESP_FAIL;
            }
            close_err = uDeviceClose(s_gnss, false);
            if (close_err == 0) {
                s_opened = false;
                s_gnss = NULL;
            }
            if (stop_err != 0 || pwr_off_err != 0 || close_err != 0) {
                ESP_LOGW(TAG, "ubxlib shutdown: stop=%ld power-off=%ld close=%ld",
                         (long)stop_err, (long)pwr_off_err, (long)close_err);
            }
            if (close_err != 0) {
                m10q_data_lock();
                s_powered = false;
                m10q_data_unlock();
                xSemaphoreGive(s_fix_work_mux);
                return ESP_FAIL;
            }
        }
        /* Ensure the UART driver is fully removed so it does not hold a power
         * lock that would block light sleep. */
        uart_driver_delete(M10Q_UART_NUM);
        /* Persist LKP if we have a fresh fix. */
        m10q_fix_t final_fix;
        m10q_get_fix(&final_fix);
        if (final_fix.valid) {
            /* A valid PVT event may still be queued behind this shutdown lock.
             * Record it here before the queue is cleared; the worker drops it
             * afterward because the power generation changes at shutdown. */
            if (!s_had_fix) {
                position_updated(true, final_fix.lat, final_fix.lon, final_fix.sat_count,
                                 fix_utc_epoch(&final_fix));
            }
            int32_t olat = 0, olon = 0;
            lkp_load(&olat, &olon);
            if (m10q_distance_m(olat / 1e7, olon / 1e7, final_fix.lat, final_fix.lon) >= 50) {
                lkp_persist(final_fix.lat, final_fix.lon);
            }
        }
        esp_err_t rail_err = m10q_set_rail(false);
        if (rail_err != ESP_OK) {
            ESP_LOGE(TAG, "BLDO1 disable failed: %s", esp_err_to_name(rail_err));
            m10q_data_lock();
            s_powered = false;
            m10q_data_unlock();
            xSemaphoreGive(s_fix_work_mux);
            return rail_err;
        }
        m10q_clear_runtime_state();
        xSemaphoreGive(s_fix_work_mux);
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
    m10q_expire_snapshot(fix, (uint32_t)(esp_timer_get_time() / 1000));
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
    m10q_data_lock();
    bool powered = s_powered;
    bool valid = s_fix.valid && s_fix.pvt_received &&
                 (uint32_t)(esp_timer_get_time() / 1000) - s_fix.updated_ms < M10Q_FRESH_MS;
    m10q_data_unlock();
    return !powered ? M10Q_STATE_OFF : valid ? M10Q_STATE_FIXED : M10Q_STATE_ACQUIRING;
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
    *agc = 0;
    return ESP_ERR_NOT_SUPPORTED;
}

void m10q_poll_agc(void)
{
    /* MON-RF is unsupported by this receiver. */
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
    if (rx_bytes) *rx_bytes = 0;
    if (nmea_lines) *nmea_lines = 0;
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
    if ((uint32_t)(esp_timer_get_time() / 1000) - nav->updated_ms >= M10Q_FRESH_MS) nav->updated = false;
    return ESP_OK;
}

esp_err_t m10q_get_versions(uGnssVersionType_t *ver)
{
    if (!ver) return ESP_ERR_INVALID_ARG;
    m10q_data_lock();
    bool available = s_versions_valid;
    if (available) *ver = s_versions;
    m10q_data_unlock();
    return available ? ESP_OK : ESP_ERR_NOT_FOUND;
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
        rt->weekday > 6) {
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
    (void)on;
}

void m10q_rtc_calibrate(void)
{
    if (!s_cal_task || s_cal_active) {
        return;
    }
    m10q_fix_t fix;
    m10q_get_fix(&fix);
    bool ok = fix.valid && fix.time_valid;
    if (ok) {
        /* A manual cal request means "correct the clock", so carry the GPS time
         * through: without s_pending_utc, cal_task skips rtc_sync_from_utc()
         * and only trims the rate, leaving an absolute time error in place. */
        s_pending_utc = fix_utc_epoch(&fix);
        xTaskNotify(s_cal_task, RTC_CAL_START_BIT, eSetBits);
    } else {
        ESP_LOGW(TAG, "PPS cal: no valid fix, skipping");
    }
}
