/*
 * mock_hw.h - fake hardware-driver surface for the host simulator.
 *
 * Declares the exact subset of the real driver APIs (pcf85063a.h,
 * sensor_cache.h, m10q.h, bhi260ap.h, tracking.h, alarm.h, axp2101.h,
 * power_mgmt.h, daily_log.h, lvgl_app.h) that the six ported screens call,
 * with identical names/signatures/field names so the screen .c files are
 * verbatim copies of main/lvgl_app.c's screen code, not a rewrite. Anything
 * no screen touches is omitted - this is a mock, not a port of the real
 * headers.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int esp_err_t;
#define ESP_OK 0

/* --- pcf85063a.h subset ---
 * The real RTC stores UTC directly; sensor_cache_get_rtc() below mirrors that
 * by synthesizing UTC fields, and pcf85063a_time_to_epoch() mirrors the real
 * driver's TZ-independent civil<->epoch conversion so the ported watch-face
 * code (which calls it to get local display time) is a true verbatim copy. */
typedef struct {
    uint8_t  sec;
    uint8_t  min;
    uint8_t  hour;
    uint8_t  day;
    uint8_t  weekday;
    uint8_t  month;
    uint16_t year;
} pcf85063a_time_t;

time_t pcf85063a_time_to_epoch(const pcf85063a_time_t *t);

/* --- axp2101.h subset ---
 * Real functions take an i2c_master_dev_handle_t (ESP-IDF I2C driver type,
 * unavailable on the host); mocked here as an opaque pointer so the copied
 * screen code's call sites (axp2101_set_charge_enabled(twatch_pmu_dev, ...))
 * compile unchanged. twatch_pmu_dev is a dummy non-NULL handle. */
typedef void *i2c_master_dev_handle_t;
extern i2c_master_dev_handle_t twatch_pmu_dev;

typedef enum {
    AXP2101_CHG_TRI,
    AXP2101_CHG_PRE,
    AXP2101_CHG_CC,
    AXP2101_CHG_CV,
    AXP2101_CHG_DONE,
    AXP2101_CHG_STOP,
} axp2101_charge_state_t;

esp_err_t axp2101_set_charge_enabled(i2c_master_dev_handle_t dev, bool enable);
esp_err_t axp2101_set_charge_current_ma(i2c_master_dev_handle_t dev, uint16_t ma);

/* --- sensor_cache.h subset --- */
typedef struct {
    uint8_t  batt_pct;
    uint16_t batt_mv;
    axp2101_charge_state_t chg_state;
    bool     chg_enabled;
    uint16_t chg_ma;
    int16_t  batt_temp_c10;
    bool     valid;
} sensor_cache_t;

void sensor_cache_get(sensor_cache_t *out);
bool sensor_cache_get_rtc(pcf85063a_time_t *out);

typedef struct {
    bool  estimate_valid;
    float pct_per_hour;
    float runtime_h;
    float charge_h;
} battery_estimate_t;

void battery_estimate_get(battery_estimate_t *out);

/* --- power_mgmt.h subset --- */
bool power_mgmt_get_night_mode_auto(void);
void power_mgmt_set_night_mode_auto(bool on);
bool power_mgmt_get_skip_sleep_on_usb(void);
void power_mgmt_set_skip_sleep_on_usb(bool yes);

/* --- m10q.h subset --- */
#define M10Q_MAX_SATS 24

typedef enum {
    M10Q_STATE_OFF = 0,
    M10Q_STATE_ACQUIRING,
    M10Q_STATE_FIXED,
} m10q_state_t;

typedef struct {
    uint8_t  prn;
    int16_t  elevation_deg;
    int16_t  azimuth_deg;
    int16_t  snr_db;
    bool     used;
} m10q_sat_t;

typedef struct {
    bool     valid;
    bool     fix_3d;
    double   lat;
    double   lon;
    double   alt_m;
    uint16_t sat_count;
    uint16_t sat_in_view;
    uint16_t speed_kmh;
    uint16_t course_deg;
    uint16_t hdop;
    uint16_t hacc_m;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    int32_t  rtc_offset_s;
    m10q_sat_t sats[M10Q_MAX_SATS];
} m10q_fix_t;

typedef struct {
    uint32_t total_fixes;
    uint32_t fixes_today;
    uint32_t ttf_avg_ms;
    uint32_t ttf_best_ms;
} m10q_stats_t;

m10q_state_t m10q_get_state(void);
esp_err_t m10q_get_fix(m10q_fix_t *fix);
void m10q_get_dbg(uint32_t *rx_bytes, uint32_t *nmea_lines);
esp_err_t m10q_get_stats(m10q_stats_t *stats);
uint32_t m10q_get_gsv_count(void);

/* Sim-only (not a real m10q.h function): directly flips the mock's internal
 * "GNSS enabled" flag, standing in for the real lvgl_gps_set_enabled() ->
 * gps_ctrl_task power transition, which is skipped in the sim. See
 * gps_screen.c's gps_pwr_switch_cb(). */
void mock_gnss_set_enabled(bool on);

/* --- bhi260ap.h subset --- */
typedef enum {
    BHI260AP_ACTIVITY_STILL = 0,
    BHI260AP_ACTIVITY_WALKING,
    BHI260AP_ACTIVITY_RUNNING,
    BHI260AP_ACTIVITY_ON_BICYCLE,
    BHI260AP_ACTIVITY_IN_VEHICLE,
    BHI260AP_ACTIVITY_TILTING,
    BHI260AP_ACTIVITY_UNKNOWN,
} bhi260ap_activity_t;

esp_err_t bhi260ap_get_daily_steps(uint32_t *steps);
esp_err_t bhi260ap_get_status(bool *ready, uint32_t *steps);
esp_err_t bhi260ap_get_rotation(int16_t *x, int16_t *y, int16_t *z, int16_t *w,
                                uint16_t *accuracy);
esp_err_t bhi260ap_get_activity(uint8_t *activity);
esp_err_t bhi260ap_consume_gestures(bool *wrist_tilt, bool *wake_gesture, bool *glance,
                                    bool *pickup, bool *tilt);

/* --- daily_log.h subset --- */
typedef enum {
    DAILY_ACT_STILL,
    DAILY_ACT_WALKING,
    DAILY_ACT_RUNNING,
    DAILY_ACT_CYCLING,
    DAILY_ACT_VEHICLE,
    DAILY_ACT_TILTING,
    DAILY_ACT_UNKNOWN,
    DAILY_ACT_COUNT,
} daily_activity_t;

esp_err_t daily_log_get_activity_seconds(const uint32_t *out[DAILY_ACT_COUNT]);

/* --- tracking.h subset ---
 * TRACKING_ENABLED forced to 1 (the sim always has "tracking" wired up via
 * the mock start/stop below), so gps_screen.c's copied #if TRACKING_ENABLED
 * branch is the one compiled, matching the firmware's real branch. */
#define TRACKING_ENABLED 1

typedef struct {
    uint32_t dist_cm;
    uint32_t steps;
} tracking_totals_t;

bool tracking_is_active(void);
void tracking_get_totals(tracking_totals_t *totals);

/* --- lvgl_app.h subset (tracking start/stop are mocked here directly,
 * rather than the real lvgl_tracking_start/stop() which blanks the display
 * and drives the real tracking task) --- */
void lvgl_tracking_start(void);
void lvgl_tracking_stop(void);

/* --- mesh_log.h subset --- */
#define MESH_LOG_COUNT     8
#define MESH_LOG_TEXT_MAX  200

typedef enum {
    MESH_MSG_UNKNOWN = 0,
    MESH_MSG_TEXT,
    MESH_MSG_OTHER,
} mesh_msg_kind_t;

typedef struct {
    uint32_t from;
    char text[MESH_LOG_TEXT_MAX + 1];
    uint8_t channel_hash;
    mesh_msg_kind_t kind;
    int16_t rssi_dbm;
    int8_t snr_db;
    int64_t received_at_us;
} mesh_msg_t;

size_t mesh_log_get_recent(mesh_msg_t *out, size_t max);

/* --- alarm.h subset --- */
#define ALARM_RING_BEEP  0
#define ALARM_RING_VIB   1
#define ALARM_RING_BOTH  2

typedef struct {
    bool     enabled;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  ring_mode;
} alarm_config_t;

esp_err_t alarm_check(void);
bool alarm_is_snoozing(void);
void alarm_get_config(alarm_config_t *cfg);
esp_err_t alarm_set(uint8_t hour, uint8_t min, bool enabled, uint8_t ring_mode);
esp_err_t alarm_dismiss(void);
esp_err_t alarm_snooze(void);

/* --- sd_log.h / twatch_board.h / ble_debug.h subset (status bar) --- */
bool sd_log_available(void);
bool twatch_sd_card_seated(void);
bool ble_debug_is_connected(void);

#ifdef __cplusplus
}
#endif
