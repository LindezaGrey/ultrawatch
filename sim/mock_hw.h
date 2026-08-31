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
#define ESP_ERR_INVALID_ARG 0x102

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
uint32_t power_mgmt_get_display_timeout_s(void);
void power_mgmt_set_display_timeout_s(uint32_t seconds);
uint8_t power_mgmt_get_brightness(void);
void power_mgmt_set_brightness(uint8_t level);
bool power_mgmt_get_sparmodus_active(void);
void power_mgmt_set_sparmodus_active(bool on);

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
 * Kept around even though gps_screen.c no longer calls into it (Phase 3
 * repointed the Start/Stop button to gpx_log_* below) - tracking.c's
 * pedometer feature is a separate, still out-of-scope concept, not removed
 * from the mock surface just because its one UI hook moved elsewhere. */
#define TRACKING_ENABLED 1

typedef struct {
    uint32_t dist_cm;
    uint32_t steps;
} tracking_totals_t;

bool tracking_is_active(void);
void tracking_get_totals(tracking_totals_t *totals);

/* --- gpx_log.h subset --- */
esp_err_t gpx_log_start(void);
esp_err_t gpx_log_stop(void);
bool gpx_log_is_active(void);
uint32_t gpx_log_point_count(void);

/* --- lvgl_app.h subset (tracking start/stop are mocked here directly,
 * rather than the real lvgl_tracking_start/stop() which blanks the display
 * and drives the real tracking task) --- */
void lvgl_tracking_start(void);
void lvgl_tracking_stop(void);

/* --- mesh_log.h subset --- */
#define MESH_LOG_COUNT     8
#define MESH_LOG_TEXT_MAX  200

/* Same clock the mock's received_at_us values are stamped with
 * (mock_now_s()*1e6, see mesh_log_get_recent()'s mock in mock_hw.c) -
 * lets the shared mesh_screen.c call one portable name instead of the
 * real esp_timer_get_time(). */
int64_t mesh_log_now_us(void);

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

/* ---- LoRa message presets (mesh_log.h subset) ---- */
#define MESH_PRESET_COUNT   4
#define MESH_PRESET_MAX_LEN 31

/* Copies preset `idx` into `out` ("" if idx is out of range). Mock keeps
 * the same 4 in-memory strings the real defaults use (not NVS-backed -
 * mesh_preset_set() edits are lost on restart, same as every other
 * mocked "on" state in this file). */
void mesh_preset_get(int idx, char *out, size_t outlen);
void mesh_preset_set(int idx, const char *text);

#define MESH_NODE_TABLE_MAX 32

typedef struct {
    uint32_t node_id;
    char     name[32];
    int64_t  last_seen_us;
    int16_t  last_rssi_dbm;
    int8_t   last_snr_db;
} mesh_node_t;

size_t mesh_log_get_nodes(mesh_node_t *out, size_t max);
size_t mesh_log_node_count(void);
void mesh_log_node_name(uint32_t node_id, char *out, size_t outlen);
bool mesh_log_get_notify_enabled(void);
void mesh_log_set_notify_enabled(bool enabled);
bool mesh_log_get_enabled(void);
void mesh_log_set_enabled(bool on);

/* --- wifi_scan.h subset --- */
#define WIFI_SCAN_MAX_RESULTS 16
#define WIFI_SCAN_SSID_MAX    32

typedef struct {
    char ssid[WIFI_SCAN_SSID_MAX + 1];
    int8_t rssi_dbm;
    bool open;
} wifi_scan_result_t;

bool wifi_scan_get_enabled(void);
void wifi_scan_set_enabled(bool on);
bool wifi_scan_is_scanning(void);
bool wifi_scan_low_mem(void);
size_t wifi_scan_get_results(wifi_scan_result_t *out, size_t max);

/* --- ble_scan.h subset --- */
#define BLE_SCAN_MAX_RESULTS 20
#define BLE_SCAN_NAME_MAX    31

typedef struct {
    char name[BLE_SCAN_NAME_MAX + 1];
    uint8_t addr[6];
    int8_t rssi_dbm;
} ble_scan_result_t;

bool ble_scan_get_enabled(void);
void ble_scan_set_enabled(bool on);
bool ble_scan_is_scanning(void);
bool ble_scan_low_mem(void);
size_t ble_scan_get_results(ble_scan_result_t *out, size_t max);

/* --- haptic.h subset --- */
typedef struct {
    const char *name;
    uint8_t wave_id;
} haptic_pattern_t;

#define HAPTIC_PATTERN_COUNT 6
extern const haptic_pattern_t HAPTIC_PATTERNS[HAPTIC_PATTERN_COUNT];

size_t haptic_get_pattern_index(void);
void haptic_set_pattern_index(size_t idx);
uint8_t haptic_get_wave_id(void);
void haptic_play_test(uint8_t wave_id);
void haptic_play_test_async(uint8_t wave_id);

/* --- alarm.h subset (multi-alarm + shared ring engine, 2026-08-29 overhaul) --- */
#define ALARM_RING_BEEP  0
#define ALARM_RING_VIB   1
#define ALARM_RING_BOTH  2

#define ALARM_MAX_COUNT   8
#define ALARM_WEEKDAY_ALL 0x7Fu

typedef struct {
    bool     in_use;
    bool     enabled;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  ring_mode;
    uint8_t  weekday_mask;
} alarm_entry_t;

typedef enum {
    ALARM_RING_SOURCE_ALARM = 0,
    ALARM_RING_SOURCE_TIMER,
} alarm_ring_source_t;

esp_err_t alarm_check(void);
bool alarm_is_ringing(void);
bool alarm_is_snoozing(void);
bool alarm_is_armed(void);
int alarm_add(uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask);
esp_err_t alarm_update(int idx, uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask);
esp_err_t alarm_remove(int idx);
esp_err_t alarm_set_enabled(int idx, bool enabled);
size_t alarm_get_all(alarm_entry_t *out, size_t max);
int alarm_get_ringing_index(void);
esp_err_t alarm_dismiss(void);
esp_err_t alarm_snooze(void);
bool alarm_get_sound_enabled(void);
void alarm_set_sound_enabled(bool enabled);

/* --- cd_timer.h subset ---
 * cdtimer_check() is a real cd_timer.h function (called every second from
 * watch_face.c's shared update timer, same as the firmware) - mocked here
 * as a simplified decrement-only version that does not fire
 * alarm_ring_now() on expiry (alarm_ring_now() isn't mocked), unlike the
 * real cd_timer.c. */
esp_err_t cdtimer_start(uint32_t seconds);
void cdtimer_cancel(void);
bool cdtimer_is_active(void);
uint32_t cdtimer_remaining_seconds(void);
void cdtimer_check(void);

/* --- sd_log.h / twatch_board.h / ble_debug.h subset (status bar) --- */
bool sd_log_available(void);
bool twatch_sd_card_seated(void);
bool ble_debug_is_connected(void);
bool ble_debug_is_advertising(void);
void ble_debug_set_advertising(bool on);
esp_err_t sd_log_get_space(uint64_t *total_bytes, uint64_t *free_bytes);

/* --- lvgl_app.h subset (Settings/Peripherie screen + the shared
 * gps_screen.c's gps_pwr_switch_cb()) ---
 * Real lvgl_gps_set_enabled() drives the GNSS power task; the sim has no
 * task, so this mock just calls mock_gnss_set_enabled() directly. */
void lvgl_gps_set_enabled(bool on);

/* --- esp_lv_adapter.h subset ---
 * Real esp_lv_adapter_report_activity() resets the idle-sleep timer; no-op
 * here, the sim has no auto-sleep to defer. */
esp_err_t esp_lv_adapter_report_activity(void);

/* Real uwatch_firmware_version() wraps esp_app_get_description()->version
 * (ESP-IDF, unavailable on the host) - mocked to a fixed string here for
 * the Settings Info page. */
const char *uwatch_firmware_version(void);

#ifdef __cplusplus
}
#endif
