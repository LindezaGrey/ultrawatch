/*
 * bhi260ap.h - Bosch BHI260AP smart sensor / IMU (I2C, Bosch FSC protocol).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Activity recognition classes reported by BHY2_SENSOR_ID_AR. */
typedef enum {
    BHI260AP_ACTIVITY_STILL = 0,
    BHI260AP_ACTIVITY_WALKING,
    BHI260AP_ACTIVITY_RUNNING,
    BHI260AP_ACTIVITY_ON_BICYCLE,
    BHI260AP_ACTIVITY_IN_VEHICLE,
    BHI260AP_ACTIVITY_TILTING,
    BHI260AP_ACTIVITY_UNKNOWN,
} bhi260ap_activity_t;

/* Bring up the BHI260AP: upload the RAM firmware from the SPIFFS assets
 * partition, boot it, and enable the step counter. The assets partition must
 * be mounted first. On failure logs and returns an error (never aborts). */
esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev);

/* Poll the FIFO once (delivers queued sensor events to registered callbacks). */
esp_err_t bhi260ap_process_fifo(void);

/* Host (ESP32) sleep: switch the chip to AP-suspend mode so only the wake-up
 * sensors run at low power. The sensor rail (ALDO4) must stay powered; call
 * bhi260ap_ap_resume() on wake. */
esp_err_t bhi260ap_ap_suspend(void);

/* Host awake: leave AP-suspend mode and resume the normal sensor streams. */
esp_err_t bhi260ap_ap_resume(void);

/* Consume pending wake-up FIFO events until the INT line de-asserts high.
 * Call at wake-arm time (after AP-suspend) so a leftover WU event - e.g. a
 * real gesture that landed during the power-down window - doesn't hold the
 * level-wake line low and get mistaken for a stuck sensor. */
esp_err_t bhi260ap_drain_wakeup_fifo(void);

/* Reset the driver to the uninitialized state after the sensor rail was
 * power-cycled (e.g. auto-sleep), so it can be brought up again. */
void bhi260ap_deinit(void);

/* Age of the newest sensor sample in ms. Used to detect a dead/rebooted chip
 * after the sensor rail was power-cycled: a live 12.5 Hz accel stream keeps
 * this under ~1 s, while a powered-down or bootloader-mode chip goes stale. */
uint32_t bhi260ap_get_data_age_ms(void);

/* True while the chip is in AP-suspend (host sleeping, FIFO polling paused). */
bool bhi260ap_is_suspended(void);

/* Latest step count (persistent: on-chip counter + NVS base offset, so the
 * total survives reboots). */
esp_err_t bhi260ap_get_step_count(uint32_t *steps);

/* Steps taken since the most recent midnight (daily count). Computed as the
 * lifetime total minus the lifetime total captured at the last day-start (see
 * bhi260ap_daily_set_day_start). The day-start snapshot is persisted to NVS,
 * so it survives reboots; the caller is expected to call
 * bhi260ap_daily_set_day_start() once per day at 00:00. */
esp_err_t bhi260ap_get_daily_steps(uint32_t *steps);

/* Snapshot the current lifetime total as the start of a new day, so
 * bhi260ap_get_daily_steps() restarts near zero. Persisted to NVS. */
esp_err_t bhi260ap_daily_set_day_start(void);

/* Sensor status: ready flag and latest step count. Either out-param may be
 * NULL. Returns ESP_OK (never fails; reports the last known state). */
esp_err_t bhi260ap_get_status(bool *ready, uint32_t *steps);

/* Latest accelerometer values in mg. Any out-param may be NULL. */
esp_err_t bhi260ap_get_accel(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

/* Latest gyroscope values in degrees per second. Any out-param may be NULL. */
esp_err_t bhi260ap_get_gyro(int16_t *x_dps, int16_t *y_dps, int16_t *z_dps);

/* Orientation in degrees: pitch/roll computed from the accelerometer (always
 * available on this 6-DoF board); heading is always 0 (needs a magnetometer,
 * which the T-Watch Ultra does not have). Any out-param may be NULL. */
esp_err_t bhi260ap_get_orientation(int16_t *heading, int16_t *pitch, int16_t *roll);

/* Latest game-rotation-vector quaternion (x/y/z/w, Q14 scaling) and its
 * accuracy (0..3). From the 6-DoF GAMERV fusion (accel+gyro, no magnetometer
 * needed). Either out-param may be NULL. */
esp_err_t bhi260ap_get_rotation(int16_t *x, int16_t *y, int16_t *z, int16_t *w,
                                uint16_t *accuracy);

/* Latest activity-recognition class (bhi260ap_activity_t). */
esp_err_t bhi260ap_get_activity(uint8_t *activity);

/* Consume latched gesture events since the last call (wrist tilt, wake
 * gesture, glance, pickup, tilt detector). Each non-NULL out-param is set to
 * true if that gesture occurred and then cleared. */
esp_err_t bhi260ap_consume_gestures(bool *wrist_tilt, bool *wake_gesture, bool *glance,
                                    bool *pickup, bool *tilt);

/* Wake-up FIFO sample tracer (debug): enable to tally WU-FIFO sample ids,
 * then read per-id counts. Used to find the source of spurious wake events. */
void bhi260ap_wu_trace_start(void);
void bhi260ap_wu_trace_stop(void);
uint32_t bhi260ap_wu_trace_get_count(uint8_t id);
esp_err_t bhi260ap_set_sensor_rate(uint8_t id, float rate);

/* GNSS data-injection readiness probe: reports GPS sensor availability,
 * switches the BHI260AP into real-time injection mode and watches for a BSX
 * injected-sensor-config request naming the GPS physical sensor. Returns
 * BHY2_OK when a live GPS injection driver requests GPS data, else a negative
 * bhy2 error code. */
int8_t bhi260ap_gnss_inject_probe(void);
void bhi260ap_meta_hist_start(void);
void bhi260ap_meta_hist_stop(void);
uint8_t bhi260ap_meta_hist_len_get(void);
void bhi260ap_meta_hist_get(uint8_t idx, uint8_t *key, uint8_t *count);
void bhi260ap_meta_print(uint32_t n);

#ifdef __cplusplus
}
#endif
