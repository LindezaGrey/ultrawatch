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

#ifdef __cplusplus
}
#endif
