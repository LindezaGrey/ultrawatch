/*
 * power_mgmt.h - watch power management (DFS + automatic light sleep).
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void power_mgmt_init(void);

/* Adapter auto-sleep callbacks (PAUSE mode). */
esp_err_t power_mgmt_enter_sleep(void *ctx);
esp_err_t power_mgmt_exit_sleep(void *ctx);

/* Night mode: auto-entered between these local-time hours (inclusive start,
 * exclusive end, wraps midnight). Compile-time constants for now. */
#define PM_NIGHT_START_HOUR  23
#define PM_NIGHT_END_HOUR    7
#define PM_NIGHT_BRIGHTNESS  0x1A   /* ~10% of full scale */

/* True while night mode is active (red-only, dim, touch wake disabled). */
bool power_mgmt_is_night_mode(void);

/* Re-evaluate night mode from the current time (call after peripherals that
 * re-enable touch input, e.g. the touch driver, are brought up). */
void power_mgmt_recheck_night_mode(void);

/* Optional callback fired on every night-mode state change, so the UI layer
 * can force a full redraw through the red-only transform. */
typedef void (*power_mgmt_night_mode_cb_t)(bool night);
void power_mgmt_register_night_mode_cb(power_mgmt_night_mode_cb_t cb);

/* Night-mode auto: when enabled (default) the watch auto-enters night mode
 * between PM_NIGHT_START_HOUR and PM_NIGHT_END_HOUR; when disabled, night
 * mode stays off regardless of the time. Persisted in NVS. */
bool power_mgmt_get_night_mode_auto(void);
void power_mgmt_set_night_mode_auto(bool on);

/* "Do not sleep while on USB": when enabled (default) the watch refuses
 * auto-sleep while VBUS is present (charging / development); disable to allow
 * sleep even when plugged in. Persisted in NVS. */
bool power_mgmt_get_skip_sleep_on_usb(void);
void power_mgmt_set_skip_sleep_on_usb(bool yes);

#ifdef __cplusplus
}
#endif
