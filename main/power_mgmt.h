/*
 * power_mgmt.h - watch power management (DFS + automatic light sleep).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void power_mgmt_init(void);

/* Loads the NVS-persisted settings (night mode, USB-sleep, display timeout,
 * brightness) without touching GPIOs/tasks/ISRs. power_mgmt_init() already
 * calls this itself; exposed separately so lvgl_app_start() can load
 * power_mgmt_get_display_timeout_s() *before* esp_lv_adapter_init() (which
 * needs it immediately, ahead of where power_mgmt_init()'s heavier GPIO/task
 * setup has to run relative to sensor_cache_init()/alarm_init()). Safe to
 * call twice - power_mgmt_init() calling it again is a harmless re-load. */
void power_mgmt_load_config(void);

/* Adapter auto-sleep callbacks (PAUSE mode). */
esp_err_t power_mgmt_enter_sleep(void *ctx);
esp_err_t power_mgmt_exit_sleep(void *ctx);

/* Clean software shutdown: blank the display, unmount the SD card, then hand
 * off to the PMIC's own controlled power-off. Normally triggered by a
 * PWRKEY long-press (>4s) - see power_mgmt.c's pm_wake_task(). Exposed
 * publicly only so it can be triggered from the console for testing (no way
 * to simulate a physical 4s button hold otherwise); not meant to be called
 * from elsewhere in the app. */
void power_mgmt_shutdown(void);

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

/* Fired on every short PWRKEY press and every BOOT press while awake (never
 * on the PWRKEY long-press that triggers power_mgmt_shutdown()). A no-op
 * most of the time; exists so alarm.c can stop a ringing alarm from either
 * physical button without power_mgmt owning any alarm-specific logic. */
typedef void (*power_mgmt_button_cb_t)(void);
void power_mgmt_register_button_cb(power_mgmt_button_cb_t cb);

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

/* Display auto-off timeout, in seconds (default 5). Persisted in NVS, but
 * NOT applied live: it only takes effect via esp_lv_adapter_init()'s
 * auto_sleep.idle_timeout_ms at next boot - the adapter component exposes
 * no runtime reconfigure API. Documented exception to this project's usual
 * immediate-apply convention. */
uint32_t power_mgmt_get_display_timeout_s(void);
void power_mgmt_set_display_timeout_s(uint32_t seconds);

/* Normal (non-night-mode) display brightness, 0-255 (default 0x80).
 * Persisted in NVS and applied immediately via co5300_set_brightness()
 * unless night mode is currently active (in which case it takes effect the
 * next time night mode exits). */
uint8_t power_mgmt_get_brightness(void);
void power_mgmt_set_brightness(uint8_t level);

/* Ultra-Sparmodus (docs/application.md section 10): the persisted "user
 * wants this mode" flag, backed by NVS (survives a normal reboot) - NOT
 * the same as the RTC_DATA_ATTR flag in main/uwatch_main.c that survives a
 * *deep sleep* wake (this one is only read at boot/from Settings to decide
 * whether to start the deep-sleep cycle in the first place; the RTC one is
 * what a post-wake app_main() actually branches on, since NVS is far too
 * slow/heavy to touch on every silent per-minute wake). Stage 1 of Phase 6:
 * only the flag + Settings toggle exist so far, no real sleep/wake yet. */
bool power_mgmt_get_sparmodus_active(void);
void power_mgmt_set_sparmodus_active(bool on);

/* True if app_main() should immediately re-arm wake sources and go back
 * to deep sleep without doing anything else (booting from the silent
 * per-minute timer wake while Ultra-Sparmodus is active) - call this as
 * the very first thing in app_main(), before nvs_flash_init() or any
 * other init. Encapsulates both the RTC_DATA_ATTR flag (which survives a
 * deep-sleep wake) and the wake-cause check so app_main() itself stays a
 * one-line fork - see main/uwatch_main.c. */
bool power_mgmt_sparmodus_should_resleep_silently(void);

/* Real deep-sleep entry: shuts every peripheral down, arms wake sources
 * (touch/PWRKEY/BOOT/RTC-alarm + the 60s silent timer), and calls
 * esp_deep_sleep_start() - does not return. Currently reachable only via
 * the `sparmodus on` debug console command (see main/debug_cmds.c) while
 * Stage 3's auto-entry-on-idle-timeout wiring is still being built. */
void power_mgmt_sparmodus_enter_sleep(void);

/* Re-arms the same wake sources and re-enters deep sleep, touching
 * nothing else - what power_mgmt_sparmodus_should_resleep_silently()
 * returning true means to do. */
void power_mgmt_sparmodus_resleep(void);

#ifdef __cplusplus
}
#endif
