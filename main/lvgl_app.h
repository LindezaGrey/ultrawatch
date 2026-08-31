/*
 * lvgl_app.h - LVGL integration via the Espressif esp_lvgl_adapter.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lvgl_app_start(void);

/* Firmware version string (esp_app_get_description()->version - ESP-IDF,
 * unavailable on the host) - lets main/screens/settings_screen.c's Info
 * page call one portable name; the sim mocks this to a fixed "sim"
 * string instead. */
const char *uwatch_firmware_version(void);

/* Invalidate the active LVGL screen so it repaints fully (used after wake,
 * when the panel GRAM was blanked during sleep). Safe to call from any task. */
void lvgl_force_redraw(void);

/* Diagnostics for the recurring "white screen after wake" reports (see
 * night_mode_draw_bitmap()'s own comment): a live incident showed the
 * panel-command retry logic (co5300_wake/display_on) reporting success
 * with no draw_bitmap warning either, yet the screen stayed white until a
 * later manual redraw - meaning the failure is happening somewhere
 * upstream of any existing log line. These let a debug command answer
 * "did a real pixel flush happen recently" without guessing:
 *   - lvgl_flush_count_get(): total esp_lcd_panel_draw_bitmap() attempts
 *     since boot (incremented regardless of success/failure) - compare a
 *     reading from before a white screen against one taken right when
 *     it's reported to see whether any flush was even attempted meanwhile.
 *   - lvgl_flush_age_ms_get(): ms since the last flush attempt, or
 *     UINT32_MAX if none has happened yet this boot. */
uint32_t lvgl_flush_count_get(void);
uint32_t lvgl_flush_age_ms_get(void);

/* Capture the active LVGL screen and print it to the console as base64 RGB565
 * between ==SHOT:WxH== and ==ENDSHOT== markers (debug). Call from any task. */
esp_err_t lvgl_app_dump_screenshot(void);

/* One-shot GNSS position check (background): power on, wait for a 3D fix,
 * persist the last-known position if moved (50 m gate), power off. */
void lvgl_gps_refresh(void);

/* True while GNSS is deliberately enabled (GPS screen switch). Defaults off;
 * the power manager uses this to keep the GNSS rail alive across sleep. */
bool lvgl_gps_enabled(void);

/* Enable/disable GNSS (single entry for the UI switch, console commands and
 * boot): persists the choice, updates the session on/off state and requests
 * the power transition on the GPS control task. */
void lvgl_gps_set_enabled(bool on);

/* Start/stop a step-gated tracking session (blanks the display while active). */
void lvgl_tracking_start(void);
void lvgl_tracking_stop(void);

/* Load the BHI260AP status screen (with the GAMERV orientation cube). */
void lvgl_show_bhi_screen(void);

/* Jump to the Mesh screen and wake the display (called by mesh_log.c's
 * always-on background listener when a new message arrives). Safe to call
 * from any task. */
void lvgl_mesh_screen_show(void);

#ifdef __cplusplus
}
#endif
