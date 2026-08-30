/*
 * sd_log.h - SD card mount (on demand) + PNG screenshot capture.
 *
 * The SD card is mounted at /sdcard (FATFS over SPI2, CS=21) only for as
 * long as something is actually reading/writing it - sd_log_session_begin()/
 * sd_log_session_end() (below) bracket each burst of file I/O and are what
 * every writer (daily_log, gpx_log, mesh_log, crash_dump, the screenshot/
 * clear/debug-console helpers here) uses. A card left mounted for the whole
 * awake session sits exposed to corruption for however long that session
 * runs, for no benefit - minimizing the mounted window is the whole point.
 * sd_log_mount()/sd_log_unmount() are the low-level primitives the session
 * API (and power_mgmt.c's sleep/shutdown paths, as a belt-and-braces
 * unmount-before-power-off) call directly; new callers should reach for the
 * session API instead of these two.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mount the SD card (FATFS over SPI2, CS=21), powering ALDO1 on first. Logs
 * and continues on failure. Safe to call repeatedly (a no-op if already
 * mounted) - used both at startup and to remount after sd_log_unmount() (see
 * power_mgmt.c's sleep/wake handling). */
esp_err_t sd_log_mount(void);

/* Cleanly unmount the card and power ALDO1 off. Safe to call repeatedly (a
 * no-op if not mounted). Call this before cutting SD power any other way -
 * yanking ALDO1 without unmounting first is what leaves the card in an
 * undefined state that fails to remount with resp/CRC errors. */
esp_err_t sd_log_unmount(void);

/* True if the SD card is mounted and writable right now - only true for
 * the brief window between a sd_log_session_begin() and its matching
 * sd_log_session_end(), not for the whole awake session (see above). */
bool sd_log_available(void);

/* Acquire a mounted session: mounts on the first (0->1) caller, and every
 * call must be paired with exactly one sd_log_session_end(), including on
 * the failure path (unbalanced calls leak the mount open or unmount out
 * from under a still-active caller). Refcounted and mutex-guarded, so
 * independent tasks (daily_log/gpx_log/mesh_log/...) can each bracket
 * their own writes without unmounting on top of each other. Returns
 * ESP_ERR_NOT_FOUND (matching sd_log_mount()'s failure mode) if no card
 * could be mounted - sd_log_available() reflects the real state either
 * way. */
esp_err_t sd_log_session_begin(void);

/* Release one sd_log_session_begin() reference; unmounts on the last
 * (->0) release. */
void sd_log_session_end(void);

/* Total/free bytes on the card, via esp_vfs_fat_info(). ESP_ERR_NOT_FOUND
 * if no card is mounted (check sd_log_available() first, or just handle
 * the error). */
esp_err_t sd_log_get_space(uint64_t *total_bytes, uint64_t *free_bytes);

/* Write a screenshot (RGB565 little-endian, w*h pixels) to
 * /sdcard/shot/shot_NNNN.png. Returns ESP_OK on success, ESP_ERR_NOT_FOUND
 * if no card. */
esp_err_t sd_log_save_screenshot(const uint16_t *rgb565, int w, int h);

/* Delete all screenshots on the SD card. */
esp_err_t sd_log_clear(void);

#ifdef __cplusplus
}
#endif
