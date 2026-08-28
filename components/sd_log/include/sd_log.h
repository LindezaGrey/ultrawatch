/*
 * sd_log.h - SD card mount (on demand) + PNG screenshot capture.
 *
 * The SD card is mounted at /sdcard (FATFS over SPI2, CS=21) only while the
 * watch is awake: sd_log_mount() powers ALDO1 on and mounts; sd_log_unmount()
 * cleanly unmounts and powers ALDO1 off (see power_mgmt.c's sleep/wake
 * handling). Screenshots can be written as PNG to /sdcard/shot/ when a card
 * is mounted; otherwise the serial fallback is used by the caller.
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

/* True if the SD card is mounted and writable. */
bool sd_log_available(void);

/* Write a screenshot (RGB565 little-endian, w*h pixels) to
 * /sdcard/shot/shot_NNNN.png. Returns ESP_OK on success, ESP_ERR_NOT_FOUND
 * if no card. */
esp_err_t sd_log_save_screenshot(const uint16_t *rgb565, int w, int h);

/* Delete all screenshots on the SD card. */
esp_err_t sd_log_clear(void);

#ifdef __cplusplus
}
#endif
