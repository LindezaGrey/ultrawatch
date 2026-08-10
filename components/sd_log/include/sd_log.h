/*
 * sd_log.h - SD card mount + RAM log buffer with periodic flush + PNG capture.
 *
 * The SD card is mounted at /sdcard (FATFS over SPI). An esp_log vprintf hook
 * tees every log line into a small RAM ring buffer; a low-priority task flushes
 * it to /sdcard/log/uwatch.log periodically. Screenshots can be written as PNG
 * to /sdcard/shot/ when a card is present; otherwise the serial fallback is used
 * by the caller.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mount the SD card (FATFS over SPI2, CS=21). Logs and continues on failure.
 * Safe to call once at startup. */
esp_err_t sd_log_mount(void);

/* True if the SD card is mounted and writable. */
bool sd_log_available(void);

/* Start the RAM log ring + flush task (call after sd_log_mount). */
esp_err_t sd_log_start(void);

/* Write a screenshot (RGB565 little-endian, w*h pixels) to /sdcard/shot/NNNN.png.
 * Returns ESP_OK on success, ESP_ERR_NOT_FOUND if no card. */
esp_err_t sd_log_save_screenshot(const uint16_t *rgb565, int w, int h);

/* Flush any buffered log lines to disk now (called by the flush task and on
 * shutdown). */
void sd_log_flush(void);

/* Truncate the log file and delete all screenshots on the SD card. */
esp_err_t sd_log_clear(void);

#ifdef __cplusplus
}
#endif
