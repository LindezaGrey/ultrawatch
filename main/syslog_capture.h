/*
 * syslog_capture.h - mirrors ESP_LOGx output to the SD card
 * (/sdcard/log/syslog.txt), so a transient glitch (e.g. the white-screen
 * DMA-retry warnings, main/lvgl_app.c's night_mode_draw_bitmap()) leaves
 * durable evidence instead of only a live serial console that's easy to
 * miss or lose on reconnect.
 *
 * Chains onto whatever esp_log_set_vprintf() hook was already installed
 * (or the libc default) - console output is unaffected, this only adds a
 * second destination. Buffered in RAM and flushed to SD every
 * SYSLOG_FLUSH_INTERVAL_MS or immediately if the buffer fills first, so a
 * burst of logging never blocks the caller on SD I/O.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at boot, after twatch_board_init() (needs the SD card's ALDO1
 * rail bookkeeping and NVS ready, same as every other SD-backed logger).
 * Only sets up the mutex/wake-semaphore and installs the vprintf hook -
 * does not start a task of its own, see syslog_capture_service() below. */
void syslog_capture_init(void);

/* Blocks up to timeout_ms (or returns early if the buffer filled - see
 * syslog_vprintf()'s high-water-mark wake), then flushes whatever's
 * pending. Called in a loop from the shared housekeeping task
 * (main/housekeeping.c) instead of this module owning its own task -
 * reclaims a whole task stack (see docs/application.md section 12). Pass
 * 10000 (10s) to match this module's original standalone cadence. */
void syslog_capture_service(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
