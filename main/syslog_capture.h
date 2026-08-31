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

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at boot, after twatch_board_init() (needs the SD card's ALDO1
 * rail bookkeeping and NVS ready, same as every other SD-backed logger). */
void syslog_capture_init(void);

#ifdef __cplusplus
}
#endif
