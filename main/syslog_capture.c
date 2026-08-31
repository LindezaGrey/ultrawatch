/*
 * syslog_capture.c - see syslog_capture.h.
 */
#include "syslog_capture.h"
#include "sd_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#define SYSLOG_FILE               "/sdcard/log/syslog.txt"
#define SYSLOG_BUF_SIZE           4096
/* Flush once the buffer is this full, rather than waiting for the next
 * periodic tick - leaves headroom for the one line that pushed it over,
 * which still gets appended (into the now-empty buffer) after the flush. */
#define SYSLOG_HIGH_WATER         (SYSLOG_BUF_SIZE * 3 / 4)
#define SYSLOG_FLUSH_INTERVAL_MS  10000
#define SYSLOG_LINE_MAX           256

static char s_buf[SYSLOG_BUF_SIZE];
static size_t s_buf_len;
static SemaphoreHandle_t s_mux;
static SemaphoreHandle_t s_flush_wake;
static vprintf_like_t s_prev_vprintf;

/* Tees every ESP_LOGx line into the RAM buffer, then always passes through
 * to whatever was previously installed (console output, and/or ble_debug's
 * own hook if that's ever re-enabled - see its header comment already
 * anticipating this). Never touches the SD card itself - that's the flush
 * task's job, so a burst of logging can't stall the caller on I/O. */
static int syslog_vprintf(const char *fmt, va_list args)
{
    char line[SYSLOG_LINE_MAX];
    va_list args_copy;
    va_copy(args_copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, args_copy);
    va_end(args_copy);

    /* Routine mount/unmount chatter (sd_log/sdspi_transaction) is generated
     * BY every flush this task does - buffering it would feed the next
     * flush, which mounts again and generates more of it: a self-sustaining
     * mount/unmount heartbeat forever, even on an otherwise idle system.
     * Confirmed live (reported as "very laggy" right after this feature
     * shipped) - excluded from capture, not from the live console. */
    if (n > 0 && (strstr(line, "sd_log:") || strstr(line, "sdspi_transaction:"))) {
        n = 0;
    }

    if (n > 0) {
        size_t len = (size_t)n < sizeof(line) - 1 ? (size_t)n : sizeof(line) - 1;
        if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (s_buf_len + len > SYSLOG_BUF_SIZE) {
                len = SYSLOG_BUF_SIZE - s_buf_len;   /* truncate rather than drop the whole line */
            }
            memcpy(s_buf + s_buf_len, line, len);
            s_buf_len += len;
            bool full = s_buf_len >= SYSLOG_HIGH_WATER;
            xSemaphoreGive(s_mux);
            if (full) {
                xSemaphoreGive(s_flush_wake);
            }
        }
    }

    if (s_prev_vprintf) {
        return s_prev_vprintf(fmt, args);
    }
    return vprintf(fmt, args);
}

static void syslog_flush(void)
{
    if (xSemaphoreTake(s_mux, portMAX_DELAY) != pdTRUE) {
        return;
    }
    if (s_buf_len == 0) {
        xSemaphoreGive(s_mux);
        return;
    }
    /* Copy out and reset the shared buffer before doing any SD I/O, so the
     * vprintf hook (called from any task) is never blocked on the card. */
    static char flush_copy[SYSLOG_BUF_SIZE];
    size_t len = s_buf_len;
    memcpy(flush_copy, s_buf, len);
    s_buf_len = 0;
    xSemaphoreGive(s_mux);

    if (sd_log_session_begin() != ESP_OK) {
        /* sd_log_session_begin() takes the SD session mutex unconditionally,
         * even on failure - it must be released here or every other SD
         * writer (daily_log, gpx_log, mesh_log, crash_dump, screenshots,
         * debug console commands) deadlocks forever on its own
         * sd_log_session_begin(). */
        sd_log_session_end();
        return;   /* no card - drop this window's log, matches every other logger's behavior */
    }
    FILE *f = fopen(SYSLOG_FILE, "a");
    if (f) {
        fwrite(flush_copy, 1, len, f);
        fclose(f);
    }
    sd_log_session_end();
}

void syslog_capture_service(uint32_t timeout_ms)
{
    xSemaphoreTake(s_flush_wake, pdMS_TO_TICKS(timeout_ms));
    syslog_flush();
}

void syslog_capture_init(void)
{
    s_mux = xSemaphoreCreateMutex();
    s_flush_wake = xSemaphoreCreateBinary();
    s_prev_vprintf = esp_log_set_vprintf(syslog_vprintf);
}
