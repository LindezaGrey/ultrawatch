/*
 * system/syslog_capture.c - see syslog_capture.h.
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

static const char *TAG = "syslog_cap";

#define SYSLOG_FILE               "/sdcard/log/syslog.txt"
#define SYSLOG_OLD_FILE           "/sdcard/log/syslog.1.txt"
/* FATFS has no append-side cap, so the mirror only ever grew: 8.0 MB over six
 * weeks, which turned a bare `synclog` full-file scan into a ~200 s console
 * read and left the interesting part (the hour before a reboot) buried. Two
 * generations keeps the window before a failure without an unbounded file. */
#define SYSLOG_MAX_BYTES          (2u * 1024u * 1024u)
/* The telemetry writers share this card; if it gets tight, drop the old
 * generation rather than let the log mirror be what fills it. */
#define SYSLOG_MIN_FREE_BYTES     (8u * 1024u * 1024u)
/* Ping-pong pair: while one batch is being written to the card, capture keeps
 * filling the other, so a flush costs no copy and the RAM mirror holds twice
 * as much. Internal DRAM only - the hook runs in any task, including ones that
 * may be inside a flash operation with the cache disabled. */
#define SYSLOG_BUF_SIZE           8192
/* Flush once the buffer is this full, rather than waiting for the next
 * periodic tick - leaves headroom for the one line that pushed it over,
 * which still gets appended (into the now-empty buffer) after the flush. */
#define SYSLOG_HIGH_WATER         (SYSLOG_BUF_SIZE * 3 / 4)
#define SYSLOG_LINE_MAX           256

static char s_bufs[2][SYSLOG_BUF_SIZE];
static int s_buf_idx;                    /* the buffer capture is filling */
static size_t s_buf_len;
static SemaphoreHandle_t s_mux;
static SemaphoreHandle_t s_flush_wake;
static vprintf_like_t s_prev_vprintf;

/* Put a failed flush back ahead of lines captured while SD I/O was in
 * progress. Keeping the oldest batch takes priority if the fixed RAM buffer
 * filled during the failure, so the next successful flush stays ordered. */
static void syslog_restore(const char *data, size_t len)
{
    if (xSemaphoreTake(s_mux, portMAX_DELAY) != pdTRUE) {
        return;
    }
    char *buf = s_bufs[s_buf_idx];
    if (len >= SYSLOG_BUF_SIZE) {
        memcpy(buf, data, SYSLOG_BUF_SIZE);
        s_buf_len = SYSLOG_BUF_SIZE;
    } else {
        size_t retained_len = s_buf_len;
        if (retained_len > SYSLOG_BUF_SIZE - len) {
            retained_len = SYSLOG_BUF_SIZE - len;
        }
        memmove(buf + len, buf, retained_len);
        memcpy(buf, data, len);
        s_buf_len = len + retained_len;
    }
    bool full = s_buf_len >= SYSLOG_HIGH_WATER;
    xSemaphoreGive(s_mux);
    if (full) {
        xSemaphoreGive(s_flush_wake);
    }
}

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
            char *buf = s_bufs[s_buf_idx];
            if (s_buf_len + len > SYSLOG_BUF_SIZE) {
                len = SYSLOG_BUF_SIZE - s_buf_len;   /* truncate rather than drop the whole line */
            }
            memcpy(buf + s_buf_len, line, len);
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

/* syslog.txt -> syslog.1.txt, dropping the previous generation. */
static void syslog_rotate(void)
{
    remove(SYSLOG_OLD_FILE);
    if (rename(SYSLOG_FILE, SYSLOG_OLD_FILE) != 0) {
        /* Nothing to rename, or the card refused: start the mirror over rather
         * than keep appending past the cap. */
        remove(SYSLOG_FILE);
    }
    ESP_LOGI(TAG, "syslog rotated, %s holds the previous generation", SYSLOG_OLD_FILE);
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
    /* Hand capture the other buffer and keep this batch for ourselves, so the
     * vprintf hook (called from any task) is never blocked on the card and the
     * batch needs no copy-out. */
    int idx = s_buf_idx;
    s_buf_idx ^= 1;
    size_t len = s_buf_len;
    s_buf_len = 0;
    xSemaphoreGive(s_mux);

    if (sd_log_session_begin() != ESP_OK) {
        /* sd_log_session_begin() takes the SD session mutex unconditionally,
         * even on failure - it must be released here or every other SD
         * writer (daily_log, gpx_log, mesh_log, crash_dump, screenshots,
         * debug console commands) deadlocks forever on its own
         * sd_log_session_begin(). */
        sd_log_session_end();
        syslog_restore(s_bufs[idx], len);
        return;
    }
    uint64_t total = 0, free_bytes = 0;
    if (sd_log_get_space(&total, &free_bytes) == ESP_OK &&
            free_bytes < SYSLOG_MIN_FREE_BYTES &&
            remove(SYSLOG_OLD_FILE) == 0) {
        ESP_LOGW(TAG, "SD low (%llu MB free), dropped %s",
                 (unsigned long long)(free_bytes / (1024u * 1024u)), SYSLOG_OLD_FILE);
    }
    FILE *f = fopen(SYSLOG_FILE, "a");
    bool saved = false;
    if (f) {
        long size = ftell(f);
        if (size >= 0 && (unsigned long)size + len > SYSLOG_MAX_BYTES) {
            fclose(f);
            syslog_rotate();
            f = fopen(SYSLOG_FILE, "a");
        }
        if (f) {
            size_t written = fwrite(s_bufs[idx], 1, len, f);
            int close_rc = fclose(f);
            saved = written == len && close_rc == 0;
        }
    }
    sd_log_session_end();
    if (!saved) {
        syslog_restore(s_bufs[idx], len);
    }
}

void syslog_capture_service(uint32_t timeout_ms)
{
    static TickType_t last_flush;
    if (!s_flush_wake) {
        /* init() failed to allocate: keep the caller's cadence, do nothing. */
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return;
    }
    bool signaled = xSemaphoreTake(s_flush_wake, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    TickType_t now = xTaskGetTickCount();
    if (signaled || now - last_flush >= pdMS_TO_TICKS(60000)) {
        syslog_flush();
        last_flush = now;
    }
}

void syslog_capture_init(void)
{
    s_mux = xSemaphoreCreateMutex();
    s_flush_wake = xSemaphoreCreateBinary();
    if (!s_mux || !s_flush_wake) {
        /* Leave both unset so the guards in the capture/flush paths cover
         * the whole module instead of half of it. */
        ESP_LOGE(TAG, "out of memory, syslog capture disabled");
        if (s_mux) {
            vSemaphoreDelete(s_mux);
        }
        if (s_flush_wake) {
            vSemaphoreDelete(s_flush_wake);
        }
        s_mux = s_flush_wake = NULL;
        return;
    }
    s_prev_vprintf = esp_log_set_vprintf(syslog_vprintf);
}
