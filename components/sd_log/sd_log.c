/*
 * sd_log.c - SD card mount + RAM log ring buffer + PNG capture.
 *
 * Uses the board's SPI2 bus (already initialized by twatch_board) for the SD
 * card (CS=21, FATFS). A low-priority task flushes the RAM log ring to
 * /sdcard/log/uwatch.log. Screenshots are encoded with libpng (managed
 * component) and written to /sdcard/shot/.
 *
 * All SD I/O happens in the flush task; the vprintf hook only appends to the
 * ring buffer so it never blocks on slow flash writes.
 */
#include "sd_log.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_log_write.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>

#include "png.h"

static const char *TAG = "sd_log";

#define SD_LOG_BASE      "/sdcard"
#define SD_LOG_DIR       "/sdcard/log"
#define SD_LOG_FILE      "/sdcard/log/uwatch.log"
#define SD_SHOT_DIR      "/sdcard/shot"
#define SD_CS_GPIO       21

#define LOG_RING_SIZE    8192
#define LOG_FLUSH_INTERVAL_MS  2000

/* ---- RAM log ring buffer ---- */
static char s_ring[LOG_RING_SIZE];
static size_t s_ring_len;
static SemaphoreHandle_t s_ring_mux;
static bool s_sd_ready;

static sdmmc_card_t *s_card;
static vprintf_like_t s_prev_vprintf;

/* Append one formatted line to the ring (keeps the tail). */
static void ring_append(const char *fmt, va_list args)
{
    char line[256];
    int n = vsnprintf(line, sizeof(line), fmt, args);
    if (n <= 0) {
        return;
    }
    if (n >= (int)sizeof(line)) {
        n = (int)sizeof(line) - 1;
    }

    if (xSemaphoreTake(s_ring_mux, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    if ((size_t)n + 1 > sizeof(s_ring) - s_ring_len) {
        /* Drop oldest lines to make room. */
        size_t need = (size_t)n + 1;
        size_t room = sizeof(s_ring) - s_ring_len;
        size_t drop = need - room;
        size_t start = 0;
        size_t found = 0;
        for (size_t i = 0; i < s_ring_len && found < drop; i++) {
            if (s_ring[i] == '\n') {
                found++;
                start = i + 1;
            }
        }
        if (start == 0) {
            s_ring_len = 0;   /* no newlines; discard all */
        } else {
            memmove(s_ring, s_ring + start, s_ring_len - start);
            s_ring_len -= start;
        }
    }
    memcpy(s_ring + s_ring_len, line, (size_t)n);
    s_ring_len += (size_t)n;
    if (s_ring_len + 1 < sizeof(s_ring)) {
        s_ring[s_ring_len] = '\n';
        s_ring_len++;
    }
    xSemaphoreGive(s_ring_mux);
}

/* esp_log vprintf hook: tee every line into the ring, keep console output. */
static int log_hook(const char *fmt, va_list args)
{
    va_list ap2;
    va_copy(ap2, args);
    ring_append(fmt, ap2);
    va_end(ap2);
    if (s_prev_vprintf) {
        return s_prev_vprintf(fmt, args);
    }
    return vprintf(fmt, args);
}

/* ---- PNG encoding via libpng ---- */

/* Allocate libpng's internal buffers from PSRAM to avoid exhausting the
 * internal DMA-capable heap that the display driver needs. */
static png_voidp png_psram_malloc(png_structp png_ptr, png_alloc_size_t size)
{
    (void)png_ptr;
    return heap_caps_malloc((size_t)size, MALLOC_CAP_SPIRAM);
}

static void png_psram_free(png_structp png_ptr, png_voidp ptr)
{
    (void)png_ptr;
    free(ptr);
}

static void png_write_cb(png_structp png_ptr, png_bytep data, png_size_t length)
{
    FILE *f = (FILE *)png_get_io_ptr(png_ptr);
    fwrite(data, 1, length, f);
}

static void png_flush_cb(png_structp png_ptr)
{
    FILE *f = (FILE *)png_get_io_ptr(png_ptr);
    fflush(f);
}

static esp_err_t write_png(const char *path, const uint16_t *rgb565, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        return ESP_FAIL;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png ? png_create_info_struct(png) : NULL;
    if (!png || !info) {
        if (png) {
            png_destroy_write_struct(&png, info ? &info : NULL);
        }
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    png_set_mem_fn(png, NULL, png_psram_malloc, png_psram_free);

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(f);
        return ESP_FAIL;
    }

    png_set_write_fn(png, f, png_write_cb, png_flush_cb);
    png_set_IHDR(png, info, (png_uint_32)w, (png_uint_32)h, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    /* Convert RGB565 -> RGB888 per scanline. */
    png_bytep row = (png_bytep)heap_caps_malloc((size_t)w * 3, MALLOC_CAP_SPIRAM);
    if (!row) {
        png_destroy_write_struct(&png, &info);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint16_t v = rgb565[y * w + x];
            row[x * 3 + 0] = (png_byte)(((v >> 11) & 0x1F) * 255 / 31);
            row[x * 3 + 1] = (png_byte)(((v >> 5) & 0x3F) * 255 / 63);
            row[x * 3 + 2] = (png_byte)((v & 0x1F) * 255 / 31);
        }
        png_write_row(png, row);
    }
    free(row);

    png_write_end(png, info);
    png_destroy_write_struct(&png, &info);
    fclose(f);
    return ESP_OK;
}

/* ---- Flush task ---- */

static void flush_task(void *arg)
{
    (void)arg;
    char buf[2048];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LOG_FLUSH_INTERVAL_MS));
        if (!s_sd_ready) {
            continue;
        }
        sd_log_flush();
        (void)buf;
    }
}

/* ---- Public API ---- */

esp_err_t sd_log_mount(void)
{
    if (s_card) {
        return ESP_OK;   /* already mounted */
    }

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 0,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS_GPIO;

    esp_err_t err = esp_vfs_fat_sdspi_mount(SD_LOG_BASE, &host, &slot, &mount_cfg, &s_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s (logging to serial only)", esp_err_to_name(err));
        s_sd_ready = false;
        return err;
    }

    mkdir(SD_LOG_DIR, 0755);
    mkdir(SD_SHOT_DIR, 0755);
    s_sd_ready = true;
    ESP_LOGI(TAG, "SD mounted: %u MB", (unsigned)(s_card->csd.capacity / 1024 / 1024));
    return ESP_OK;
}

bool sd_log_available(void)
{
    return s_sd_ready;
}

esp_err_t sd_log_start(void)
{
    if (!s_ring_mux) {
        s_ring_mux = xSemaphoreCreateMutex();
    }
    if (!s_ring_mux) {
        return ESP_ERR_NO_MEM;
    }

    /* Install the vprintf hook (keep the previous one so console still works). */
    s_prev_vprintf = esp_log_set_vprintf(log_hook);

    xTaskCreate(flush_task, "sd_flush", 4096, NULL, 2, NULL);
    return ESP_OK;
}

void sd_log_flush(void)
{
    if (!s_sd_ready || !s_ring_mux) {
        return;
    }
    size_t n = 0;
    if (xSemaphoreTake(s_ring_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    n = s_ring_len;
    s_ring_len = 0;
    xSemaphoreGive(s_ring_mux);
    if (n == 0) {
        return;
    }

    FILE *f = fopen(SD_LOG_FILE, "a");
    if (!f) {
        return;
    }
    fwrite(s_ring, 1, n, f);
    fclose(f);
}

esp_err_t sd_log_save_screenshot(const uint16_t *rgb565, int w, int h)
{
    if (!s_sd_ready) {
        return ESP_ERR_NOT_FOUND;
    }
    static uint32_t counter;
    counter++;
    char path[64];
    snprintf(path, sizeof(path), "%s/shot_%04lu.png", SD_SHOT_DIR, (unsigned long)counter);
    return write_png(path, rgb565, w, h);
}
