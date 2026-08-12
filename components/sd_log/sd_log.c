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
#include <dirent.h>
#include "freertos/semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>

#include "png.h"
#include "twatch_board.h"
#include "axp2101.h"

static const char *TAG = "sd_log";

#define SD_LOG_BASE      "/sdcard"
#define SD_LOG_DIR       "/sdcard/log"
#define SD_LOG_FILE      "/sdcard/log/uwatch.log"
#define SD_VERSION_FILE  "/sdcard/log/.lastver"
#define SD_SHOT_DIR      "/sdcard/shot"
#define SD_CS_GPIO       21

#define LOG_RING_SIZE    8192
#define LOG_FLUSH_INTERVAL_MS  2000
#define SHOT_KEEP_MAX    10   /* retain only the newest screenshots */

/* ---- RAM log ring buffer ----
 * CPU-only buffer (never DMA), so it lives in PSRAM to spare the small
 * internal DMA heap the display driver needs. */
static char *s_ring;
static size_t s_ring_len;
static SemaphoreHandle_t s_ring_mux;
static bool s_sd_ready;

static sdmmc_card_t *s_card;
static vprintf_like_t s_prev_vprintf;
static char s_version[64];   /* running firmware version/hash, if set */
static bool s_card_power_cycled;   /* ALDO1 already toggled this boot */

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
    if ((size_t)n + 1 > LOG_RING_SIZE - s_ring_len) {
        /* Drop oldest lines to make room. */
        size_t need = (size_t)n + 1;
        size_t room = LOG_RING_SIZE - s_ring_len;
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
    /* Bound-check before copying: s_ring_len must stay < LOG_RING_SIZE. */
    if (s_ring_len + (size_t)n >= LOG_RING_SIZE) {
        s_ring_len = 0;   /* should not happen after drop above; be safe */
    }
    memcpy(s_ring + s_ring_len, line, (size_t)n);
    s_ring_len += (size_t)n;
    if (s_ring_len + 1 < LOG_RING_SIZE) {
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
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LOG_FLUSH_INTERVAL_MS));
        if (!s_sd_ready) {
            continue;
        }
        sd_log_flush();
    }
}

/* ---- Public API ---- */

void sd_log_set_version(const char *version)
{
    if (version) {
        strncpy(s_version, version, sizeof(s_version) - 1);
        s_version[sizeof(s_version) - 1] = '\0';
    }
}

/* Reset the log when the firmware build (git hash) changed since the last
 * boot. The recorded version is stored in /sdcard/log/.lastver. */
static void check_version_reset(void)
{
    if (s_version[0] == '\0') {
        return;   /* no version known; keep whatever is on the card */
    }
    char old[sizeof(s_version)] = { 0 };
    FILE *f = fopen(SD_VERSION_FILE, "r");
    if (f) {
        size_t n = fread(old, 1, sizeof(old) - 1, f);
        old[n] = '\0';
        fclose(f);
        /* strip trailing newline/CR */
        while (n > 0 && (old[n - 1] == '\n' || old[n - 1] == '\r')) {
            old[--n] = '\0';
        }
    }
    if (strcmp(old, s_version) != 0) {
        /* New build: start a fresh log. */
        f = fopen(SD_LOG_FILE, "w");
        if (f) {
            fclose(f);
        }
        f = fopen(SD_VERSION_FILE, "w");
        if (f) {
            fprintf(f, "%s\n", s_version);
            fclose(f);
        }
        ESP_LOGI(TAG, "firmware version %s (was '%s'); log reset", s_version, old);
    }
}

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

    /* After a soft reset (esp_restart / watchdog / USB reflash) the card is
     * left powered with its SD registers in an undefined state, so the first
     * init usually fails with CRC/response errors. Power-cycle the SD rail
     * (ALDO1) to force a clean card power-on, then retry a few times. */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS_GPIO;

    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0 || !s_card_power_cycled) {
            /* Toggle the SD rail: off, settle, on, settle. */
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, false);
            vTaskDelay(pdMS_TO_TICKS(50));
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, true);
            vTaskDelay(pdMS_TO_TICKS(100));
            s_card_power_cycled = true;
        }
        err = esp_vfs_fat_sdspi_mount(SD_LOG_BASE, &host, &slot, &mount_cfg, &s_card);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "SD mount attempt %d failed: %s", attempt + 1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s (logging to serial only)", esp_err_to_name(err));
        s_sd_ready = false;
        return err;
    }

    mkdir(SD_LOG_DIR, 0755);
    mkdir(SD_SHOT_DIR, 0755);
    check_version_reset();
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
    if (!s_ring) {
        s_ring = heap_caps_malloc(LOG_RING_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_ring) {
            return ESP_ERR_NO_MEM;
        }
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
    /* Hold the mutex for the whole write. The ring is contiguous (compacts,
     * never wraps) and small (LOG_RING_SIZE), so the SD append takes a few ms
     * at most; ring_append() just blocks briefly instead of racing the write.
     * No large stack buffer needed (the flush task has a limited stack). */
    if (xSemaphoreTake(s_ring_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    size_t n = s_ring_len;
    if (n == 0) {
        xSemaphoreGive(s_ring_mux);
        return;
    }
    FILE *f = fopen(SD_LOG_FILE, "a");
    if (f) {
        fwrite(s_ring, 1, n, f);
        fclose(f);
    }
    s_ring_len = 0;
    xSemaphoreGive(s_ring_mux);
}

esp_err_t sd_log_clear(void)
{
    if (!s_sd_ready) {
        return ESP_ERR_NOT_FOUND;
    }
    /* Truncate the log. */
    FILE *f = fopen(SD_LOG_FILE, "w");
    if (!f) {
        return ESP_FAIL;
    }
    fclose(f);

    /* Delete screenshots. */
    DIR *d = opendir(SD_SHOT_DIR);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL) {
            if (strncmp(e->d_name, "shot_", 5) == 0) {
                char name[32];
                strncpy(name, e->d_name, sizeof(name) - 1);
                name[sizeof(name) - 1] = '\0';
                char path[128];
                snprintf(path, sizeof(path), "%s/%s", SD_SHOT_DIR, name);
                remove(path);
            }
        }
        closedir(d);
    }
    return ESP_OK;
}

/* Find the highest existing shot_NNNN.png number in the shot dir (0 if none). */
static uint32_t shot_next_counter(void)
{
    uint32_t max = 0;
    DIR *d = opendir(SD_SHOT_DIR);
    if (!d) {
        return 0;
    }
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        unsigned n = 0;
        if (sscanf(e->d_name, "shot_%u.png", &n) == 1 && n > max) {
            max = n;
        }
    }
    closedir(d);
    return max;
}

/* Delete all but the SHOT_KEEP_MAX newest screenshots (by shot number). */
static void trim_shots(void)
{
    uint32_t nums[SHOT_KEEP_MAX] = { 0 };
    int cnt = 0;
    DIR *d = opendir(SD_SHOT_DIR);
    if (!d) {
        return;
    }
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        unsigned n = 0;
        if (sscanf(e->d_name, "shot_%u.png", &n) != 1) {
            continue;
        }
        /* Keep a sorted list of the highest numbers seen so far. */
        if (cnt < SHOT_KEEP_MAX) {
            nums[cnt++] = n;
        } else {
            uint32_t *minp = &nums[0];
            for (int i = 1; i < cnt; i++) {
                if (nums[i] < *minp) {
                    minp = &nums[i];
                }
            }
            if (n > *minp) {
                *minp = n;
            }
        }
    }
    closedir(d);
    if (cnt < SHOT_KEEP_MAX) {
        return;   /* not enough to trim */
    }

    /* Delete any screenshot not in the kept set. */
    d = opendir(SD_SHOT_DIR);
    if (!d) {
        return;
    }
    while ((e = readdir(d)) != NULL) {
        unsigned n = 0;
        if (sscanf(e->d_name, "shot_%u.png", &n) != 1) {
            continue;
        }
        bool keep = false;
        for (int i = 0; i < cnt; i++) {
            if (nums[i] == n) {
                keep = true;
                break;
            }
        }
        if (!keep) {
            char path[128];
            char name[24];
            strncpy(name, e->d_name, sizeof(name) - 1);
            name[sizeof(name) - 1] = '\0';
            snprintf(path, sizeof(path), "%s/%s", SD_SHOT_DIR, name);
            remove(path);
        }
    }
    closedir(d);
}

esp_err_t sd_log_save_screenshot(const uint16_t *rgb565, int w, int h)
{
    if (!s_sd_ready) {
        return ESP_ERR_NOT_FOUND;
    }
    /* Persistent counter from the card contents (survives reboot). */
    uint32_t counter = shot_next_counter() + 1;
    char path[64];
    snprintf(path, sizeof(path), "%s/shot_%04lu.png", SD_SHOT_DIR, (unsigned long)counter);
    esp_err_t err = write_png(path, rgb565, w, h);
    if (err == ESP_OK) {
        trim_shots();
    }
    return err;
}
