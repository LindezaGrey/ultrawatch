/*
 * sd_log.c - SD card mount (on demand) + PNG screenshot capture.
 *
 * Uses the board's SPI2 bus (already initialized by twatch_board) for the SD
 * card (CS=21, FATFS). Mounted only while the watch is awake - see
 * sd_log_mount()/sd_log_unmount() and power_mgmt.c's sleep/wake handling.
 * Screenshots are encoded with libpng (managed component) and written to
 * /sdcard/shot/.
 */
#include "sd_log.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "png.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "spi2_power.h"

static const char *TAG = "sd_log";

#define SD_LOG_BASE      "/sdcard"
#define SD_LOG_DIR       "/sdcard/log"
#define SD_SHOT_DIR      "/sdcard/shot"
#define SD_CS_GPIO       21

#define SHOT_KEEP_MAX    10   /* retain only the newest screenshots */

static bool s_sd_ready;
static sdmmc_card_t *s_card;

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

/* Re-applies ALDO1's steady-state policy right now: up if a card is
 * seated, down otherwise. ALDO1 is registered with spi2_power as a
 * SPI2_POWER_SHARED rail (twatch_board.c) precisely because it is not
 * just the SD card's rail - SPI2 is shared with the SX1262 and the
 * ST25R3916, and a seated-but-unpowered card clamps the bus's MISO net
 * through its ESD protection diodes, corrupting every read on SPI2 while
 * this rail is down (see docs/nfc.md). A no-op, momentary bus hold is
 * enough to trigger the re-evaluation: spi2_power_release()'s last-holder
 * path always re-reads twatch_sd_card_seated() fresh rather than trusting
 * anything cached, which is also what closes the old "known gap" here - a
 * card inserted mid-session used to only be picked up at the next mount()
 * or sleep/wake cycle; now ANY spi2_power session ending anywhere (an NFC
 * open/close, a future LoRa session) re-evaluates it too. */
static void sd_rail_reconcile(void)
{
    spi2_power_hold(AXP2101_RAIL_MAX);
    spi2_power_release(AXP2101_RAIL_MAX);
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

    /* Give the card a clean power-on before every mount attempt: either a
     * true cold boot, or a remount after sd_log_unmount() cut ALDO1 for
     * sleep. A card left powered with its SD registers in an undefined
     * state (e.g. a soft reset / watchdog reboot that skipped
     * sd_log_unmount()) usually fails its first init with CRC/response
     * errors, so the retry loop below re-cycles the rail again if needed. */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) {
            /* Toggle the SD rail: off, settle, on, settle. */
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, false);
            vTaskDelay(pdMS_TO_TICKS(50));
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        err = esp_vfs_fat_sdspi_mount(SD_LOG_BASE, &host, &slot, &mount_cfg, &s_card);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "SD mount attempt %d failed: %s", attempt + 1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s", esp_err_to_name(err));
        s_sd_ready = false;
        sd_rail_reconcile();
        return err;
    }

    mkdir(SD_LOG_DIR, 0755);   /* parent dir for crash dumps + daily activity CSVs */
    mkdir(SD_SHOT_DIR, 0755);
    s_sd_ready = true;
    ESP_LOGI(TAG, "SD mounted: %u MB", (unsigned)(s_card->csd.capacity / 1024 / 1024));
    return ESP_OK;
}

esp_err_t sd_log_unmount(void)
{
    if (!s_card) {
        return ESP_OK;   /* already unmounted */
    }
    s_sd_ready = false;

    esp_err_t err = esp_vfs_fat_sdcard_unmount(SD_LOG_BASE, s_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD unmount: %s", esp_err_to_name(err));
    }
    s_card = NULL;

    /* Reconcile ALDO1 only after a clean unmount - yanking the rail out from
     * under a still-mounted card is what leaves it in the undefined state
     * that fails to remount with resp/CRC errors. See sd_rail_reconcile(). */
    sd_rail_reconcile();
    return err;
}

bool sd_log_available(void)
{
    return s_sd_ready;
}

esp_err_t sd_log_clear(void)
{
    if (!s_sd_ready) {
        return ESP_ERR_NOT_FOUND;
    }
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
