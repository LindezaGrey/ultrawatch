/*
 * lvgl_app.c - LVGL UI on the CO5300 AMOLED via esp_lvgl_adapter.
 *
 * The adapter runs LVGL in its own FreeRTOS task (tick + locking included).
 * Panel specifics handled here:
 *   - RGB565_SWAPPED color format: the CO5300 samples big-endian RGB565, so
 *     LVGL renders big-endian buffers that the panel reads correctly.
 *   - Areas are rounded to even coordinates BEFORE rendering (via
 *     LV_EVENT_INVALIDATE_AREA), so the draw buffer always matches the
 *     flushed area and the panel never receives odd boundaries.
 *   - High-DPI fonts: Roboto-Regular.ttf lives in a SPIFFS partition mounted
 *     at /assets; LVGL's FreeType renders it at large sizes (~315 PPI panel).
 */
#include "lvgl_app.h"
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_lv_adapter.h"
#include "lvgl.h"
#include "libs/freetype/lv_freetype.h"
#include "co5300.h"

static const char *TAG = "lvgl_app";

/* Round invalidated areas to even coordinates (SH8601 requirement) BEFORE
 * LVGL renders, so the buffer content always matches the flushed area. */
static void area_rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}

static void lvgl_build_ui(const lv_font_t *font)
{
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), 0);

    /* High-DPI title. */
    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "UWatch");
    lv_obj_set_style_text_font(title, font, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -60);

    /* Smaller subtitle. */
    lv_obj_t *sub = lv_label_create(lv_screen_active());
    lv_label_set_text(sub, "LILYGO T-Watch Ultra");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x888888), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, 40);
}

static void mount_assets(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/assets",
        .partition_label = "assets",
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_OK) {
        size_t used = 0, total = 0;
        esp_spiffs_info(conf.partition_label, &total, &used);
        ESP_LOGI(TAG, "assets SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    } else {
        ESP_LOGW(TAG, "assets SPIFFS mount failed: %s", esp_err_to_name(ret));
    }
}

esp_err_t lvgl_app_start(void)
{
    mount_assets();

    const esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lv_adapter_init(&adapter_cfg), TAG, "adapter init");

    esp_lv_adapter_display_config_t display_cfg = ESP_LV_ADAPTER_DISPLAY_SPI_WITH_PSRAM_DEFAULT_CONFIG(
        co5300_get_panel(),
        co5300_get_panel_io(),
        CO5300_RES_X,
        CO5300_RES_Y,
        ESP_LV_ADAPTER_ROTATE_0);   /* rotation not supported for QSPI */
    /* Partial-render stripe height: keep flushes within the SPI max transfer
     * (410 x 48 x 2 = ~39 KB < 65 KB) and even for the SH8601 panel. */
    display_cfg.profile.buffer_height = 48;

    lv_display_t *disp = esp_lv_adapter_register_display(&display_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "register display failed");
        return ESP_FAIL;
    }

    /* Big-endian RGB565 for the CO5300 panel. */
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);

    /* Even-coordinate areas, rounded before rendering (no buffer mismatch). */
    lv_display_add_event_cb(disp, area_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), TAG, "adapter start");

    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        /* High-DPI vector font from SPIFFS (Roboto, 64 px for ~315 PPI). */
        lv_font_t *font = lv_freetype_font_create("/assets/fonts/Roboto-Regular.ttf",
                                                  LV_FREETYPE_FONT_RENDER_MODE_BITMAP, 64,
                                                  LV_FREETYPE_FONT_STYLE_NORMAL);
        if (font) {
            lvgl_build_ui(font);
        } else {
            ESP_LOGE(TAG, "lv_freetype_font_create failed");
            lvgl_build_ui(NULL);   /* fall back to default font */
        }
        esp_lv_adapter_unlock();
    }

    ESP_LOGI(TAG, "LVGL started (Roboto FreeType font)");
    return ESP_OK;
}
