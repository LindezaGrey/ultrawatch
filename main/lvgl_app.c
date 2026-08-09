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
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_spiffs.h"
#include "esp_lv_adapter.h"
#include "lvgl.h"
#include "libs/freetype/lv_freetype.h"
#include "co5300.h"
#include "cst9217.h"

static const char *TAG = "lvgl_app";

/* Debug label showing the tapped coordinates (touch verification). */
static lv_obj_t *s_touch_label;

static void touch_event_cb(lv_event_t *e)
{
    lv_obj_t *label = lv_event_get_user_data(e);
    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    char text[32];
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        snprintf(text, sizeof(text), "TOUCH OFF");
    } else {
        snprintf(text, sizeof(text), "X=%03d Y=%03d", (int)p.x, (int)p.y);
    }
    lv_label_set_text(label, text);
}

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

static void lvgl_build_boot_screen(const lv_font_t *title_font, const lv_font_t *small_font)
{
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), 0);

    /* App name (high-DPI title). */
    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "UWatch");
    lv_obj_set_style_text_font(title, title_font, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -80);

    /* Board subtitle. */
    lv_obj_t *sub = lv_label_create(lv_screen_active());
    lv_label_set_text(sub, "LILYGO T-Watch Ultra");
    lv_obj_set_style_text_font(sub, small_font, 0);
    lv_obj_set_style_text_color(sub, lv_color_hex(0x888888), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, -20);

    /* Version + git commit hash for build recognition. */
    char text[64];
    lv_obj_t *ver = lv_label_create(lv_screen_active());
    snprintf(text, sizeof(text), "v%s", esp_app_get_description()->version);
    lv_label_set_text(ver, text);
    lv_obj_set_style_text_font(ver, small_font, 0);
    lv_obj_set_style_text_color(ver, lv_color_hex(0x666666), 0);
    lv_obj_align(ver, LV_ALIGN_CENTER, 0, 40);

    lv_obj_t *hash = lv_label_create(lv_screen_active());
    snprintf(text, sizeof(text), "git %s", UWATCH_GIT_HASH);
    lv_label_set_text(hash, text);
    lv_obj_set_style_text_font(hash, small_font, 0);
    lv_obj_set_style_text_color(hash, lv_color_hex(0x555555), 0);
    lv_obj_align(hash, LV_ALIGN_CENTER, 0, 80);

    /* Touch debug label (shows tapped coordinates). */
    s_touch_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_touch_label, "touch: -");
    lv_obj_set_style_text_font(s_touch_label, small_font, 0);
    lv_obj_set_style_text_color(s_touch_label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(s_touch_label, LV_ALIGN_CENTER, 0, 130);
    lv_obj_add_event_cb(lv_screen_active(), touch_event_cb, LV_EVENT_PRESSED, s_touch_label);
    lv_obj_add_event_cb(lv_screen_active(), touch_event_cb, LV_EVENT_RELEASED, s_touch_label);
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
        /* High-DPI vector fonts from SPIFFS (Roboto; ~315 PPI panel). */
        lv_font_t *title_font = lv_freetype_font_create("/assets/fonts/Roboto-Regular.ttf",
                                                        LV_FREETYPE_FONT_RENDER_MODE_BITMAP, 64,
                                                        LV_FREETYPE_FONT_STYLE_NORMAL);
        lv_font_t *small_font = lv_freetype_font_create("/assets/fonts/Roboto-Regular.ttf",
                                                        LV_FREETYPE_FONT_RENDER_MODE_BITMAP, 28,
                                                        LV_FREETYPE_FONT_STYLE_NORMAL);
        if (title_font && small_font) {
            lvgl_build_boot_screen(title_font, small_font);
        } else {
            ESP_LOGE(TAG, "freetype font create failed");
            lvgl_build_boot_screen(NULL, NULL);   /* fall back to default font */
        }
        esp_lv_adapter_unlock();
    }

    /* Touch input (CST9217). */
    esp_lcd_touch_handle_t tp = cst9217_get_handle();
    if (tp) {
        esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, tp);
        uint16_t nx = 0, ny = 0;
        if (cst9217_get_resolution(&nx, &ny) == ESP_OK && nx && ny) {
            touch_cfg.scale.x = (float)CO5300_RES_X / nx;
            touch_cfg.scale.y = (float)CO5300_RES_Y / ny;
            ESP_LOGI(TAG, "touch scale: %f x %f", touch_cfg.scale.x, touch_cfg.scale.y);
        }
        if (esp_lv_adapter_register_touch(&touch_cfg)) {
            ESP_LOGI(TAG, "touch registered");
        } else {
            ESP_LOGE(TAG, "touch registration failed");
        }
    } else {
        ESP_LOGW(TAG, "no CST9217 touch handle");
    }

    ESP_LOGI(TAG, "LVGL started (Roboto FreeType font)");
    return ESP_OK;
}
