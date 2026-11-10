/*
 * lvgl_app.c - LVGL UI on the CO5300 AMOLED via esp_lvgl_adapter.
 *
 * The adapter runs LVGL in its own FreeRTOS task (tick + locking included).
 *   - RGB565_SWAPPED: the CO5300 samples big-endian RGB565.
 *   - Even-coordinate areas rounded BEFORE rendering (SH8601 requirement).
 *   - High-DPI vector fonts (Cascadia Code) via LVGL FreeType from SPIFFS.
 *   - Boot screen -> watch face (time from the RTC-synced system clock).
 */
#include "lvgl_app.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_app_desc.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "esp_lv_adapter.h"
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "daily_log.h"
#include "co5300.h"
#include "bhi260ap.h"
#include "sd_log.h"
#include "pcf85063a.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "tracking.h"
#include "sensor_cache.h"
#include "m10q.h"
#include "power_mgmt.h"
#include "display_controller.h"
#include "touch_controller.h"
#include "cst9217.h"
#include "alarm.h"
#include "cd_timer.h"
#include "gpx_log.h"
#include "gps_reception.h"
#include "mesh_log.h"
#include "st25r3916.h"
#include "ndef.h"
#include "ble_debug.h"
#include "screens/screens.h"
#include "screens/navigation.h"

static const char *TAG = "lvgl_app";

/* Fonts (embedded Cascadia Code bitmaps, see cascadia_fonts.h). */
static const lv_font_t *s_font_time = &cascadia_72;   /* HH:MM:SS */
static const lv_font_t *s_font_small = &cascadia_22;  /* body text */

/* BHI260AP status screen now lives in main/screens/bhi_screen.c. */

/* GPS screen's own widgets/state now live in main/screens/gps_screen.c. */
static gps_reception_reason_t s_gps_off_reason;
static bool s_gps_seed_pending;
static double s_gps_seed_lat, s_gps_seed_lon;
static bool s_gps_enabled;                     /* persisted "GNSS on" choice */

/* Mesh screen + Node-Overview sub-screen: main/screens/mesh_screen.c. */

/* NFC screen (opened from Apps). One-shot scan on demand -
 * st25r3916_open() holds the shared SPI2 bus rails (spi2_power.h) for as
 * long as a session is open, so scanning is explicit (a button), not
 * automatic on screen entry like GPS's always-on acquire. */
typedef enum {
    NFC_SCAN_IDLE = 0,
    NFC_SCAN_SCANNING,
    NFC_SCAN_FOUND,
    NFC_SCAN_TIMEOUT,
    NFC_SCAN_ERROR,
} nfc_scan_state_t;
static volatile nfc_scan_state_t s_nfc_scan_state = NFC_SCAN_IDLE;
static volatile bool s_nfc_scan_req;      /* Start tap, consumed by nfc_ctrl_task */
static st25r3916_tag_t s_nfc_last_tag;    /* valid only when state == NFC_SCAN_FOUND */
static esp_err_t s_nfc_last_err;          /* valid only when state == NFC_SCAN_ERROR */
#define NFC_MAX_NDEF_RECORDS 4
static ndef_record_t s_nfc_last_ndef[NFC_MAX_NDEF_RECORDS];   /* valid only when state == NFC_SCAN_FOUND */
static size_t s_nfc_last_ndef_count;      /* 0 = not NDEF-formatted or no message - both normal */
static uint32_t s_nfc_scan_start_ms;
static TaskHandle_t s_nfc_ctrl_task;
#define NFC_SCAN_TIMEOUT_MS 8000   /* matches nfcpoll's own default window */

/* Alarms/Timers list + edit/timer sub-screens + ringing screen now live in
 * main/screens/alarm_screen.c and main/screens/ring_screen.c. */

/* Settings category list + its 6 sub-pages now live in
 * main/screens/settings_screen.c. */

/* GNSS control runs off the LVGL task (m10q_power blocks for seconds during
 * baud probing/config); the UI issues a request and a worker task applies it. */
#define GPS_CTRL_NONE    0
#define GPS_CTRL_ON      1
#define GPS_CTRL_OFF     2
/* One pending command is intentional: if the user changes the GPS switch
 * repeatedly while a slow power transition runs, only their latest choice
 * should be applied.  The lock makes taking-and-clearing that command atomic
 * with a concurrent UI update. */
static int s_gps_ctrl_req;
static portMUX_TYPE s_gps_ctrl_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_gps_ctrl_task;
#define LKP_REFRESH_INTERVAL_MS (60 * 1000)   /* NVS write throttle, not fix rate */
static uint32_t s_gps_lkp_saved_ms;    /* monotonic milliseconds */

/* Persisted "GNSS enabled" setting (NVS, namespace "gps", key "en"). */
#define GPS_NVS_NS       "gps"
#define GPS_NVS_KEY_EN   "en"

static bool gps_load_enabled(void)
{
    bool en = false;   /* default: GNSS off at startup */
    nvs_handle_t h;
    if (nvs_open(GPS_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 1;
        nvs_get_u8(h, GPS_NVS_KEY_EN, &v);
        nvs_close(h);
        en = (v != 0);
    }
    return en;
}

static void gps_save_enabled(bool on)
{
    nvs_handle_t h;
    if (nvs_open(GPS_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, GPS_NVS_KEY_EN, on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static lv_indev_t *s_touch_indev;
static void nfc_start_btn_cb(lv_event_t *e);
static void nfc_ctrl_task(void *arg);
static void gps_ctrl_task(void *arg);
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

/* screen_new() is now declared inline in screens/screens.h (shared).
 *
 * The status bar (build_status_bar()/build_status_bar_big()/
 * update_status_bar()/status_bar_set_hidden(), screens/status_bar.h) is
 * now shown only on the watch face (screens/watch_face.c) - per explicit
 * feedback, it used to repeat (at a smaller size) on every other
 * nav-ring screen too, which read as visual noise rather than useful
 * status. */

/* Red-only night-mode transform. LVGL renders RGB565_SWAPPED (big-endian on
 * the panel): each pixel is 2 bytes, byte0 = MSB = RRRRR GGG, byte1 = GGG BBBBB.
 * Keeping only the red channel zeroes green/blue. Applied in-place; the blit is
 * synchronous, so the buffer is safe to mutate before esp_lcd_panel_draw_bitmap. */
/* See lvgl_app.h's header comment on these two getters - the "did a real
 * flush happen" diagnostic for the recurring white-screen-after-wake
 * reports. Updated unconditionally at the top of every call, success or
 * failure, so a stuck count/growing age proves the flush pipeline itself
 * never ran, not just that this one band failed. */
static volatile uint32_t s_flush_count;
static volatile int64_t s_last_flush_us = -1;

uint32_t lvgl_flush_count_get(void)
{
    return s_flush_count;
}

uint32_t lvgl_flush_age_ms_get(void)
{
    if (s_last_flush_us < 0) {
        return UINT32_MAX;
    }
    int64_t age_us = esp_timer_get_time() - s_last_flush_us;
    return (age_us < 0) ? 0 : (uint32_t)(age_us / 1000);
}

static esp_err_t night_mode_draw_bitmap(lv_display_t *disp, esp_lcd_panel_handle_t panel,
                                        int x_start, int y_start, int x_end, int y_end,
                                        const void *color_map, void *user_ctx)
{
    (void)disp;
    (void)user_ctx;
    s_flush_count++;
    s_last_flush_us = esp_timer_get_time();
    /* Ultra-Sparmodus's explicit-wake display (docs/application.md section
     * 10.3) reuses this same red-only transform as night mode - both want
     * "red instead of white", just for different reasons and without
     * night mode's other side effects (touch stays enabled). */
    if (power_mgmt_is_night_mode() || power_mgmt_get_sparmodus_active()) {
        uint8_t *buf = (uint8_t *)color_map;
        size_t n = (size_t)(x_end - x_start) * (y_end - y_start);
        for (size_t i = 0; i < n; i++) {
            buf[i * 2]     &= 0xF8;   /* keep 5-bit red */
            buf[i * 2 + 1]  = 0x00;   /* drop green/blue */
        }
    }
    /* A burst of full-screen redraws (multiple 48-row DMA bands queued in
     * quick succession) can transiently exhaust the ~113 KB internal
     * DMA-capable RAM pool the draw buffers live in (see
     * esp_lv_adapter_display_config_t's buffer_height comment at the
     * registration site) - esp_lcd_panel_draw_bitmap() then fails with
     * ESP_ERR_NO_MEM for that one band. Previously this error was just
     * passed straight back up with nothing retrying it: LVGL still marks
     * the frame flushed and moves on, leaving that band's GRAM content
     * whatever it was before (commonly white/garbage on a fresh boot or a
     * just-woken panel) - a real live report of a "white screen that
     * recovered by itself" on the next real redraw is consistent with
     * exactly this. A short retry loop gives the DMA pool a moment to
     * free up (earlier bands in the same flush cycle complete and release
     * their buffers) rather than silently dropping the band. */
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end, y_end, color_map);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "draw_bitmap attempt %d failed: %s", attempt + 1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (err == ESP_OK) {
        display_controller_note_draw_complete();
    }
    return err;
}

/* Full-screen redraw (e.g. after waking from sleep, when co5300_blank() left
 * the GRAM black and the adapter's SPI path does not auto-refresh on
 * resume). Audited live alongside the DMA-retry fix in
 * night_mode_draw_bitmap() above (a real report of a "white screen that
 * recovered by itself"): every full-screen invalidate in this file funnels
 * through this one function now (night_mode_changed() below used to
 * duplicate the same three lines instead of calling it) - its callers
 * (pm_apply_night_mode() via night_mode_changed(), and power_mgmt.c's
 * wake-from-sleep/debug_cmds.c's redraw command directly) are all
 * edge-triggered/one-shot, not fired repeatedly in a burst, so no separate
 * throttling was needed here beyond the retry already in the draw callback. */
void lvgl_force_redraw(void)
{
    esp_err_t lock_err = esp_lv_adapter_lock(-1);
    /* Diagnostic for the recurring white-screen-after-wake reports: a
     * live incident showed the wake path's own lvgl_force_redraw() +
     * co5300_display_on() call reporting no error anywhere, yet the
     * screen stayed white - meaning if this ever fails or invalidates a
     * NULL/unexpected screen, it currently does so completely silently.
     * ESP_LOGW only (not every call - lvgl_force_redraw() also runs on
     * ordinary night-mode toggles, not just wake), so this stays quiet
     * on the success path most callers hit. */
    if (lock_err != ESP_OK) {
        ESP_LOGW(TAG, "lvgl_force_redraw: lock failed: %s", esp_err_to_name(lock_err));
        return;
    }
    lv_obj_t *scr = lv_screen_active();
    if (!scr) {
        ESP_LOGW(TAG, "lvgl_force_redraw: lv_screen_active() is NULL");
    } else {
        lv_obj_invalidate(scr);
    }
    esp_lv_adapter_unlock();
}

/* On a night-mode change, invalidate the whole screen so every pixel is
 * redrawn through the red-only transform (content flushed before the state
 * flip would otherwise keep its old colors). */
static void night_mode_changed(bool night)
{
    (void)night;
    lvgl_force_redraw();
}

static void lvgl_build_boot_screen(void)
{
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), 0);
    lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "UWatch");
    lv_obj_set_style_text_font(title, s_font_time, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -70);

    lv_obj_t *sub = lv_label_create(lv_screen_active());
    lv_label_set_text(sub, "LILYGO T-Watch Ultra");
    lv_obj_set_style_text_font(sub, s_font_small, 0);
    lv_obj_set_style_text_color(sub, lv_color_hex(0x888888), 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, -10);

    char text[64];
    lv_obj_t *ver = lv_label_create(lv_screen_active());
    snprintf(text, sizeof(text), "v%s", esp_app_get_description()->version);
    lv_label_set_text(ver, text);
    lv_obj_set_style_text_font(ver, s_font_small, 0);
    lv_obj_set_style_text_color(ver, lv_color_hex(0x666666), 0);
    lv_obj_align(ver, LV_ALIGN_CENTER, 0, 40);

    lv_obj_t *hash = lv_label_create(lv_screen_active());
    snprintf(text, sizeof(text), "git %s", UWATCH_GIT_HASH);
    lv_label_set_text(hash, text);
    lv_obj_set_style_text_font(hash, s_font_small, 0);
    lv_obj_set_style_text_color(hash, lv_color_hex(0x555555), 0);
    lv_obj_align(hash, LV_ALIGN_CENTER, 0, 80);
}

/* Show the boot screen for a few seconds, then the watch face. */
static void boot_to_watch_face(lv_timer_t *timer)
{
    lv_timer_delete(timer);
    lv_obj_clean(lv_screen_active());
    lvgl_build_watch_face();
    navigation_note_activity();
}

/* BHI260AP status screen: main/screens/bhi_screen.c (shared with the
 * sim). */

void lvgl_show_bhi_screen(void)
{
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        ESP_LOGW(TAG, "show bhi: LVGL lock timeout");
        return;
    }
    navigation_open(NAV_BHI);
    esp_lv_adapter_unlock();
}

/* GPS screen (skyplot + fix info): main/screens/gps_screen.c (shared with
 * the sim). gps_ctrl_task (below) is the real background power-transition
 * worker - firmware-only, no sim equivalent - and talks to the screen via
 * gps_screen_set_powered()/gps_screen_is_powered() (screens/screens.h)
 * instead of touching screen-owned state directly. */


/* Mesh screen + Node-Overview sub-screen: main/screens/mesh_screen.c
 * (shared with the sim). lvgl_mesh_screen_show() below is the only
 * ESP-only piece (needs esp_lv_adapter_lock()/request_wake()) and calls
 * into lvgl_build_mesh_screen()/mesh_screen_update() there. */

/* Runs on the mesh_log background task (not the LVGL task), so the LVGL lock
 * is required around screen changes - same pattern as alarm_ring_cb(). Jumps
 * to the Mesh screen from wherever the UI currently is and wakes the display,
 * since the SX1262 listens continuously regardless of which screen is open
 * or whether the watch is asleep. */
void lvgl_mesh_screen_show(void)
{
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        ESP_LOGW(TAG, "mesh screen show: LVGL lock timeout");
        return;
    }
    navigation_open(NAV_MESH);
    esp_lv_adapter_request_wake();
    esp_lv_adapter_unlock();
}

/* Settings category list + its 6 sub-pages: main/screens/settings_screen.c
 * (shared with the sim). lvgl_show_settings_disp() there is called
 * from the shared navigation long-press handler (tap-and-hold shortcut,
 * docs/application.md section 9.3). */

/* ---- NFC screen (opened from Apps) ---- */

static void nfc_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_nfc_status_label) {
        return;
    }
    if (lv_screen_active() != s_nfc_screen) {
        return;
    }

    char buf[96];
    nfc_scan_state_t st = s_nfc_scan_state;

    switch (st) {
    case NFC_SCAN_SCANNING: {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint32_t elapsed = (now - s_nfc_scan_start_ms) / 1000;
        snprintf(buf, sizeof(buf), "Scanning... (%lus)", (unsigned long)elapsed);
        lv_label_set_text(s_nfc_status_label, buf);
        lv_label_set_text(s_nfc_uid_label, "Hold tag near the back of the watch");
        lv_obj_set_style_text_color(s_nfc_uid_label, lv_color_hex(0x9E9E9E), 0);
        /* Keep the watch awake for the scan window - otherwise auto-sleep
         * could cut rails st25r3916_open() is relying on mid-scan. */
        power_mgmt_report_activity();
        break;
    }
    case NFC_SCAN_FOUND: {
        lv_label_set_text(s_nfc_status_label, "Tag found");
        char hex[3 * 10 + 1] = { 0 };
        for (uint8_t i = 0; i < s_nfc_last_tag.uid_len && i < sizeof(s_nfc_last_tag.uid); i++) {
            snprintf(hex + i * 3, 4, "%02X ", s_nfc_last_tag.uid[i]);
        }
        /* UID line plus decoded NDEF content (if any) on its own lines below -
         * LV_LABEL_LONG_WRAP handles an explicit "
" fine (unlike
         * LONG_DOT, which hangs on that combination - see mesh_screen_update()'s
         * own doc comment on that bug). s_nfc_last_ndef_count == 0 covers
         * both "not NDEF-formatted" and "formatted, empty message" - both
         * normal outcomes for a tag, not an error. */
        char found_buf[512];
        int off = snprintf(found_buf, sizeof(found_buf), "UID: %s(%u bytes)",
                           hex, (unsigned)s_nfc_last_tag.uid_len);
        if (s_nfc_last_ndef_count == 0) {
            snprintf(found_buf + off, sizeof(found_buf) - off, "\nNo NDEF data");
        } else {
            for (size_t i = 0; i < s_nfc_last_ndef_count && off < (int)sizeof(found_buf); i++) {
                const char *kind = (s_nfc_last_ndef[i].kind == NDEF_TEXT) ? "Text" :
                                    (s_nfc_last_ndef[i].kind == NDEF_URI)  ? "URI"  : "Other";
                off += snprintf(found_buf + off, sizeof(found_buf) - off,
                                "\n%s: %s", kind, s_nfc_last_ndef[i].text);
            }
        }
        lv_label_set_text(s_nfc_uid_label, found_buf);
        lv_obj_set_style_text_color(s_nfc_uid_label, lv_color_hex(0x3DD68A), 0);
        break;
    }
    case NFC_SCAN_TIMEOUT:
        lv_label_set_text(s_nfc_status_label, "No tag found");
        lv_label_set_text(s_nfc_uid_label, "Timed out - tap Start to try again");
        lv_obj_set_style_text_color(s_nfc_uid_label, lv_color_hex(0x9E9E9E), 0);
        break;
    case NFC_SCAN_ERROR:
        lv_label_set_text(s_nfc_status_label, "Reader error");
        snprintf(buf, sizeof(buf), "%s", esp_err_to_name(s_nfc_last_err));
        lv_label_set_text(s_nfc_uid_label, buf);
        lv_obj_set_style_text_color(s_nfc_uid_label, lv_color_hex(0xE57373), 0);
        break;
    case NFC_SCAN_IDLE:
    default:
        lv_label_set_text(s_nfc_status_label, "Ready");
        lv_label_set_text(s_nfc_uid_label, "Tap Start, then hold a tag near the watch");
        lv_obj_set_style_text_color(s_nfc_uid_label, lv_color_hex(0x9E9E9E), 0);
        break;
    }

    if (s_nfc_start_btn) {
        bool scanning = (st == NFC_SCAN_SCANNING);
        if (scanning) {
            lv_obj_add_state(s_nfc_start_btn, LV_STATE_DISABLED);
        } else {
            lv_obj_clear_state(s_nfc_start_btn, LV_STATE_DISABLED);
        }
        lv_obj_t *bl = lv_obj_get_child(s_nfc_start_btn, 0);
        if (bl) {
            lv_label_set_text(bl, scanning ? "Scanning" : "Start");
        }
    }
}

/* NFC screen Start button: only issues a request if idle - nfc_ctrl_task
 * (not this, the LVGL task) does st25r3916_open()/try()/close(), all of
 * which block for real time and must not run here. */
static void nfc_start_btn_cb(lv_event_t *e)
{
    (void)e;
    if (s_nfc_scan_state == NFC_SCAN_SCANNING) {
        return;
    }
    s_nfc_scan_req = true;
}

/* Background task owning every st25r3916 call: st25r3916_open() alone can
 * take on the order of 100ms (rail settle + oscillator start + regulator
 * calibration), and a full scan attempt loops st25r3916_try() for up to
 * NFC_SCAN_TIMEOUT_MS - none of that belongs on the LVGL task. One-shot per
 * request, per the chosen UX: stops on the first tag read OR the timeout,
 * whichever comes first, rather than looping indefinitely like nfcpoll's own
 * console command does. */
static void nfc_ctrl_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_nfc_scan_req) {
            s_nfc_scan_req = false;
            s_nfc_scan_state = NFC_SCAN_SCANNING;
            s_nfc_scan_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

            esp_err_t err = st25r3916_open();
            if (err != ESP_OK) {
                st25r3916_close();
                s_nfc_last_err = err;
                s_nfc_scan_state = NFC_SCAN_ERROR;
            } else {
                st25r3916_tag_t tag = { 0 };
                esp_err_t try_err = ESP_ERR_NOT_FOUND;
                TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(NFC_SCAN_TIMEOUT_MS);
                while (xTaskGetTickCount() < deadline) {
                    try_err = st25r3916_try(&tag, 300);
                    if (try_err != ESP_ERR_NOT_FOUND) {
                        break;
                    }
                }

                /* NDEF read while the tag is still selected, i.e. before
                 * st25r3916_close() - same sequence as nfcpoll's console
                 * command. s_nfc_last_ndef_count staying 0 covers both "not
                 * NDEF-formatted" and "formatted, empty message" - both
                 * normal outcomes for a tag, shown as "no NDEF data" rather
                 * than as an error. */
                s_nfc_last_ndef_count = 0;
                if (try_err == ESP_OK) {
                    uint8_t ndef_buf[256];
                    size_t ndef_len = 0;
                    if (st25r3916_read_type2(ndef_buf, sizeof(ndef_buf), &ndef_len, 500) == ESP_OK) {
                        s_nfc_last_ndef_count = ndef_parse(ndef_buf, ndef_len,
                                                           s_nfc_last_ndef, NFC_MAX_NDEF_RECORDS);
                    }
                }

                st25r3916_close();
                if (try_err == ESP_OK) {
                    s_nfc_last_tag = tag;
                    s_nfc_scan_state = NFC_SCAN_FOUND;
                } else if (try_err == ESP_ERR_NOT_FOUND) {
                    s_nfc_scan_state = NFC_SCAN_TIMEOUT;
                } else {
                    s_nfc_last_err = try_err;
                    s_nfc_scan_state = NFC_SCAN_ERROR;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void lvgl_build_nfc_screen(void)
{
    nfc_screen_create(nfc_start_btn_cb);
    nfc_screen_update(NULL);
    lv_timer_create(nfc_screen_update, 500, NULL);
    if (s_nfc_ctrl_task == NULL) {
        if (xTaskCreate(nfc_ctrl_task, "nfc_ctrl", 4096, NULL,
                        ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY, &s_nfc_ctrl_task) != pdPASS) {
            s_nfc_ctrl_task = NULL;
            ESP_LOGE(TAG, "NFC control task allocation failed");
        }
    }
}

/* Worker task that actually powers the GNSS receiver. m10q_power() blocks for
 * up to a few seconds (receiver startup + configuration), so it must not run
 * on the LVGL task or the UI freezes. */
static void gps_ctrl_task(void *arg)
{
    (void)arg;
    gps_reception_t reception = { 0 };
#if TRACKING_ENABLED
    bool tracking_seeded = false;
#endif
    for (;;) {
        taskENTER_CRITICAL(&s_gps_ctrl_mux);
        int req = s_gps_ctrl_req;
        bool seed = s_gps_seed_pending;
        double seed_lat = s_gps_seed_lat, seed_lon = s_gps_seed_lon;
        s_gps_seed_pending = false;
        s_gps_ctrl_req = GPS_CTRL_NONE;
        taskEXIT_CRITICAL(&s_gps_ctrl_mux);
        if (req != GPS_CTRL_NONE) {
            bool on = req == GPS_CTRL_ON;
            esp_err_t err = m10q_power(on);
            gps_screen_set_powered(m10q_get_state() != M10Q_STATE_OFF);
            if (err == ESP_OK && on) {
                gps_reception_start(&reception, (uint32_t)(esp_timer_get_time() / 1000));
                ESP_LOGI(TAG, "GNSS on: 60s poor reception, 180s maximum acquisition");
            } else if (err != ESP_OK) {
                ESP_LOGE(TAG, "GNSS power transition failed: %s", esp_err_to_name(err));
                taskENTER_CRITICAL(&s_gps_ctrl_mux);
                if (s_gps_ctrl_req == GPS_CTRL_NONE) s_gps_enabled = false;
                taskEXIT_CRITICAL(&s_gps_ctrl_mux);
            }
            /* This worker serializes NVS writes as well as power transitions.
             * A newer queued UI request is applied on the next iteration. */
            gps_save_enabled(lvgl_gps_enabled());
        }

        if (seed && gps_screen_is_powered()) m10q_seed_position(seed_lat, seed_lon);
        if (gps_screen_is_powered()) {
#if TRACKING_ENABLED
            if (!tracking_fix_due()) tracking_seeded = false;
            if (tracking_fix_due() && !tracking_seeded) {
                double lat, lon;
                if (tracking_get_estimated_position(&lat, &lon)) m10q_seed_position(lat, lon);
                tracking_seeded = true;
            }
#endif
            m10q_fix_t fix = { 0 };
            m10q_get_fix(&fix);
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            gps_reception_reason_t reason = gps_reception_update(&reception, now, &fix);
            if (reason != GPS_RECEPTION_OK) {
                /* Do not replace a newer explicit on/off request with an
                 * automatic shutdown. Claim this transition under the same lock. */
                taskENTER_CRITICAL(&s_gps_ctrl_mux);
                bool stop = s_gps_ctrl_req == GPS_CTRL_NONE;
                if (stop) {
                    s_gps_enabled = false;
                    s_gps_off_reason = reason;
                }
                taskEXIT_CRITICAL(&s_gps_ctrl_mux);
                if (stop) {
                    ESP_LOGW(TAG, "GNSS auto-off: %s", lvgl_gps_off_reason());
                    esp_err_t err = m10q_power(false);
                    gps_screen_set_powered(m10q_get_state() != M10Q_STATE_OFF);
                    if (err != ESP_OK) ESP_LOGE(TAG, "GNSS auto-off failed: %s", esp_err_to_name(err));
                    gps_save_enabled(lvgl_gps_enabled());
#if TRACKING_ENABLED
                    tracking_fix_clear();
#endif
                }
            } else if (fix.valid && fix.fix_3d && now - fix.updated_ms < 3000) {
#if TRACKING_ENABLED
                if (tracking_fix_due()) tracking_on_fix(fix.lat, fix.lon);
#endif
                if (now - s_gps_lkp_saved_ms >= LKP_REFRESH_INTERVAL_MS) {
                    m10q_update_last_position(fix.lat, fix.lon);
                    s_gps_lkp_saved_ms = now;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* An explicit check starts a fresh acquisition window, like the UI switch. */
void lvgl_gps_refresh(void)
{
    lvgl_gps_set_enabled(true);
}

bool lvgl_gps_enabled(void)
{
    taskENTER_CRITICAL(&s_gps_ctrl_mux);
    bool enabled = s_gps_enabled;
    taskEXIT_CRITICAL(&s_gps_ctrl_mux);
    return enabled;
}

bool lvgl_gps_timed_out(void)
{
    taskENTER_CRITICAL(&s_gps_ctrl_mux);
    bool timed_out = s_gps_off_reason != GPS_RECEPTION_OK;
    taskEXIT_CRITICAL(&s_gps_ctrl_mux);
    return timed_out;
}

const char *lvgl_gps_off_reason(void)
{
    taskENTER_CRITICAL(&s_gps_ctrl_mux);
    gps_reception_reason_t reason = s_gps_off_reason;
    taskEXIT_CRITICAL(&s_gps_ctrl_mux);
    switch (reason) {
    case GPS_POOR_SIGNAL: return "Poor signal - GPS off";
    case GPS_NO_TELEMETRY: return "No data - GPS off";
    case GPS_NO_FIX: return "No fix - GPS off";
    default: return "GNSS off";
    }
}

void lvgl_gps_seed_position(double lat, double lon)
{
    if (!isfinite(lat) || !isfinite(lon) || fabs(lat) > 90 || fabs(lon) > 180) return;
    taskENTER_CRITICAL(&s_gps_ctrl_mux);
    s_gps_seed_lat = lat; s_gps_seed_lon = lon;
    s_gps_seed_pending = true;
    taskEXIT_CRITICAL(&s_gps_ctrl_mux);
}

void lvgl_gps_set_enabled(bool on)
{
    taskENTER_CRITICAL(&s_gps_ctrl_mux);
    s_gps_enabled = on;
    s_gps_off_reason = GPS_RECEPTION_OK;
    s_gps_ctrl_req = on ? GPS_CTRL_ON : GPS_CTRL_OFF;
    taskEXIT_CRITICAL(&s_gps_ctrl_mux);
}

/* Start/stop a step-gated tracking session. The display keeps working normally
 * (no blanking); the watch face shows a red dot while active. Called from the
 * GPS screen buttons or the console. */
void lvgl_tracking_start(void)
{
    if (tracking_start() != ESP_OK) {
        ESP_LOGE(TAG, "tracking start failed");
        return;
    }
    ESP_LOGI(TAG, "tracking started (step-gated GNSS, display may sleep)");
}

void lvgl_tracking_stop(void)
{
    if (tracking_stop() != ESP_OK) {
        ESP_LOGE(TAG, "tracking stop failed");
        return;
    }
    ESP_LOGI(TAG, "tracking stopped");
}

/* Alarms/Timers list + create/edit + timer-start sub-screens:
 * main/screens/alarm_screen.c (shared with the sim). Ringing screen:
 * main/screens/ring_screen.c. alarm_ring_cb() below (the real ring
 * start/stop callback) stays firmware-only. */

/* Ring start/stop callback (runs on the alarm ring task, so the LVGL lock is
 * required around screen changes). source distinguishes an alarm-sourced
 * ring (show its configured time, offer Snooze) from a timer-sourced one
 * (no configured time to show, no snooze concept - see alarm.h). */
static void alarm_ring_cb(bool ringing, alarm_ring_source_t source)
{
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        ESP_LOGW(TAG, "alarm_ring_cb: LVGL lock timeout");
        return;
    }
    if (ringing) {
        if (lv_screen_active() != s_ring_screen) {
            if (!s_ring_screen) {
                lvgl_build_ring_screen();
            }
            if (source == ALARM_RING_SOURCE_TIMER) {
                lv_label_set_text(s_ring_title_label, "TIMER");
                /* "Time's up" overflows cascadia_72's width at this font
                 * size (confirmed via the sim's screenshot capture - a
                 * pre-existing bug, not something the high-DPI pass
                 * introduced); "Done" fits comfortably. */
                lv_label_set_text(s_ring_time_label, "Done");
                lv_obj_add_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_label_set_text(s_ring_title_label, "ALARM");
                /* Stamp the time of whichever entry triggered this ring. */
                int idx = alarm_get_ringing_index();
                alarm_entry_t list[ALARM_MAX_COUNT];
                alarm_get_all(list, ALARM_MAX_COUNT);
                char buf[8] = "--:--";
                if (idx >= 0 && idx < ALARM_MAX_COUNT && list[idx].in_use) {
                    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)list[idx].hour,
                             (unsigned)list[idx].min);
                }
                lv_label_set_text(s_ring_time_label, buf);
                lv_obj_clear_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
            }
            lv_scr_load(s_ring_screen);
            esp_lv_adapter_request_wake();
        }
    } else {
        /* Ring finished: back to the watch face. */
        if (lv_screen_active() == s_ring_screen) {
            lv_scr_load(s_watch_screen);
            watch_face_update(NULL);
        }
    }
    esp_lv_adapter_unlock();
}

static uint32_t navigation_clock_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Sensor task: bring up the BHI260AP (RAM firmware upload + boot) once the
 * assets partition is mounted, then poll the FIFO to stream sensor events.
 *
 * The sensor rail (ALDO4) is power-cycled by auto-sleep, which erases the
 * chip's RAM firmware. After wake the chip answers FIFO reads with empty data
 * (bootloader mode), so FIFO errors don't reliably appear. Instead, detect a
 * dead stream by data staleness: accel samples at 12.5 Hz when alive, so a
 * stale age (> 5 s) means the chip needs re-initialization. */
#define BHI_STALE_MS 5000

/* Consecutive failed init attempts before this cycle's re-init back-off is
 * applied (see bhi260_task's retry_delay_ms below) - reset to 0 on any
 * successful init. Uncapped in principle but the delay itself is capped, so
 * this just needs to not overflow across a very long-wedged session. */
#define BHI_REINIT_BACKOFF_MAX_MS 30000

/* Soft recovery only re-applies the virtual-sensor enables, and each attempt
 * resets the driver's staleness clock, so this many consecutive stale periods
 * is roughly BHI_SOFT_RECOVERY_LIMIT * BHI_STALE_MS of a dead stream before
 * the firmware image itself is assumed gone. Without the escalation the soft
 * path ran 880 times in a row on 2026-10-11 and the step counter stayed frozen
 * for two days. */
#define BHI_SOFT_RECOVERY_LIMIT 3

static void bhi260_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(50));
    bool initialized = (bhi260ap_init(twatch_imu_dev) == ESP_OK);
    uint32_t fail_streak = 0;
    uint32_t soft_streak = 0;
    uint32_t retry_delay_ms = 200;
    if (!initialized) {
        ESP_LOGW(TAG, "BHI260AP init failed; retrying");
        fail_streak = 1;
    }
    for (;;) {
        /* A wake-time resume can time out behind an in-flight FIFO read. Retry
         * here rather than leaving the sensor task parked in AP-suspend. */
        if (bhi260ap_resume_pending()) {
            esp_err_t resume_err = bhi260ap_ap_resume();
            if (resume_err != ESP_OK) {
                ESP_LOGW(TAG, "BHI260AP resume retry failed: %s", esp_err_to_name(resume_err));
            }
        }
        /* While still suspended, keep polling and stale recovery paused so the
         * wake-up gesture stream stays armed. */
        if (bhi260ap_is_suspended()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        retry_delay_ms = 200;
        if (initialized) {
            if (bhi260ap_process_fifo() != ESP_OK) {
                ESP_LOGW(TAG, "BHI260AP FIFO read failed");
            }
            /* Consume gesture flags so they don't stick */
            bool wg = false, gg = false, pg = false;
            bhi260ap_consume_gestures(NULL, &wg, &gg, &pg, NULL);
            if (wg) ESP_LOGI(TAG, "Wake gesture consumed");
            if (gg) ESP_LOGI(TAG, "Glance gesture consumed");
            if (pg) ESP_LOGI(TAG, "Pickup gesture consumed");
            if (bhi260ap_get_data_age_ms() > BHI_STALE_MS) {
                /* Staleness alone doesn't prove the chip is dead - it could
                 * just as easily mean a dropped virtual-sensor config (or
                 * transient I2C contention that has since cleared). Ping
                 * first: a live chip gets a cheap soft recovery (re-apply
                 * the sensor enables) instead of a full deinit+reinit, which
                 * re-uploads the whole RAM firmware over I2C - a multi-
                 * hundred-ms to multi-second burst that hogs the bus every
                 * other I2C device (PMU, RTC, the xl9555 GPIO expander
                 * gating DISP_PWR/TOUCH_RST) shares, and would otherwise
                 * fire on every transient stall instead of just real ones. */
                if (bhi260ap_ping()) {
                    if (++soft_streak >= BHI_SOFT_RECOVERY_LIMIT) {
                        /* The enables are not the problem - escalate. */
                        ESP_LOGW(TAG, "BHI260AP soft recovery did not restore data "
                                      "(%lu tries); re-initializing",
                                 (unsigned long)soft_streak);
                        bhi260ap_deinit();
                        initialized = false;
                        fail_streak = 0;
                        soft_streak = 0;
                        /* Keep a wedged chip from re-uploading its firmware on
                         * every pass once escalation is in play. */
                        retry_delay_ms = BHI_REINIT_BACKOFF_MAX_MS;
                    } else {
                        ESP_LOGW(TAG, "BHI260AP data stale but chip responsive; soft recovery");
                        bhi260ap_reenable_sensors();
                    }
                } else {
                    ESP_LOGW(TAG, "BHI260AP unresponsive, re-initializing");
                    bhi260ap_deinit();
                    initialized = false;
                    fail_streak = 0;   /* this is a fresh re-init attempt, not a repeat failure yet */
                    soft_streak = 0;
                }
            }
        } else {
            if (bhi260ap_init(twatch_imu_dev) == ESP_OK) {
                initialized = true;
                fail_streak = 0;
                soft_streak = 0;
            } else {
                /* Back off exponentially on repeated failures instead of
                 * hammering the bus with a full boot handshake every 200ms -
                 * a genuinely wedged/absent chip would otherwise generate
                 * that traffic forever. */
                fail_streak++;
                retry_delay_ms = 200u << (fail_streak > 7 ? 7 : fail_streak);
                if (retry_delay_ms > BHI_REINIT_BACKOFF_MAX_MS) {
                    retry_delay_ms = BHI_REINIT_BACKOFF_MAX_MS;
                }
                ESP_LOGW(TAG, "BHI260AP init failed (%lu in a row); retrying in %lu ms",
                         (unsigned long)fail_streak, (unsigned long)retry_delay_ms);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
    }
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

const char *uwatch_firmware_version(void)
{
    return esp_app_get_description()->version;
}

esp_err_t lvgl_app_start(void)
{
    mount_assets();

    /* Load live inactivity settings and brightness before starting the UI. */
    power_mgmt_load_config();

    /* A missing touch controller must not prevent the watch face from
     * starting; this matches the former board-startup behaviour, which
     * reported the driver failure but continued without touch input. */
    esp_err_t touch_err = touch_controller_init();
    if (touch_err != ESP_OK) {
        ESP_LOGE(TAG, "touch controller init failed: %s", esp_err_to_name(touch_err));
    }

    ESP_RETURN_ON_ERROR(display_controller_init(), TAG, "display controller init");
    display_controller_set_brightness(power_mgmt_get_brightness());

    const esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    esp_lv_adapter_config_t adapter_cfg_mut = adapter_cfg;
    /* Auto light sleep: pause LVGL after idle, then tickless light sleep. */
    adapter_cfg_mut.auto_sleep.enable = true;
    adapter_cfg_mut.auto_sleep.mode = ESP_LV_ADAPTER_AUTO_SLEEP_MODE_PAUSE;
    /* power_mgmt owns the live timeout; the adapter supplies the final idle gate. */
    adapter_cfg_mut.auto_sleep.idle_timeout_ms = 250;
    adapter_cfg_mut.auto_sleep.callbacks.on_enter_sleep = power_mgmt_enter_sleep;
    adapter_cfg_mut.auto_sleep.callbacks.on_exit_sleep = power_mgmt_exit_sleep;
    ESP_RETURN_ON_ERROR(esp_lv_adapter_init(&adapter_cfg_mut), TAG, "adapter init");
    power_mgmt_report_activity();

    /* Draw buffers in internal DMA-capable RAM: PSRAM buffers would need a
     * temp internal-DMA copy per band flush, and rapid redraws exhaust the
     * ~113 KB internal DMA pool (ESP_ERR_NO_MEM -> corrupted screen, now
     * also retried a few times in night_mode_draw_bitmap() rather than
     * just dropped - see that function). Shrunk from 48 rows (38 KB/band)
     * to 32 rows (~26 KB/band) after a live "white screen, recovered by
     * itself" report. 40-row bands keep three in-flight DMA buffers below
     * the ~113 KB pool while reducing a full-screen redraw from 16 transfers
     * (at 32 rows) to 13; 48 rows previously left too little headroom during
     * redraw bursts. */
    esp_lv_adapter_display_config_t display_cfg = ESP_LV_ADAPTER_DISPLAY_SPI_WITHOUT_PSRAM_DEFAULT_CONFIG(
        co5300_get_panel(),
        co5300_get_panel_io(),
        CO5300_RES_X,
        CO5300_RES_Y,
        ESP_LV_ADAPTER_ROTATE_0);   /* rotation not supported for QSPI */
    display_cfg.profile.buffer_height = 40;   /* 13 bands/frame; full frame exceeds SPI DMA max */

    lv_display_t *disp = esp_lv_adapter_register_display(&display_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "register display failed");
        return ESP_FAIL;
    }

    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    lv_display_add_event_cb(disp, area_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    /* Red-only night-mode pixel transform (no-op outside night mode). */
    const esp_lv_adapter_draw_bitmap_callbacks_t draw_cbs = {
        .custom_draw_bitmap = night_mode_draw_bitmap,
    };
    esp_lv_adapter_set_draw_bitmap_callbacks(disp, &draw_cbs, NULL);

    ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), TAG, "adapter start");
    display_controller_attach_lvgl();

    /* Background telemetry cache (AXP + RTC) so the UI never blocks on I2C.
     * Must start before power_mgmt_init(): its wake task reads the cached RTC
     * for night-mode checks. */
    sensor_cache_init();

    /* Alarm clock: load config + arm the RTC alarm. Must happen before
     * power_mgmt_init() so pm_arm_gpio_wakeup() sees the armed state. */
    alarm_init();

    /* Power management: DFS + light sleep + wake sources. */
    power_mgmt_init();

    /* Alarm clocks: show the ring screen when the alarm starts/stops. */
    alarm_register_ring_cb(alarm_ring_cb);

    /* Default charge current: 500 mA (~0.45C for the 1100 mAh cell, well
     * under its typical 1C max rating). */
    axp2101_set_charge_current_ma(twatch_pmu_dev, 500);

    /* Force a full redraw when night mode toggles so the red-only transform
     * reaches every pixel. */
    power_mgmt_register_night_mode_cb(night_mode_changed);

    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        lvgl_build_boot_screen();
        lv_timer_create(boot_to_watch_face, 3000, NULL);
        esp_lv_adapter_unlock();
    }
    display_controller_request_visible(DISPLAY_REASON_BOOT);

    /* Touch input (CST9217). */
    esp_lcd_touch_handle_t tp = touch_controller_get_handle();
    if (tp) {
        esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, tp);
        touch_cfg.callbacks.custom_touch_read = cst9217_read_points;
        uint16_t nx = 0, ny = 0;
        if (touch_controller_get_resolution(&nx, &ny) == ESP_OK && nx && ny) {
            touch_cfg.scale.x = (float)CO5300_RES_X / nx;
            touch_cfg.scale.y = (float)CO5300_RES_Y / ny;
        }
        s_touch_indev = esp_lv_adapter_register_touch(&touch_cfg);
        if (s_touch_indev) {
            ESP_LOGI(TAG, "touch registered");
        } else {
            ESP_LOGE(TAG, "touch registration failed");
        }
        /* The touch driver re-enables its GPIO interrupt; re-apply night mode
         * so touch stays disabled (no accidental wake) during night hours. */
        power_mgmt_recheck_night_mode();
    }

    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        navigation_init(s_touch_indev, navigation_clock_ms);
        esp_lv_adapter_unlock();
    }

    ESP_LOGI(TAG, "LVGL started (watch face)");

    /* BHI260AP sensor task (needs SPIFFS assets, already mounted above). */
    if (xTaskCreate(bhi260_task, "bhi260", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "BHI260AP task allocation failed");
    }

    /* GNSS control task. GNSS is off at startup unless the GPS screen switch
     * is on (default off); enabling it starts one long-lived power-on session
     * that survives sleep (see lvgl_gps_enabled()). */
    if (s_gps_ctrl_task == NULL) {
        /* REVERTED to 8192: the 3072 cut (based on an idle-only high-water-mark
     * measurement) overflowed live the first time this task's real GNSS
     * power-on path (deep ubxlib call chains) actually ran - clean
     * stack-canary panic + reboot, not silent corruption, but a real
     * regression. Idle-snapshot stack sizing is not safe for tasks with
     * deep, rarely-exercised branches - see docs/application.md section 12. */
        if (xTaskCreate(gps_ctrl_task, "gps_ctrl", 8192, NULL,
                        ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY, &s_gps_ctrl_task) != pdPASS) {
            s_gps_ctrl_task = NULL;
            ESP_LOGE(TAG, "GNSS control task allocation failed");
        }
    }
    lvgl_gps_set_enabled(gps_load_enabled());

    /* Step-gated distance tracking (lifetime distance + steps). No-op while
     * TRACKING_ENABLED is 0. */
    tracking_init();

    return ESP_OK;
}

/* ---- Screenshot dump (debug) ----
 * Captures the active LVGL screen and streams it over the USB-JTAG console
 * as raw RGB565 (little-endian) pixels, framed for the host decode script:
 *   ==SHOT:<w>x<h>==  <raw pixels>  ==ENDSHOT==
 * Byte order is the native little-endian RGB565 snapshot. Call from any task;
 * the LVGL lock is held only during the snapshot, not during the transfer. */

esp_err_t lvgl_app_dump_screenshot(void)
{
    int w = CO5300_RES_X;
    int h = CO5300_RES_Y;
    size_t px_size = (size_t)w * h * 2;

    /* Snapshot buffer must come from PSRAM: internal RAM (~365 KB) cannot hold
     * a 410x502 RGB565 frame. */
    void *px = heap_caps_aligned_alloc(16, px_size, MALLOC_CAP_SPIRAM);
    if (!px) {
        ESP_LOGE(TAG, "screenshot: px alloc failed (%u B)", (unsigned)px_size);
        return ESP_ERR_NO_MEM;
    }

    /* Capture under the LVGL lock (fast), then release before any transfer. */
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        free(px);
        return ESP_FAIL;
    }
    lv_image_dsc_t dsc;
    lv_result_t res = lv_snapshot_take_to_buf(lv_screen_active(), LV_COLOR_FORMAT_RGB565,
                                              &dsc, px, px_size);
    esp_lv_adapter_unlock();
    if (res != LV_RESULT_OK) {
        ESP_LOGE(TAG, "screenshot: snapshot failed");
        free(px);
        return ESP_FAIL;
    }

    /* Prefer saving to the SD card as PNG; fall back to streaming raw RGB565
     * over the USB-JTAG console when no card is present. */
    if (sd_log_save_screenshot((const uint16_t *)px, w, h) == ESP_OK) {
        free(px);
        return ESP_OK;
    }

    /* Suppress ESP logging while streaming so no other task interleaves text. */
    esp_log_level_t lvl = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_NONE);
    fflush(stdout);

    char header[64];
    snprintf(header, sizeof(header), "==SHOT:%dx%d==\n", w, h);
    fputs(header, stdout);

    /* Raw RGB565 little-endian via stdout (mirrors to USB-JTAG). */
    size_t chunk = 4096;
    const uint8_t *src = px;
    size_t left = px_size;
    while (left > 0) {
        size_t c = (left < chunk) ? left : chunk;
        size_t wr = fwrite(src, 1, c, stdout);
        src += wr;
        left -= wr;
    }
    printf("==ENDSHOT==\n");

    free(px);
    fflush(stdout);
    esp_log_level_set("*", lvl);
    return ESP_OK;
}
