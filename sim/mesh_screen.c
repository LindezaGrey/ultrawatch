/*
 * mesh_screen.c - the real Mesh screen from main/lvgl_app.c
 * (mesh_screen_update / lvgl_build_mesh_screen, added alongside the
 * Meshtastic decoder work), copied verbatim (same layout, same single-line
 * CLIP-mode rows) and run against mock_hw.c's mesh_log_get_recent() instead
 * of the real ring buffer.
 *
 * Deviations from the firmware original:
 *   - screen_new() copied in here too, same as every non-watch-face screen.
 *   - esp_timer_get_time() (ESP-IDF, unavailable on the host) replaced with
 *     lv_tick_get() * 1000 (ms -> us), the same "time since start" tick the
 *     sim already drives from SDL_GetTicks() (see main.c); mock_hw.c's
 *     mesh_log_get_recent() uses the matching mock_now_s()*1e6 clock so the
 *     age-in-seconds math still comes out sane.
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_micro = &cascadia_18;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_mesh_screen;

static lv_obj_t *s_mesh_empty_label;
static lv_obj_t *s_mesh_row_label[MESH_LOG_COUNT];

static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

static void mesh_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_mesh_screen) {
        return;
    }

    mesh_msg_t msgs[MESH_LOG_COUNT];
    size_t n = mesh_log_get_recent(msgs, MESH_LOG_COUNT);

    if (n == 0) {
        lv_obj_clear_flag(s_mesh_empty_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_mesh_empty_label, LV_OBJ_FLAG_HIDDEN);
    }

    int64_t now_us = (int64_t)lv_tick_get() * 1000;
    for (size_t i = 0; i < MESH_LOG_COUNT; i++) {
        if (i >= n) {
            lv_label_set_text(s_mesh_row_label[i], "");
            continue;
        }
        uint32_t age_s = (uint32_t)((now_us - msgs[i].received_at_us) / 1000000);
        char buf[MESH_LOG_TEXT_MAX + 48];
        switch (msgs[i].kind) {
        case MESH_MSG_TEXT:
            snprintf(buf, sizeof(buf), "!%08lx ch=0x%02x %ddBm %+ddB %lus: %s",
                     (unsigned long)msgs[i].from, (unsigned)msgs[i].channel_hash,
                     (int)msgs[i].rssi_dbm, (int)msgs[i].snr_db,
                     (unsigned long)age_s, msgs[i].text);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0xE0E0E0), 0);
            break;
        case MESH_MSG_OTHER:
            snprintf(buf, sizeof(buf), "!%08lx ch=0x%02x %ddBm %+ddB %lus %s",
                     (unsigned long)msgs[i].from, (unsigned)msgs[i].channel_hash,
                     (int)msgs[i].rssi_dbm, (int)msgs[i].snr_db,
                     (unsigned long)age_s, msgs[i].text);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0x8FB0D0), 0);
            break;
        case MESH_MSG_UNKNOWN:
        default:
            snprintf(buf, sizeof(buf), "!%08lx ch=0x%02x %ddBm %+ddB %lus (unknown channel)",
                     (unsigned long)msgs[i].from, (unsigned)msgs[i].channel_hash,
                     (int)msgs[i].rssi_dbm, (int)msgs[i].snr_db,
                     (unsigned long)age_s);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0x777766), 0);
            break;
        }
        lv_label_set_text(s_mesh_row_label[i], buf);
    }
}

static void lvgl_build_mesh_screen(void)
{
    s_mesh_screen = screen_new();
    lv_obj_set_style_bg_color(s_mesh_screen, lv_color_hex(0x201810), 0);

    lv_obj_t *title = lv_label_create(s_mesh_screen);
    lv_label_set_text(title, "MESH");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_mesh_empty_label = lv_label_create(s_mesh_screen);
    lv_label_set_text(s_mesh_empty_label, "No messages yet");
    lv_obj_set_style_text_font(s_mesh_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_mesh_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_mesh_empty_label, LV_ALIGN_CENTER, 0, 0);

    /* One single-line label per ring-buffer slot, newest first, stacked top
     * to bottom - see main/lvgl_app.c's lvgl_build_mesh_screen() for why DOT
     * mode was replaced with CLIP (hung the render thread) and why the row
     * text has no "\n" (a forced second line overlapped the row below). */
    for (int i = 0; i < MESH_LOG_COUNT; i++) {
        lv_obj_t *l = lv_label_create(s_mesh_screen);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_micro, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
        lv_obj_set_size(l, 380, 26);
        lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 56 + i * 30);
        s_mesh_row_label[i] = l;
    }

    lv_obj_t *hint = lv_label_create(s_mesh_screen);
    lv_label_set_text(hint, "< swipe left: GPS");
    lv_obj_set_style_text_font(hint, s_font_micro, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);

    mesh_screen_update(NULL);
    lv_timer_create(mesh_screen_update, 1000, NULL);
}

void sim_mesh_screen_build(void)
{
    if (!s_mesh_screen) {
        lvgl_build_mesh_screen();
    }
    lv_scr_load(s_mesh_screen);
}
