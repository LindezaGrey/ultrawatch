/*
 * mesh_screen.c - the Mesh screen + its Node-Overview sub-screen from
 * main/lvgl_app.c (2026-08-29 Phase 4 rework: compact rows, connection
 * details line, scrollable row container with a fling-vs-scroll gesture,
 * inert preset grid, separate node table), copied verbatim and run against
 * mock_hw.c's mesh_log_get_recent()/mesh_log_get_nodes() instead of the
 * real ring buffer/node table.
 *
 * Deviations from the firmware original:
 *   - screen_new() copied in here too, same as every non-watch-face screen.
 *   - esp_timer_get_time() (ESP-IDF, unavailable on the host) replaced with
 *     lv_tick_get() * 1000 (ms -> us), the same "time since start" tick the
 *     sim already drives from SDL_GetTicks() (see main.c); mock_hw.c's
 *     mesh_log_get_recent()/mesh_log_get_nodes() use the matching
 *     mock_now_s()*1e6 clock so the age-in-seconds math still comes out
 *     sane.
 *   - The fling-vs-scroll gesture math (mesh_gesture_cb) is copied
 *     unmodified - it only uses lv_tick_get()/lv_indev_get_point(), both
 *     already available here.
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include <time.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_micro = &cascadia_18;
static const lv_font_t *s_font_sec   = &cascadia_36;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_mesh_screen;

static lv_obj_t *s_mesh_empty_label;
static lv_obj_t *s_mesh_conn_label;
static lv_obj_t *s_mesh_list_cont;
static lv_obj_t *s_mesh_row_label[MESH_LOG_COUNT];
static lv_point_t s_mesh_press;
static uint32_t s_mesh_press_tick;

static lv_obj_t *s_node_screen;
static lv_obj_t *s_node_empty_label;
static lv_obj_t *s_node_row_label[MESH_NODE_TABLE_MAX];

static void lvgl_build_node_screen(void);
static void lvgl_show_node_overview(void);
static void mesh_screen_update(lv_timer_t *timer);

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

    if (s_mesh_conn_label) {
        char cbuf[40];
        if (n > 0) {
            snprintf(cbuf, sizeof(cbuf), "Ch 0x%02x  Nodes: %u",
                     (unsigned)msgs[0].channel_hash, (unsigned)mesh_log_node_count());
        } else {
            snprintf(cbuf, sizeof(cbuf), "Nodes: %u", (unsigned)mesh_log_node_count());
        }
        lv_label_set_text(s_mesh_conn_label, cbuf);
    }

    pcf85063a_time_t rtc;
    bool have_rtc = sensor_cache_get_rtc(&rtc);
    time_t now_epoch = have_rtc ? pcf85063a_time_to_epoch(&rtc) : 0;

    int64_t now_us = (int64_t)lv_tick_get() * 1000;
    for (size_t i = 0; i < MESH_LOG_COUNT; i++) {
        if (i >= n) {
            lv_label_set_text(s_mesh_row_label[i], "");
            continue;
        }
        uint32_t age_s = (uint32_t)((now_us - msgs[i].received_at_us) / 1000000);
        char tbuf[8] = "--:--";
        if (have_rtc) {
            time_t msg_epoch = now_epoch - (time_t)age_s;
            struct tm lt;
            localtime_r(&msg_epoch, &lt);
            snprintf(tbuf, sizeof(tbuf), "%02u:%02u", (unsigned)lt.tm_hour, (unsigned)lt.tm_min);
        }
        char sender[40];
        mesh_log_node_name(msgs[i].from, sender, sizeof(sender));
        if (sender[0] == '\0') {
            snprintf(sender, sizeof(sender), "!%08lx", (unsigned long)msgs[i].from);
        }

        char buf[MESH_LOG_TEXT_MAX + 96];
        switch (msgs[i].kind) {
        case MESH_MSG_TEXT:
            snprintf(buf, sizeof(buf), "%s  %s  %s", tbuf, sender, msgs[i].text);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0xE0E0E0), 0);
            break;
        case MESH_MSG_OTHER:
            snprintf(buf, sizeof(buf), "%s  %s  (node info)", tbuf, sender);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0x8FB0D0), 0);
            break;
        case MESH_MSG_UNKNOWN:
        default:
            snprintf(buf, sizeof(buf), "%s  %s  (unknown)", tbuf, sender);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0x777766), 0);
            break;
        }
        lv_label_set_text(s_mesh_row_label[i], buf);
    }
}

static void mesh_preset_btn_cb(lv_event_t *e)
{
    (void)e;
}

#define MESH_FLING_MIN_DIST_PX 60
#define MESH_FLING_MAX_MS      400
#define MESH_FLING_MIN_VEL     0.5f

static void mesh_gesture_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_active();
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        s_mesh_press = p;
        s_mesh_press_tick = lv_tick_get();
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) {
        return;
    }
    int dy = p.y - s_mesh_press.y;
    uint32_t elapsed = lv_tick_get() - s_mesh_press_tick;
    if (dy < -MESH_FLING_MIN_DIST_PX && elapsed > 0 && elapsed < MESH_FLING_MAX_MS &&
            (-dy / (float)elapsed) > MESH_FLING_MIN_VEL) {
        lvgl_show_node_overview();
    }
}

static void lvgl_build_mesh_screen(void)
{
    s_mesh_screen = screen_new();
    lv_obj_set_style_bg_color(s_mesh_screen, lv_color_hex(0x201810), 0);

    lv_obj_t *title = lv_label_create(s_mesh_screen);
    lv_label_set_text(title, "MESH");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    s_mesh_conn_label = lv_label_create(s_mesh_screen);
    lv_label_set_text(s_mesh_conn_label, "");
    lv_obj_set_style_text_font(s_mesh_conn_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_mesh_conn_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_mesh_conn_label, LV_ALIGN_TOP_MID, 0, 62);

    s_mesh_empty_label = lv_label_create(s_mesh_screen);
    lv_label_set_text(s_mesh_empty_label, "No messages yet");
    lv_obj_set_style_text_font(s_mesh_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_mesh_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_mesh_empty_label, LV_ALIGN_CENTER, 0, 0);

    s_mesh_list_cont = lv_obj_create(s_mesh_screen);
    lv_obj_set_size(s_mesh_list_cont, 380, 250);
    lv_obj_align(s_mesh_list_cont, LV_ALIGN_TOP_MID, 0, 96);
    lv_obj_set_flex_flow(s_mesh_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_mesh_list_cont, 4, 0);
    lv_obj_set_style_bg_opa(s_mesh_list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_mesh_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_mesh_list_cont, 2, 0);
    lv_obj_add_event_cb(s_mesh_list_cont, mesh_gesture_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_mesh_list_cont, mesh_gesture_cb, LV_EVENT_RELEASED, NULL);

    for (int i = 0; i < MESH_LOG_COUNT; i++) {
        lv_obj_t *l = lv_label_create(s_mesh_list_cont);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_micro, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
        lv_obj_set_size(l, 370, 26);
        s_mesh_row_label[i] = l;
    }

    static const char *presets[4] = { "Bin ok", "Verzoegerung", "Notfall", "Standort senden" };
    for (int i = 0; i < 4; i++) {
        int col = i % 2;
        int row = i / 2;
        lv_obj_t *btn = lv_button_create(s_mesh_screen);
        lv_obj_set_size(btn, 180, 44);
        lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 15 + col * 195, 356 + row * 52);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3226), 0);
        lv_obj_t *l = lv_label_create(btn);
        lv_label_set_text(l, presets[i]);
        lv_obj_set_style_text_font(l, s_font_micro, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0x888888), 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(btn, mesh_preset_btn_cb, LV_EVENT_CLICKED, NULL);
    }

    lv_obj_t *hint = lv_label_create(s_mesh_screen);
    lv_label_set_text(hint, "< swipe left: GPS");
    lv_obj_set_style_text_font(hint, s_font_micro, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);

    mesh_screen_update(NULL);
    lv_timer_create(mesh_screen_update, 1000, NULL);
}

/* ---- Node-Overview sub-screen ---- */

static void node_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_node_screen) {
        return;
    }
    mesh_node_t nodes[MESH_NODE_TABLE_MAX];
    size_t n = mesh_log_get_nodes(nodes, MESH_NODE_TABLE_MAX);

    if (n == 0) {
        lv_obj_clear_flag(s_node_empty_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_node_empty_label, LV_OBJ_FLAG_HIDDEN);
    }

    int64_t now_us = (int64_t)lv_tick_get() * 1000;
    for (size_t i = 0; i < MESH_NODE_TABLE_MAX; i++) {
        if (i >= n) {
            lv_obj_add_flag(s_node_row_label[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(s_node_row_label[i], LV_OBJ_FLAG_HIDDEN);
        uint32_t age_s = (uint32_t)((now_us - nodes[i].last_seen_us) / 1000000);
        char buf[64];
        const char *name = nodes[i].name[0] ? nodes[i].name : NULL;
        if (name) {
            snprintf(buf, sizeof(buf), "%-16s %lus ago  %ddBm %+ddB",
                     name, (unsigned long)age_s, (int)nodes[i].last_rssi_dbm,
                     (int)nodes[i].last_snr_db);
        } else {
            snprintf(buf, sizeof(buf), "!%08lx  %lus ago  %ddBm %+ddB",
                     (unsigned long)nodes[i].node_id, (unsigned long)age_s,
                     (int)nodes[i].last_rssi_dbm, (int)nodes[i].last_snr_db);
        }
        lv_label_set_text(s_node_row_label[i], buf);
    }
}

static void node_back_btn_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_mesh_screen);
    mesh_screen_update(NULL);
}

static void lvgl_build_node_screen(void)
{
    s_node_screen = screen_new();
    lv_obj_set_style_bg_color(s_node_screen, lv_color_hex(0x102018), 0);

    lv_obj_t *back = lv_button_create(s_node_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, node_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_node_screen);
    lv_label_set_text(title, "NODES");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    s_node_empty_label = lv_label_create(s_node_screen);
    lv_label_set_text(s_node_empty_label, "No nodes seen yet");
    lv_obj_set_style_text_font(s_node_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_node_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_node_empty_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *list_cont = lv_obj_create(s_node_screen);
    lv_obj_set_size(list_cont, 380, 320);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 140);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 4, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 2, 0);

    for (int i = 0; i < MESH_NODE_TABLE_MAX; i++) {
        lv_obj_t *l = lv_label_create(list_cont);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_micro, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
        lv_obj_set_size(l, 370, 24);
        lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
        s_node_row_label[i] = l;
    }

    node_screen_update(NULL);
    lv_timer_create(node_screen_update, 1000, NULL);
}

static void lvgl_show_node_overview(void)
{
    if (!s_node_screen) {
        lvgl_build_node_screen();
    }
    /* Load first, refresh after - node_screen_update() no-ops unless
     * s_node_screen is already active, same as main/lvgl_app.c. */
    lv_scr_load(s_node_screen);
    node_screen_update(NULL);
}

/* Public entry point for screenshot/dev-shortcut use (see main.c's
 * UWATCH_SIM_SCREEN env var) - the node overview is reached by an upward
 * fling from Mesh on real hardware, not a swipe the sim's nav.c wires up. */
void sim_node_screen_build(void)
{
    lvgl_show_node_overview();
}

void sim_mesh_screen_build(void)
{
    if (!s_mesh_screen) {
        lvgl_build_mesh_screen();
    }
    lv_scr_load(s_mesh_screen);
    mesh_screen_update(NULL);
}
