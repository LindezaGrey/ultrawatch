/*
 * mesh_screen.c - Mesh screen (last few received Meshtastic messages) +
 * its Node-Overview sub-screen. Shared between the firmware and the host
 * sim - see watch_face.c's header comment for the mechanism.
 *
 * The background listener task (mesh_log.c) runs always-on, independent
 * of whether this screen is open; mesh_screen_update() only pulls a
 * snapshot to display. lvgl_mesh_screen_show() (main/lvgl_app.c) - called
 * from that background task when a new message arrives - stays
 * firmware-only (needs esp_lv_adapter_lock()/request_wake(), no sim
 * equivalent) and calls into lvgl_build_mesh_screen()/mesh_screen_update()
 * here.
 */
#include "screens.h"
#include <stdio.h>
#include <time.h>
#include "cascadia_fonts.h"
#include "mesh_log.h"
#include "sensor_cache.h"
#include "pcf85063a.h"

static const lv_font_t *s_font_sec   = &cascadia_36;
static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_micro = &cascadia_18;

lv_obj_t *s_mesh_screen;
lv_obj_t *s_node_screen;

static lv_obj_t *s_mesh_pwr_switch;    /* LoRa radio on/off (mesh_log_set_enabled()) */
static lv_obj_t *s_mesh_empty_label;
static lv_obj_t *s_mesh_conn_label;    /* channel + node count */
static lv_obj_t *s_mesh_list_cont;     /* scrollable row container */
static lv_obj_t *s_mesh_row_label[MESH_LOG_COUNT];
static lv_obj_t *s_mesh_preset_label[MESH_PRESET_COUNT];

static lv_obj_t *s_node_empty_label;
static lv_obj_t *s_node_row_label[MESH_NODE_TABLE_MAX];

void mesh_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_mesh_screen) {
        return;
    }

    /* LoRa on/off switch state. */
    if (s_mesh_pwr_switch) {
        bool on = mesh_log_get_enabled();
        if (lv_obj_has_state(s_mesh_pwr_switch, LV_STATE_CHECKED) != on) {
            if (on) {
                lv_obj_add_state(s_mesh_pwr_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(s_mesh_pwr_switch, LV_STATE_CHECKED);
            }
        }
    }

    mesh_msg_t msgs[MESH_LOG_COUNT];
    size_t n = mesh_log_get_recent(msgs, MESH_LOG_COUNT);

    if (n == 0) {
        lv_obj_clear_flag(s_mesh_empty_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_mesh_empty_label, LV_OBJ_FLAG_HIDDEN);
    }

    /* Connection details: channel + reachable node count, from the most
     * recent message and the node table (docs/application.md 7.2 point 3). */
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

    /* Wall-clock HH:MM of receipt, derived from the current RTC time minus
     * each message's age - avoids adding a wall-clock field to mesh_msg_t
     * (which only stores a monotonic mesh_log_now_us() stamp) just for
     * this row's display. */
    pcf85063a_time_t rtc;
    bool have_rtc = sensor_cache_get_rtc(&rtc);
    time_t now_epoch = have_rtc ? pcf85063a_time_to_epoch(&rtc) : 0;

    int64_t now_us = mesh_log_now_us();
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
            /* Known channel, decrypted fine, just not a text message (e.g.
             * NodeInfo, telemetry) - a distinct blue-grey from both a real
             * message (light) and an unknown channel (dim), since this one
             * genuinely was decrypted successfully. */
            snprintf(buf, sizeof(buf), "%s  %s  (node info)", tbuf, sender);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0x8FB0D0), 0);
            break;
        case MESH_MSG_UNKNOWN:
        default:
            /* channel_hash matched none of our known channels, or the
             * decrypt didn't parse as a valid Data message - still shown
             * (always show headers), dimmed to set it apart from content we
             * actually got something out of. */
            snprintf(buf, sizeof(buf), "%s  %s  (unknown)", tbuf, sender);
            lv_obj_set_style_text_color(s_mesh_row_label[i], lv_color_hex(0x777766), 0);
            break;
        }
        lv_label_set_text(s_mesh_row_label[i], buf);
    }

    if (s_mesh_preset_label[0]) {
        for (int i = 0; i < MESH_PRESET_COUNT; i++) {
            char buf[MESH_PRESET_MAX_LEN + 1];
            mesh_preset_get(i, buf, sizeof(buf));
            lv_label_set_text(s_mesh_preset_label[i], buf);
        }
    }
}

/* Preset buttons are visually complete per docs/application.md 7.2 point 5
 * but functionally inert: LoRa sending is out of scope until a
 * separately-scoped TX-stack project ships (see the Phase 1 planning
 * notes) - tapping one is a no-op, not a stub that pretends to send. */
static void mesh_preset_btn_cb(lv_event_t *e)
{
    (void)e;
}

/* Mesh -> Node overview is now handled at the indev level (an upward swipe
 * anywhere on the Mesh screen), same as every other nav-ring gesture - see
 * main/lvgl_app.c's swipe_event_cb(). A per-object PRESSED/RELEASED
 * callback registered directly on the message list's own (natively
 * scrollable) container used to live here, but was reported unreliable
 * in practice - most likely LVGL's own scroll-gesture recognition on that
 * scrollable object competing for the same press/release sequence. */

/* LoRa on/off switch on the Mesh screen, mirroring the GPS screen's GNSS
 * switch exactly (main/screens/gps_screen.c's gps_pwr_switch_cb() header
 * comment explains why the callback only sets the target flag and never
 * touches switch/screen state itself - same reasoning here:
 * mesh_log_set_enabled() is async, mesh_log_task applies it and
 * mesh_screen_update() reflects the result back on its own next tick. */
static void mesh_pwr_switch_cb(lv_event_t *e)
{
    (void)e;
    if (!s_mesh_pwr_switch) {
        return;
    }
    bool on = lv_obj_has_state(s_mesh_pwr_switch, LV_STATE_CHECKED);
    mesh_log_set_enabled(on);
}

void lvgl_build_mesh_screen(void)
{
    s_mesh_screen = screen_new();
    lv_obj_set_style_bg_color(s_mesh_screen, lv_color_hex(0x201810), 0);

    /* High-DPI sizing (AGENT.md "Display density & UI sizing"): title
     * promotes to cascadia_36 and conn_label to cascadia_22, same as
     * Settings/GPS. The message list itself (8 rows, LONG_CLIP-fixed) and
     * the preset labels (user-editable, up to 31 chars) stay at
     * cascadia_18 - both are dense/variable-length content that would
     * overflow their fixed-width slots at a bigger font, same reasoning
     * as GPS's diagnostics line. Everything below the title shifts down
     * to make room for the now-taller title. */
    lv_obj_t *title = lv_label_create(s_mesh_screen);
    lv_label_set_text(title, "MESH");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    /* LoRa on/off switch - same row/position convention as the GPS screen's
     * GNSS switch (main/screens/gps_screen.c). */
    lv_obj_t *pwr_lbl = lv_label_create(s_mesh_screen);
    lv_label_set_text(pwr_lbl, "LoRa");
    lv_obj_set_style_text_font(pwr_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(pwr_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(pwr_lbl, LV_ALIGN_TOP_LEFT, 90, 22);
    s_mesh_pwr_switch = lv_switch_create(s_mesh_screen);
    lv_obj_set_size(s_mesh_pwr_switch, 66, 36);
    lv_obj_align(s_mesh_pwr_switch, LV_ALIGN_TOP_RIGHT, -90, 14);
    lv_obj_add_event_cb(s_mesh_pwr_switch, mesh_pwr_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

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

    /* Scrollable row container - future-proofs the list if MESH_LOG_COUNT
     * ever grows past 8 (little practical effect today). */
    s_mesh_list_cont = lv_obj_create(s_mesh_screen);
    lv_obj_set_size(s_mesh_list_cont, 380, 250);
    lv_obj_align(s_mesh_list_cont, LV_ALIGN_TOP_MID, 0, 96);
    lv_obj_set_flex_flow(s_mesh_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_mesh_list_cont, 4, 0);
    lv_obj_set_style_bg_opa(s_mesh_list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_mesh_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_mesh_list_cont, 2, 0);

    /* One single-line label per ring-buffer slot, newest first, stacked top
     * to bottom. Fixed size + CLIP long-mode so an over-length message is
     * silently cropped within its own row instead of wrapping/spilling into
     * the next one. LV_LABEL_LONG_DOT was tried first but hangs the software
     * render thread forever (confirmed live via JTAG/GDB: CPU1 gets stuck
     * permanently inside lv_draw_label_iterate_characters, looping the same
     * line without progress) when a label is shorter than its content height
     * and contains an explicit "\n" - the DOT ellipsis-placement math doesn't
     * handle that combination. Promoted from cascadia_18 to cascadia_22
     * (s_font_small) per explicit feedback that it read too small - the
     * container is natively scrollable (never disabled), so the row height
     * growing from 26 to 32px just means fewer of the 8 rows are visible
     * without scrolling, not an overflow risk the way a fixed-height box
     * would be. */
    for (int i = 0; i < MESH_LOG_COUNT; i++) {
        lv_obj_t *l = lv_label_create(s_mesh_list_cont);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_small, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
        lv_obj_set_size(l, 370, 32);
        s_mesh_row_label[i] = l;
    }

    /* Preset buttons (inert, see mesh_preset_btn_cb()) - 2x2 grid. Text
     * comes from mesh_preset_get() (NVS-backed, editable via the debug
     * console's `presetset` - see mesh_log.h) and is refreshed on every
     * mesh_screen_update() tick so a console edit shows up live. */
    for (int i = 0; i < 4; i++) {
        int col = i % 2;
        int row = i / 2;
        lv_obj_t *btn = lv_button_create(s_mesh_screen);
        lv_obj_set_size(btn, 180, 44);
        lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 15 + col * 195, 356 + row * 52);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3226), 0);
        lv_obj_t *l = lv_label_create(btn);
        lv_obj_set_style_text_font(l, s_font_micro, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0x888888), 0);
        s_mesh_preset_label[i] = l;
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

/* ---- Node-Overview sub-screen (local, not in the nav ring) ---- */

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

    int64_t now_us = mesh_log_now_us();
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

    /* High-DPI sizing + the same rounded-corner back-button fix as
     * Settings (AGENT.md "Display density & UI sizing"; the button
     * position was measured against the actual safe-area mask, see
     * commit cfb7307). The per-node rows stay at cascadia_18 - dense,
     * LONG_CLIP-fixed table data like GPS's diagnostics line or Mesh's
     * message rows, would overflow at a bigger font. */
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

/* Not static: sim/main.c's UWATCH_SIM_SCREEN dev shortcut calls this
 * directly, since node overview is only reachable on real hardware via
 * an upward fling on the Mesh screen - not a swipe sim/nav.c wires up. */
void lvgl_show_node_overview(void)
{
    if (!s_node_screen) {
        lvgl_build_node_screen();
    }
    /* Load first, refresh after: node_screen_update() no-ops unless
     * s_node_screen is already the active screen (same guard every other
     * screen's update function uses), so calling it before the load here
     * would silently do nothing and leave stale/empty rows up to 1s until
     * the periodic timer corrects it. */
    lv_scr_load(s_node_screen);
    node_screen_update(NULL);
}
