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
#include "cst9217.h"
#include "bhi260ap.h"
#include "sd_log.h"
#include "pcf85063a.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "tracking.h"
#include "sensor_cache.h"
#include "m10q.h"
#include "power_mgmt.h"
#include "alarm.h"
#include "cd_timer.h"
#include "gpx_log.h"
#include "mesh_log.h"
#include "st25r3916.h"
#include "ndef.h"
#include "ble_debug.h"

static const char *TAG = "lvgl_app";

/* Fonts (embedded Cascadia Code bitmaps, see cascadia_fonts.h). */
static const lv_font_t *s_font_time = &cascadia_72;   /* HH:MM:SS */
static const lv_font_t *s_font_sec  = &cascadia_36;   /* UTC time */
static const lv_font_t *s_font_small = &cascadia_22;  /* body text */
static const lv_font_t *s_font_micro = &cascadia_18;  /* GPS diag line */

/* Watch face objects. */
static lv_obj_t *s_tz_label;    /* timezone abbreviation, e.g. "CEST" - docs/application.md section 4.1 */
static lv_obj_t *s_time_label;
static lv_obj_t *s_sec_label;
static lv_obj_t *s_date_label;
static lv_obj_t *s_batt_label;
static lv_obj_t *s_batt_fill;
static lv_obj_t *s_steps_label; /* step count on the watch face */
static lv_obj_t *s_gps_icon;   /* satellite status icon (grey/red/green) */
static lv_obj_t *s_track_dot;  /* solid red dot: tracking session active */
static lv_obj_t *s_snooze_icon; /* "Zz" shown while snoozing */

/* BHI260AP status screen. */
static lv_obj_t *s_bhi_screen;
static lv_obj_t *s_bhi_status_label;
static lv_obj_t *s_bhi_rv_acc_label;
static lv_obj_t *s_bhi_activity_label;
static lv_obj_t *s_bhi_gesture_label;
static lv_obj_t *s_daily_act_label[DAILY_ACT_COUNT];   /* today's per-activity minutes */
static lv_obj_t *s_cube_line[12];                       /* GAMERV 3D wireframe cube */
static lv_point_precise_t s_cube_pts[12][2];
static lv_obj_t *s_axis_line[3];                        /* x/y/z origin pointer */
static lv_point_precise_t s_axis_pts[3][2];

/* GPS screen (skyplot + fix info). */
static lv_obj_t *s_gps_screen;
static lv_obj_t *s_gps_status_label;
static lv_obj_t *s_gps_pos_label;
static lv_obj_t *s_gps_speed_label;
static lv_obj_t *s_gps_sats_label;
static lv_obj_t *s_gps_diag_label;           /* GNSS diagnostics (state/offset/ttff/rx) */
static lv_obj_t *s_gps_dots[M10Q_MAX_SATS];   /* satellite dots (in view order) */
static volatile bool s_gps_powered;
static uint32_t s_gps_acq_start_ms;            /* power-on timestamp */
static lv_obj_t *s_gps_track_label;            /* tracking stats (distance/steps/avg) */
static lv_obj_t *s_gps_track_btn;              /* Start/Stop tracking button */
static lv_obj_t *s_gps_pwr_switch;             /* GNSS on/off switch */
static bool s_gps_enabled;                     /* persisted "GNSS on" choice */

/* Mesh screen (Meshtastic message log, nav-ring slot 2). */
static lv_obj_t *s_mesh_screen;
static lv_obj_t *s_mesh_empty_label;
static lv_obj_t *s_mesh_conn_label;    /* channel + node count */
static lv_obj_t *s_mesh_list_cont;     /* scrollable row container - also the fling-gesture target */
static lv_obj_t *s_mesh_row_label[MESH_LOG_COUNT];
static lv_obj_t *s_mesh_preset_label[MESH_PRESET_COUNT];
static lv_point_t s_mesh_press;
static uint32_t s_mesh_press_tick;

/* Node-Overview sub-screen (local, not in the nav ring - reached from the
 * Mesh screen via a fast upward fling, see docs/application.md section
 * 7.2 point 4). */
static lv_obj_t *s_node_screen;
static lv_obj_t *s_node_empty_label;
static lv_obj_t *s_node_row_label[MESH_NODE_TABLE_MAX];

/* NFC screen (chained off BHI: swipe left again). One-shot scan on demand -
 * st25r3916_open() holds the shared SPI2 bus rails (spi2_power.h) for as
 * long as a session is open, so scanning is explicit (a button), not
 * automatic on screen entry like GPS's always-on acquire. */
static lv_obj_t *s_nfc_screen;
static lv_obj_t *s_nfc_status_label;
static lv_obj_t *s_nfc_uid_label;
static lv_obj_t *s_nfc_start_btn;
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

/* Alarms/Timers list screen (nav-ring slot 4) + two local sub-screens
 * (alarm create/edit, timer start, both reached by button tap and returning
 * to the list the same way - see docs/application.md section 8) + the
 * shared ringing screen. */
static lv_obj_t *s_alarm_screen;               /* the LIST screen (nav-ring slot) */
static lv_obj_t *s_alarm_row[ALARM_MAX_COUNT]; /* one row per slot, hidden if !in_use */
static lv_obj_t *s_alarm_row_time_label[ALARM_MAX_COUNT];
static lv_obj_t *s_alarm_row_switch[ALARM_MAX_COUNT];
static lv_obj_t *s_alarm_timer_label;          /* active countdown, hidden if none running */
static lv_obj_t *s_alarm_timer_cancel_btn;

static lv_obj_t *s_alarm_edit_screen;          /* create/edit one alarm */
static lv_obj_t *s_alarm_edit_time_label;
static lv_obj_t *s_alarm_edit_mode_beep;
static lv_obj_t *s_alarm_edit_mode_vib;
static lv_obj_t *s_alarm_edit_mode_both;
static lv_obj_t *s_alarm_edit_wday_btn[7];     /* Sun..Sat */
static lv_obj_t *s_alarm_edit_delete_btn;      /* hidden while creating a new entry */
static int s_alarm_edit_idx = -1;              /* -1 = creating new, >=0 = editing that slot */
static uint8_t s_alarm_edit_hour, s_alarm_edit_min, s_alarm_edit_mode, s_alarm_edit_wmask;

static lv_obj_t *s_timer_screen;               /* countdown-timer duration presets */

static lv_obj_t *s_ring_screen;                /* alarm/timer RINGING screen - not a nav-ring screen, don't confuse with s_nav_ring below */
static lv_obj_t *s_ring_title_label;           /* "ALARM" or "TIMER" */
static lv_obj_t *s_ring_time_label;
static lv_obj_t *s_ring_snooze_btn;            /* hidden for a timer-sourced ring (no snooze concept) */

/* Settings screen (docs/application.md section 9): a category list
 * (nav-ring slot 3) + 5 local sub-pages (Zeit & Zeitzone, Display,
 * Peripherie, Ton & Vibration, Info - "Presets verwalten" and
 * "Ultra-Sparmodus" are deferred, see Phase 5 plan). Each sub-page is its
 * own screen, reached by tapping a category row and left the same way
 * every other local sub-screen is (a "< Back" button + a local
 * top-to-bottom swipe gesture). */
static lv_obj_t *s_settings_screen;            /* category list */

static lv_obj_t *s_set_tz_screen;
static lv_obj_t *s_set_tz_abbrev_label;
static lv_obj_t *s_set_tz_offset_label;

static lv_obj_t *s_set_disp_screen;
static lv_obj_t *s_set_disp_timeout_label;
static lv_obj_t *s_set_disp_bright_label;

static lv_obj_t *s_set_periph_screen;
static lv_obj_t *s_set_periph_gps_switch;
static lv_obj_t *s_set_periph_bt_switch;

static lv_obj_t *s_set_sound_screen;
static lv_obj_t *s_set_sound_alarm_switch;
static lv_obj_t *s_set_sound_notify_switch;

static lv_obj_t *s_set_info_screen;
static lv_obj_t *s_set_info_batt_label;
static lv_obj_t *s_set_info_sd_label;

/* Local swipe-down-to-go-back gesture, shared by every Settings sub-page -
 * distance-threshold only, registered per-sub-page-root with the
 * sub-page's own back callback as user data (so one handler serves all 5
 * pages). Deviates from docs/application.md section 9.2's own suggested
 * left-to-right swipe (explicit user preference: top-to-bottom instead). */
static void settings_sub_swipe_cb(lv_event_t *e);

/* GNSS control runs off the LVGL task (m10q_power blocks for seconds during
 * baud probing/config); the UI issues a request and a worker task applies it. */
#define GPS_CTRL_NONE    0
#define GPS_CTRL_ON      1
#define GPS_CTRL_OFF     2
#define GPS_CTRL_REFRESH 3   /* one-shot boot position check */
#define GPS_REFRESH_TIMEOUT_MS 120000
static volatile int s_gps_ctrl_req;
static TaskHandle_t s_gps_ctrl_task;
#define LKP_REFRESH_INTERVAL_MS (60 * 1000)   /* NVS write throttle, not fix rate */
static uint32_t s_gps_lkp_saved_ms;    /* xTaskGetTickCount()-scale, wraps like the others here */

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

/* Swipe detection at the input-device level (works regardless of widget). */
#define SWIPE_DIST         60
#define MENU_TIMEOUT_MS    5000   /* return to watch face after this idle */
#define BHI_TIMEOUT_MS     10000  /* BHI screen keeps the cube up a bit longer */
#define NFC_MENU_TIMEOUT_MS 10000  /* NFC screen: outlives one scan window (8s) */
static lv_indev_t *s_touch_indev;
static lv_point_t s_swipe_start;
static bool s_swipe_active;
static lv_obj_t *s_watch_screen;
static uint32_t s_last_touch_tick;   /* lv_tick_get() at last touch */

static void swipe_event_cb(lv_event_t *e);
static void watch_face_long_press_cb(lv_event_t *e);
static void gps_track_btn_cb(lv_event_t *e);
static void gps_pwr_switch_cb(lv_event_t *e);
static void lvgl_build_bhi_screen(void);
static void lvgl_build_gps_screen(void);
static void lvgl_build_mesh_screen(void);
static void lvgl_build_node_screen(void);
static void lvgl_show_node_overview(void);
static void lvgl_build_nfc_screen(void);
static void nfc_start_btn_cb(lv_event_t *e);
static void nfc_ctrl_task(void *arg);
static void gps_power(bool on);
static void gps_refresh(void);
static void gps_ctrl_task(void *arg);
static void lvgl_build_watch_face(void);
static void menu_timeout_cb(lv_timer_t *timer);
static void lvgl_show_watch_face(void);
static void lvgl_build_alarm_screen(void);
static void lvgl_build_alarm_edit_screen(void);
static void lvgl_build_timer_screen(void);
static void alarm_list_refresh(void);
static void lvgl_build_ring_screen(void);
static void lvgl_build_settings_screen(void);
static void lvgl_build_settings_tz_screen(void);
static void settings_tz_refresh(void);
static void lvgl_build_settings_disp_screen(void);
static void lvgl_show_settings_disp(void);
static void lvgl_build_settings_periph_screen(void);
static void settings_periph_refresh(lv_timer_t *timer);
static void lvgl_build_settings_sound_screen(void);
static void settings_sound_refresh(void);
static void lvgl_build_settings_info_screen(void);
static void settings_info_refresh(lv_timer_t *timer);

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

/* New screen with scrolling disabled: no scrollbars, content locked so swipes
 * always reach the screen-swipe navigation instead of scrolling the content. */
static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

/* ---- Status bar (docs/application.md section 3) ----
 *
 * A small row of state icons shown identically on all five nav-ring screens
 * (below). LVGL objects can't be shared across screens, so each screen gets
 * its own set, built by build_status_bar() and refreshed by
 * update_status_bar() from that screen's own update timer - the same
 * "screen owns its own timer, function is a no-op when that screen isn't
 * active" pattern already used throughout this file.
 *
 * Colors follow the spec's own convention (section 3.1): grey = off/absent,
 * orange = transitioning/warning, green = active at target state, red =
 * active-but-notable (kept per-icon, not applied uniformly - see below).
 * Icons are short colored text labels, not symbol-font glyphs: several of
 * these (GPX-tracking, LoRa) have no good built-in LVGL symbol, and a
 * uniform text style avoids guessing at symbol-font availability for the
 * ones that might (SD/GPS/BT/WiFi) - matches this file's existing
 * text-icon convention (e.g. the watch face's "Zz" snooze indicator). */
#define STATUS_COLOR_GREY   lv_color_hex(0x888888)
#define STATUS_COLOR_ORANGE lv_color_hex(0xFFB300)
#define STATUS_COLOR_GREEN  lv_color_hex(0x00E676)
#define STATUS_COLOR_RED    lv_color_hex(0xFF5252)

typedef struct {
    lv_obj_t *sd;
    lv_obj_t *gps;
    lv_obj_t *gpx;
    lv_obj_t *lora;
    lv_obj_t *bt;
    lv_obj_t *wifi;
    lv_obj_t *batt;
    lv_obj_t *chg;
} status_bar_t;

/* One instance per nav-ring screen, indexed the same way as s_nav_ring
 * below (populated once each screen is built). */
static status_bar_t s_status_bar[5];

/* Row y and left/right-aligned x offsets are chosen from a measured safe-area
 * scan of assets/ui/safe_area_transparent.png (410x502 panel, rounded
 * corners physically clip/hide content there): at y~54 the corner cutout
 * requires roughly x >= 34 from either edge (straight-edge margin is ~16px,
 * but the corner radius is ~90-100px and dominates this close to the top),
 * so every icon in this row is kept clear of x < 40 / x > 410-40 - see
 * docs/application.md's "Abgerundete Ecken beachten" section. y=54 also
 * clears every ring screen's title (TOP_MID, y=18, ends ~y=44) and the GPS
 * screen's GNSS switch row (y=22, ends ~y=48) with a few px to spare. */
#define STATUS_BAR_Y 54

/* LVGL's built-in Montserrat glyph set (FontAwesome-derived, see
 * lv_symbol_def.h) already ships real icons for most of these - reuses the
 * same font the watch face's GPS satellite glyph uses, just at the smaller
 * size this build has compiled in (montserrat_14). No emoji/icon font is
 * bundled in this project, and adding one is a much bigger undertaking
 * (font pipeline + licensing) than this row needs. Two items have no good
 * built-in glyph and stay as short text: GPX-tracking (closest built-in,
 * a generic loop/record glyph, read worse than the word) and LoRa (no
 * antenna/radio symbol exists in this set at all). */
static lv_obj_t *status_icon_create(lv_obj_t *parent, const char *text, lv_align_t align, lv_coord_t x)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, STATUS_COLOR_GREY, 0);
    lv_obj_align(l, align, x, STATUS_BAR_Y);
    return l;
}

/* Builds one status bar instance into `parent` (a screen about to be shown
 * for the first time) and stores its objects in `out` for update_status_bar()
 * to refresh later. Fixed left-to-right order. The battery/charge labels are
 * right-aligned (offset from the right edge) instead of left-positioned,
 * since the battery text's width varies ("--%".."100%") and a fixed left x
 * would let a wide value spill into the right corner's safe-area cutout. */
static void build_status_bar(lv_obj_t *parent, status_bar_t *out)
{
    out->sd   = status_icon_create(parent, LV_SYMBOL_SD_CARD,   LV_ALIGN_TOP_LEFT,  40);
    out->gps  = status_icon_create(parent, LV_SYMBOL_GPS,       LV_ALIGN_TOP_LEFT,  80);
    out->gpx  = status_icon_create(parent, "GPX",               LV_ALIGN_TOP_LEFT,  118);
    out->lora = status_icon_create(parent, "LoRa",              LV_ALIGN_TOP_LEFT,  166);
    out->bt   = status_icon_create(parent, LV_SYMBOL_BLUETOOTH, LV_ALIGN_TOP_LEFT,  214);
    out->wifi = status_icon_create(parent, LV_SYMBOL_WIFI,      LV_ALIGN_TOP_LEFT,  250);
    /* CHG sits further left than its glyph alone needs, to clear the widest
     * battery string ("<icon> 100%") to its right - see update_status_bar(). */
    out->chg  = status_icon_create(parent, LV_SYMBOL_CHARGE,    LV_ALIGN_TOP_RIGHT, -115);
    out->batt = status_icon_create(parent, LV_SYMBOL_BATTERY_EMPTY " --%", LV_ALIGN_TOP_RIGHT, -40);
}

/* Refreshes one status bar instance. Safe to call even if `bar->sd` (or any
 * field) is NULL - i.e. before that screen has been built - callers already
 * guard on "is this screen active" first, matching every other per-screen
 * update function in this file. */
static void update_status_bar(const status_bar_t *bar)
{
    if (!bar->sd) {
        return;
    }

    lv_obj_set_style_text_color(bar->sd, sd_log_available() ? STATUS_COLOR_RED :
                                (twatch_sd_card_seated() ? STATUS_COLOR_ORANGE : STATUS_COLOR_GREY), 0);

    m10q_state_t gps_st = m10q_get_state();
    lv_obj_set_style_text_color(bar->gps,
        (gps_st == M10Q_STATE_FIXED) ? STATUS_COLOR_GREEN :
        (gps_st == M10Q_STATE_ACQUIRING) ? STATUS_COLOR_ORANGE : STATUS_COLOR_GREY, 0);

    /* Reflects gpx_log.c (time-based GPX file logging), not tracking.c's
     * unrelated step-gated pedometer (which stays disabled - see
     * TRACKING_ENABLED in tracking.h). */
    lv_obj_set_style_text_color(bar->gpx, gpx_log_is_active() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);

    /* LoRa has no on/off toggle yet (ALDO3 is hard-wired always-on, see
     * power_mgmt.c) - shows "on" unconditionally until Phase 6 gives it a
     * real state to reflect. */
    lv_obj_set_style_text_color(bar->lora, STATUS_COLOR_GREEN, 0);

    lv_obj_set_style_text_color(bar->bt, ble_debug_is_connected() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);

    /* WiFi has no subsystem behind it at all (see docs/application.md
     * scoping decision) - permanently off/grey. */
    lv_obj_set_style_text_color(bar->wifi, STATUS_COLOR_GREY, 0);

    sensor_cache_t cache;
    sensor_cache_get(&cache);
    if (cache.valid) {
        const char *icon = cache.batt_pct > 87 ? LV_SYMBOL_BATTERY_FULL :
                            cache.batt_pct > 62 ? LV_SYMBOL_BATTERY_3 :
                            cache.batt_pct > 37 ? LV_SYMBOL_BATTERY_2 :
                            cache.batt_pct > 12 ? LV_SYMBOL_BATTERY_1 : LV_SYMBOL_BATTERY_EMPTY;
        char buf[16];
        snprintf(buf, sizeof(buf), "%s %u%%", icon, cache.batt_pct);
        lv_label_set_text(bar->batt, buf);
        lv_obj_set_style_text_color(bar->batt, cache.batt_pct <= 15 ? STATUS_COLOR_RED : lv_color_hex(0xE0E0E0), 0);

        bool charging = (cache.chg_state != AXP2101_CHG_STOP);
        if (charging) {
            lv_obj_clear_flag(bar->chg, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_text_color(bar->chg, STATUS_COLOR_GREEN, 0);
        } else {
            lv_obj_add_flag(bar->chg, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* Red-only night-mode transform. LVGL renders RGB565_SWAPPED (big-endian on
 * the panel): each pixel is 2 bytes, byte0 = MSB = RRRRR GGG, byte1 = GGG BBBBB.
 * Keeping only the red channel zeroes green/blue. Applied in-place; the blit is
 * synchronous, so the buffer is safe to mutate before esp_lcd_panel_draw_bitmap. */
static esp_err_t night_mode_draw_bitmap(lv_display_t *disp, esp_lcd_panel_handle_t panel,
                                        int x_start, int y_start, int x_end, int y_end,
                                        const void *color_map, void *user_ctx)
{
    (void)disp;
    (void)user_ctx;
    if (power_mgmt_is_night_mode()) {
        uint8_t *buf = (uint8_t *)color_map;
        size_t n = (size_t)(x_end - x_start) * (y_end - y_start);
        for (size_t i = 0; i < n; i++) {
            buf[i * 2]     &= 0xF8;   /* keep 5-bit red */
            buf[i * 2 + 1]  = 0x00;   /* drop green/blue */
        }
    }
    return esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end, y_end, color_map);
}

/* On a night-mode change, invalidate the whole screen so every pixel is
 * redrawn through the red-only transform (content flushed before the state
 * flip would otherwise keep its old colors). */
static void night_mode_changed(bool night)
{
    (void)night;
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        lv_obj_invalidate(lv_screen_active());
        esp_lv_adapter_unlock();
    }
}

/* Full-screen redraw (e.g. after waking from sleep, when co5300_blank() left
 * the GRAM black and the adapter's SPI path does not auto-refresh on resume). */
void lvgl_force_redraw(void)
{
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        lv_obj_invalidate(lv_screen_active());
        esp_lv_adapter_unlock();
    }
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

static void watch_face_update(lv_timer_t *timer)
{
    (void)timer;

    /* Fire the alarm when the RTC AF/TF flag is set (also covers a wake from
     * light sleep via the RTC INT line). */
    alarm_check();
    cdtimer_check();

    /* Read the wall-clock time from the RTC (PCF85063A), not the ESP32 system
     * clock, so the display never drifts. The RTC is polled by the background
     * telemetry task; reading the cache keeps I2C off the UI task. */
    pcf85063a_time_t t;
    if (!sensor_cache_get_rtc(&t)) {
        return;
    }

    /* The RTC stores UTC directly; convert to local (process TZ) for the
     * primary display, which stays DST-aware. */
    time_t epoch = pcf85063a_time_to_epoch(&t);
    struct tm lt;
    localtime_r(&epoch, &lt);

    char buf[32];
    if (s_tz_label) {
        char tz[8] = { 0 };
        strftime(tz, sizeof(tz), "%Z", &lt);
        lv_label_set_text(s_tz_label, tz);
    }

    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
    lv_label_set_text(s_time_label, buf);

    /* UTC sub-display: the RTC snapshot is already UTC, no conversion. */
    snprintf(buf, sizeof(buf), "UTC %02d:%02d", t.hour, t.min);
    lv_label_set_text(s_sec_label, buf);

    static const char *wday[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };
    static const char *mon[] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                 "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };
    snprintf(buf, sizeof(buf), "%s  %02d %s %d",
             wday[lt.tm_wday], lt.tm_mday, mon[lt.tm_mon], lt.tm_year + 1900);
    lv_label_set_text(s_date_label, buf);

    sensor_cache_t cache;
    sensor_cache_get(&cache);
    uint8_t pct = cache.batt_pct;
    if (cache.valid && pct <= 100) {
        snprintf(buf, sizeof(buf), "%u%%", pct);
        lv_label_set_text(s_batt_label, buf);
        lv_obj_set_width(s_batt_fill, (lv_coord_t)(140 * pct / 100));
    }

    /* Step count from the BHI260AP (cached in the driver, no I2C here). */
    if (s_steps_label) {
        uint32_t steps = 0;
        if (bhi260ap_get_daily_steps(&steps) == ESP_OK) {
            snprintf(buf, sizeof(buf), "Steps: %lu", (unsigned long)steps);
        } else {
            snprintf(buf, sizeof(buf), "Steps: --");
        }
        lv_label_set_text(s_steps_label, buf);
    }

    /* Satellite status: green = 3D fix, red = on/no fix, grey = off. */
    if (s_gps_icon) {
        m10q_state_t st = m10q_get_state();
        m10q_fix_t fix;
        m10q_get_fix(&fix);
        lv_color_t c;
        if (st == M10Q_STATE_FIXED && fix.valid && fix.fix_3d) {
            c = lv_color_hex(0x00E676);   /* green */
        } else if (st == M10Q_STATE_ACQUIRING || (st == M10Q_STATE_FIXED && !fix.fix_3d)) {
            c = lv_color_hex(0xFF5252);   /* red */
        } else {
            c = lv_color_hex(0x888888);   /* grey */
        }
        lv_obj_set_style_text_color(s_gps_icon, c, 0);
    }

    /* Tracking dot: visible only while a tracking session is active. */
    if (s_track_dot) {
        if (tracking_is_active()) {
            lv_obj_clear_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* Snooze icon: visible while the 10 min snooze timer is pending. */
    if (s_snooze_icon) {
        if (alarm_is_snoozing()) {
            lv_obj_clear_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }

    update_status_bar(&s_status_bar[0]);
}

static void lvgl_build_watch_face(void)
{
    s_watch_screen = lv_screen_active();
    lv_obj_set_style_bg_color(s_watch_screen, lv_color_hex(0x000000), 0);

    build_status_bar(s_watch_screen, &s_status_bar[0]);

    s_date_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_date_label, "");
    lv_obj_set_style_text_font(s_date_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, -100);

    /* GNSS satellite status icon: grey = receiver off, red = on/no fix,
     * green = 3D fix. Uses the built-in symbol font for the satellite glyph
     * (LV_SYMBOL_GPS, 0xF124), which the FreeType fonts do not contain. */
    s_gps_icon = lv_label_create(lv_screen_active());
    lv_label_set_text(s_gps_icon, LV_SYMBOL_GPS);
    lv_obj_set_style_text_font(s_gps_icon, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_gps_icon, lv_color_hex(0x888888), 0);
    lv_obj_align(s_gps_icon, LV_ALIGN_TOP_MID, 0, 24);

    /* Tracking indicator: solid red dot, visible only while a tracking
     * session is active. */
    s_track_dot = lv_obj_create(lv_screen_active());
    lv_obj_set_size(s_track_dot, 12, 12);
    lv_obj_align(s_track_dot, LV_ALIGN_TOP_MID, 40, 24);
    lv_obj_clear_flag(s_track_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_track_dot, lv_color_hex(0xFF2020), 0);
    lv_obj_set_style_radius(s_track_dot, 6, 0);
    lv_obj_set_style_pad_all(s_track_dot, 0, 0);
    lv_obj_add_flag(s_track_dot, LV_OBJ_FLAG_HIDDEN);

    /* Snooze indicator: "Zz" over the alarm icon, hidden unless snoozing. */
    s_snooze_icon = lv_label_create(lv_screen_active());
    lv_label_set_text(s_snooze_icon, "Zz");
    lv_obj_set_style_text_font(s_snooze_icon, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_snooze_icon, lv_color_hex(0xFFD54F), 0);
    lv_obj_align(s_snooze_icon, LV_ALIGN_TOP_LEFT, 70, 40);
    lv_obj_add_flag(s_snooze_icon, LV_OBJ_FLAG_HIDDEN);

    /* Timezone abbreviation (e.g. "CEST"/"CET") above the local time - the
     * process TZ is already set at boot (main/uwatch_main.c) and
     * watch_face_update() already computes localtime_r() for the primary
     * display, so strftime("%Z", ...) gives this for free, no new TZ
     * plumbing needed. */
    s_tz_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_tz_label, "");
    lv_obj_set_style_text_font(s_tz_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_tz_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_tz_label, LV_ALIGN_CENTER, 0, -40);

    s_time_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_time_label, "--:--:--");
    lv_obj_set_style_text_font(s_time_label, s_font_time, 0);
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(0xFFFFFF), 0);
    /* Tighten the monospace cells so the full-width colons don't sprawl. */
    lv_obj_set_style_text_letter_space(s_time_label, -6, 0);
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, -10);

    s_sec_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_sec_label, "UTC --:--");
    lv_obj_set_style_text_font(s_sec_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_sec_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_sec_label, LV_ALIGN_CENTER, 0, 65);

    /* Battery bar. */
    lv_obj_t *bar = lv_obj_create(lv_screen_active());
    lv_obj_set_size(bar, 140, 12);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 185);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x666666), 0);
    lv_obj_set_style_border_width(bar, 2, 0);
    lv_obj_set_style_radius(bar, 6, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);

    s_batt_fill = lv_obj_create(bar);
    lv_obj_set_size(s_batt_fill, 0, 8);
    lv_obj_align(s_batt_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_clear_flag(s_batt_fill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_batt_fill, lv_color_hex(0x00E676), 0);
    lv_obj_set_style_radius(s_batt_fill, 4, 0);
    lv_obj_set_style_pad_all(s_batt_fill, 0, 0);

    s_batt_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_batt_label, "--");
    lv_obj_set_style_text_font(s_batt_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_batt_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_batt_label, LV_ALIGN_CENTER, 0, 208);

    s_steps_label = lv_label_create(lv_screen_active());
    lv_label_set_text(s_steps_label, "Steps: --");
    lv_obj_set_style_text_font(s_steps_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_steps_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_steps_label, LV_ALIGN_CENTER, 0, 150);

    watch_face_update(NULL);
    lv_timer_create(watch_face_update, 1000, NULL);

    /* Menu inactivity timeout (runs forever; no-op on the watch face). */
    lv_timer_create(menu_timeout_cb, 500, NULL);
}

/* Show the boot screen for a few seconds, then the watch face. */
static void boot_to_watch_face(lv_timer_t *timer)
{
    lv_timer_delete(timer);
    lv_obj_clean(lv_screen_active());
    lvgl_build_watch_face();
}

/* ---- BHI260AP status screen ---- */

static const char *activity_name(uint8_t activity)
{
    switch (activity) {
    case BHI260AP_ACTIVITY_STILL: return "still";
    case BHI260AP_ACTIVITY_WALKING: return "walking";
    case BHI260AP_ACTIVITY_RUNNING: return "running";
    case BHI260AP_ACTIVITY_ON_BICYCLE: return "cycling";
    case BHI260AP_ACTIVITY_IN_VEHICLE: return "in vehicle";
    case BHI260AP_ACTIVITY_TILTING: return "tilting";
    default: return "unknown";
    }
}

/* Project an 8-vertex unit cube rotated by the GAMERV quaternion (Q14 fixed
 * point) and move the 12 wireframe edge lines. Orthographic projection, so the
 * cube keeps constant size. */
static void bhi_cube_update(int16_t qx, int16_t qy, int16_t qz, int16_t qw)
{
    if (!s_cube_line[0]) {
        return;
    }
    /* Q14 -> float, renormalize. */
    double x = qx / 16384.0, y = qy / 16384.0, z = qz / 16384.0, w = qw / 16384.0;
    double n = sqrt(x * x + y * y + z * z + w * w);
    if (n < 1e-6) {
        x = 0; y = 0; z = 0; w = 1;
    } else {
        x /= n; y /= n; z /= n; w /= n;
    }

    /* Rotation matrix from the quaternion (column-vector convention). No
     * app-side conjugate here (2026-08-27): the gyro's orientation matrix is
     * now set at bhi260ap_init() so the fusion output is corrected at the
     * source instead of patched here. If that turns out wrong on hardware,
     * restore "x = -x; y = -y; z = -z;" here and revert bhi260ap_init(). */
    double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
    double r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
    double r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);

    const double S = 42.0;   /* half cube size in px */
    const double A = 1.45;   /* axis length as a multiple of the half-size */
    const double cx = 300.0, cy = 135.0;   /* cube centre on screen */

    /* Unit cube corners (8). */
    const double corners[8][3] = {
        { -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 },
        { -1, -1, 1 },  { 1, -1, 1 },  { 1, 1, 1 },  { -1, 1, 1 },
    };
    double px[8], py[8];
    for (int i = 0; i < 8; i++) {
        double vx = corners[i][0], vy = corners[i][1], vz = corners[i][2];
        px[i] = cx + S * (r00 * vx + r01 * vy + r02 * vz);
        py[i] = cy + S * (r10 * vx + r11 * vy + r12 * vz);
    }

    /* 12 cube edges. */
    const int edges[12][2] = {
        { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
        { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
        { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
    };
    for (int e = 0; e < 12; e++) {
        s_cube_pts[e][0].x = (lv_coord_t)px[edges[e][0]];
        s_cube_pts[e][0].y = (lv_coord_t)py[edges[e][0]];
        s_cube_pts[e][1].x = (lv_coord_t)px[edges[e][1]];
        s_cube_pts[e][1].y = (lv_coord_t)py[edges[e][1]];
        lv_line_set_points(s_cube_line[e], s_cube_pts[e], 2);
    }

    /* Origin cross: rotated unit axes from the cube centre. Rows of the
     * rotation matrix are the X/Y/Z axes in world x/y/z (orthographic: the
     * axes use world X=row0, Y=row1, Z=row2 -> screen x/y). */
    const double axes[3][3] = {
        { r00, r01, r02 },   /* X */
        { r10, r11, r12 },   /* Y */
        { r20, r21, r22 },   /* Z */
    };
    for (int a = 0; a < 3; a++) {
        s_axis_pts[a][0].x = (lv_coord_t)cx;
        s_axis_pts[a][0].y = (lv_coord_t)cy;
        s_axis_pts[a][1].x = (lv_coord_t)(cx + A * S * axes[a][0]);
        s_axis_pts[a][1].y = (lv_coord_t)(cy + A * S * axes[a][1]);
        lv_line_set_points(s_axis_line[a], s_axis_pts[a], 2);
    }
}

static void bhi_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_bhi_status_label) {
        return;
    }
    /* Skip when the BHI screen is not active (avoids gesture consumption and
     * needless sensor reads while on another screen). */
    if (lv_screen_active() != s_bhi_screen) {
        return;
    }
    char buf[64];

    bool ready = false;
    bhi260ap_get_status(&ready, NULL);
    lv_label_set_text(s_bhi_status_label, ready ? "BHI260AP: ready" : "BHI260AP: not ready");

    int16_t rx = 0, ry = 0, rz = 0, rw = 0;
    uint16_t racc = 0;
    bhi260ap_get_rotation(&rx, &ry, &rz, &rw, &racc);
    bhi_cube_update(rx, ry, rz, rw);
    snprintf(buf, sizeof(buf), "RV acc: %u", (unsigned)racc);
    lv_label_set_text(s_bhi_rv_acc_label, buf);

    uint8_t activity = BHI260AP_ACTIVITY_UNKNOWN;
    bhi260ap_get_activity(&activity);
    snprintf(buf, sizeof(buf), "Activity: %s", activity_name(activity));
    lv_label_set_text(s_bhi_activity_label, buf);

    /* Per-activity seconds logged today (from daily_log). Compact duration
     * format so second-level precision is actually visible for an activity
     * that just started, instead of showing "0" until a whole minute has
     * passed: Xh Ym once it's been going over an hour, Xm Ys under that,
     * Xs while it's still under a minute. */
    const uint32_t *secs[DAILY_ACT_COUNT];
    static const char *act_names[DAILY_ACT_COUNT] = {
        "Still", "Walk", "Run", "Cycle", "Vehicle", "Tilt", "Other" };
    if (daily_log_get_activity_seconds(secs) == ESP_OK) {
        for (int i = 0; i < DAILY_ACT_COUNT; i++) {
            if (s_daily_act_label[i]) {
                uint32_t s = *secs[i];
                char dur[16];
                if (s >= 3600) {
                    snprintf(dur, sizeof(dur), "%uh %um", (unsigned)(s / 3600), (unsigned)((s % 3600) / 60));
                } else if (s >= 60) {
                    snprintf(dur, sizeof(dur), "%um %us", (unsigned)(s / 60), (unsigned)(s % 60));
                } else {
                    snprintf(dur, sizeof(dur), "%us", (unsigned)s);
                }
                snprintf(buf, sizeof(buf), "%s %s", act_names[i], dur);
                lv_label_set_text(s_daily_act_label[i], buf);
            }
        }
    }

    /* Keep the last gesture shown until a new one fires (otherwise the text
     * would clear on the next 1 s refresh). */
    bool tilt = false, wake = false, glance = false, pickup = false, tdet = false;
    if (bhi260ap_consume_gestures(&tilt, &wake, &glance, &pickup, &tdet) == ESP_OK) {
        snprintf(buf, sizeof(buf), "Gesture: %s%s%s%s%s",
                 tilt ? "wristtilt " : "", wake ? "wake " : "", glance ? "glance " : "",
                 pickup ? "pickup " : "", tdet ? "tiltdet " : "");
        if (strcmp(buf, "Gesture: ") != 0) {
            lv_label_set_text(s_bhi_gesture_label, buf);
        }
    }
}

static lv_obj_t *bhi_text_row(lv_obj_t *parent, const char *text, lv_obj_t **label)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, s_font_small, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
    *label = l;
    return l;
}

static void lvgl_build_bhi_screen(void)
{
    s_bhi_screen = screen_new();
    lv_obj_set_style_bg_color(s_bhi_screen, lv_color_hex(0x002030), 0);

    lv_obj_t *title = lv_label_create(s_bhi_screen);
    lv_label_set_text(title, "BHI260AP");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    /* Status + rotation accuracy as text rows. */
    lv_obj_t *l;
    l = bhi_text_row(s_bhi_screen, "", &s_bhi_status_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 50);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_rv_acc_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 80);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_activity_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 110);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_gesture_label);
    lv_obj_set_style_text_color(l, lv_color_hex(0xFFD54D), 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 140);

    /* 3D orientation wireframe cube from the GAMERV quaternion. */
    const lv_color_t cube_col = lv_color_hex(0x4FC3F7);
    for (int e = 0; e < 12; e++) {
        lv_obj_t *ln = lv_line_create(s_bhi_screen);
        lv_obj_set_style_line_color(ln, cube_col, 0);
        lv_obj_set_style_line_width(ln, 2, 0);
        s_cube_line[e] = ln;
    }

    /* Origin cross (x/y/z pointer) from the cube centre, RGB colors. */
    const lv_color_t axis_col[3] = {
        lv_color_hex(0xFF5252),   /* X red */
        lv_color_hex(0x00E676),   /* Y green */
        lv_color_hex(0x40C4FF),   /* Z blue */
    };
    for (int a = 0; a < 3; a++) {
        lv_obj_t *al = lv_line_create(s_bhi_screen);
        lv_obj_set_style_line_color(al, axis_col[a], 0);
        lv_obj_set_style_line_width(al, 3, 0);
        s_axis_line[a] = al;
    }

    /* Today's per-activity minutes, two compact columns to save height. */
    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        lv_obj_t *al = lv_label_create(s_bhi_screen);
        lv_label_set_text(al, "");
        lv_obj_set_style_text_font(al, s_font_micro, 0);
        lv_obj_set_style_text_color(al, lv_color_hex(0x9ECBE0), 0);
        int col = (i < 4) ? 0 : 1;
        int row = (i < 4) ? i : (i - 4);
        lv_obj_align(al, LV_ALIGN_TOP_LEFT, 44 + col * 190, 300 + row * 26);
        s_daily_act_label[i] = al;
    }

    lv_obj_t *hint = lv_label_create(s_bhi_screen);
    lv_label_set_text(hint, "swipe right: clock   swipe left: NFC");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -90);

    bhi_screen_update(NULL);
    lv_timer_create(bhi_screen_update, 200, NULL);   /* 5 Hz: smooth orientation cube */
}

void lvgl_show_bhi_screen(void)
{
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        ESP_LOGW(TAG, "show bhi: LVGL lock timeout");
        return;
    }
    if (!s_bhi_screen) {
        lvgl_build_bhi_screen();
    }
    lv_scr_load(s_bhi_screen);
    esp_lv_adapter_unlock();
}

/* ---- GPS screen (skyplot + fix info) ----
 * GNSS is powered on demand: the BLDO1 rail is enabled when this screen is
 * opened and disabled when it is left. The always-on VRTC backup rail keeps
 * the receiver's ephemeris/RTC alive, so each power-up is a warm/hot start. */

#define GPS_SKY_RADIUS    100
#define GPS_SKY_CX        205
#define GPS_SKY_CY        190

static lv_obj_t *gps_ring(int radius)
{
    lv_obj_t *arc = lv_arc_create(s_gps_screen);
    lv_obj_set_size(arc, radius * 2, radius * 2);
    lv_obj_set_pos(arc, GPS_SKY_CX - radius, GPS_SKY_CY - radius);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_rotation(arc, 0);
    lv_arc_set_value(arc, 100);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x2A5A2A), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x2A5A2A), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_INDICATOR);
    /* Hide the arc knob: with a full 0-360 range and value 100, LVGL draws the
     * default (blue) knob at the 360 deg point = 3 o'clock. Without this,
     * each ring contributes a blue dot in a horizontal line on the east. */
    lv_obj_set_style_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    return arc;
}

/* Convert satellite azimuth/elevation to skyplot pixel coords. */
static void gps_sat_xy(int az, int el, int *x, int *y)
{
    double r = (double)GPS_SKY_RADIUS * (90.0 - el) / 90.0;
    double a = (double)az * M_PI / 180.0;
    *x = GPS_SKY_CX + (int)(r * sin(a) + 0.5);
    *y = GPS_SKY_CY - (int)(r * cos(a) + 0.5);
}

static lv_color_t gps_snr_color(int snr)
{
    if (snr < 25) return lv_color_hex(0x888888);
    if (snr < 35) return lv_color_hex(0xFFD54D);
    return lv_color_hex(0x3DD68A);
}

/* Decimal degrees -> DMS. The doc's example uses "48°07'24"N", but the
 * baked bitmap fonts here (cascadia_*.c) only cover ASCII 0x20-0x7E - no
 * degree sign (U+00B0) - and the source TTF isn't in the repo to
 * regenerate them with a wider range (see cascadia_22.c's header comment
 * for the original `lv_font_conv` invocation). Using the plain-ASCII
 * "D M S" letter notation instead (e.g. "48d07m24sN"), a common fallback
 * for the same reason on other text-only GPS displays. */
static void format_dms(double deg, bool is_lat, char *out, size_t outlen)
{
    char dir = is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
    deg = fabs(deg);
    int d = (int)deg;
    double m_full = (deg - d) * 60.0;
    int m = (int)m_full;
    int s = (int)((m_full - m) * 60.0 + 0.5);
    snprintf(out, outlen, "%dd%02dm%02ds%c", d, m, s, dir);
}

static void gps_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_gps_status_label) {
        return;
    }
    /* Skip all work (fix reads + widget updates) when the GPS screen is not
     * the active screen. The timer fires every second forever. */
    if (lv_screen_active() != s_gps_screen) {
        return;
    }
    m10q_fix_t fix;
    m10q_get_fix(&fix);
    m10q_state_t st = m10q_get_state();
    char buf[96];

    /* While acquiring, keep the watch awake (no auto-sleep) so the GNSS rail
     * stays powered. Once a fix is obtained, stop reporting activity: the
     * adapter's idle timeout then auto-sleeps the watch ~5 s after the fix,
     * powering BLDO1 off (VRTC backup keeps ephemeris for the next session). */
    if (st != M10Q_STATE_FIXED || !fix.valid) {
        esp_lv_adapter_report_activity();
    }
    if (st == M10Q_STATE_FIXED && fix.valid) {
        snprintf(buf, sizeof(buf), "Fix: %d sats  acc %um  HDOP %.1f",
                 (int)fix.sat_count, (unsigned)fix.hacc_m, fix.hdop / 10.0);
        lv_label_set_text(s_gps_status_label, buf);

        char dms_lat[16], dms_lon[16];
        format_dms(fix.lat, true, dms_lat, sizeof(dms_lat));
        format_dms(fix.lon, false, dms_lon, sizeof(dms_lon));
        snprintf(buf, sizeof(buf), "%.5f, %.5f  %.0f m\n%s  %s",
                 fix.lat, fix.lon, fix.alt_m, dms_lat, dms_lon);
        lv_label_set_text(s_gps_pos_label, buf);

        snprintf(buf, sizeof(buf), "Speed: %u km/h  course %u deg",
                 (unsigned)fix.speed_kmh, (unsigned)fix.course_deg);
        lv_label_set_text(s_gps_speed_label, buf);

        snprintf(buf, sizeof(buf), "%02u:%02u:%02u UTC  (%u in view)",
                 (unsigned)fix.hour, (unsigned)fix.minute, (unsigned)fix.second,
                 (unsigned)fix.sat_in_view);
        lv_label_set_text(s_gps_sats_label, buf);
    } else if (st == M10Q_STATE_ACQUIRING) {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint32_t elapsed = (now - s_gps_acq_start_ms) / 1000;
        snprintf(buf, sizeof(buf), "Acquiring... (%lus)", (unsigned long)elapsed);
        lv_label_set_text(s_gps_status_label, buf);
        lv_label_set_text(s_gps_pos_label, "");
        lv_label_set_text(s_gps_speed_label, "");
        lv_label_set_text(s_gps_sats_label, "");
    } else {
        lv_label_set_text(s_gps_status_label, "GNSS off");
        lv_label_set_text(s_gps_pos_label, "");
        lv_label_set_text(s_gps_speed_label, "");
        lv_label_set_text(s_gps_sats_label, "");
    }

    /* Diagnostics line: receiver state, RX traffic, TTFF. */
    if (s_gps_diag_label) {
        uint32_t rx = 0, lines = 0;
        m10q_get_dbg(&rx, &lines);
        (void)lines;
        m10q_stats_t stats;
        if (m10q_get_stats(&stats) == ESP_OK) {
            snprintf(buf, sizeof(buf),
                     "st=%d rx=%lu gsv=%lu fix=%lu ttf=%lums",
                     (int)m10q_get_state(), (unsigned long)rx,
                     (unsigned long)m10q_get_gsv_count(),
                     (unsigned long)stats.total_fixes,
                     (unsigned long)stats.ttf_avg_ms);
        } else {
            snprintf(buf, sizeof(buf),
                     "st=%d rx=%lu gsv=%lu",
                     (int)m10q_get_state(), (unsigned long)rx,
                     (unsigned long)m10q_get_gsv_count());
        }
        lv_label_set_text(s_gps_diag_label, buf);
    }

    /* Satellite dots. Satellites with no elevation/azimuth (receiver not yet
     * resolved) or no SNR are hidden; otherwise they'd cluster at the skyplot
     * centre as a static blob. */
    for (int i = 0; i < M10Q_MAX_SATS; i++) {
        if (!s_gps_dots[i]) {
            break;
        }
        if (i < (int)fix.sat_in_view && fix.sats[i].snr_db >= 0 &&
                (fix.sats[i].elevation_deg > 0 || fix.sats[i].azimuth_deg > 0)) {
            m10q_sat_t *s = &fix.sats[i];
            int x, y;
            gps_sat_xy(s->azimuth_deg, s->elevation_deg, &x, &y);
            lv_obj_set_pos(s_gps_dots[i], x - 7, y - 7);
            lv_obj_set_style_bg_color(s_gps_dots[i], gps_snr_color(s->snr_db), 0);
            lv_obj_set_style_border_width(s_gps_dots[i], s->used ? 0 : 2, 0);
            lv_obj_set_style_border_color(s_gps_dots[i], lv_color_hex(0xAAAAAA), 0);
            lv_obj_clear_flag(s_gps_dots[i], LV_OBJ_FLAG_HIDDEN);
            char prn[8];
            snprintf(prn, sizeof(prn), "%u", (unsigned)s->prn);
            lv_label_set_text_fmt(lv_obj_get_child(s_gps_dots[i], 0), "%s", prn);
        } else {
            lv_obj_add_flag(s_gps_dots[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* GNSS on/off switch state. */
    if (s_gps_pwr_switch) {
        bool powered = s_gps_powered && m10q_get_state() != M10Q_STATE_OFF;
        if (lv_obj_has_state(s_gps_pwr_switch, LV_STATE_CHECKED) != powered) {
            if (powered) {
                lv_obj_add_state(s_gps_pwr_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(s_gps_pwr_switch, LV_STATE_CHECKED);
            }
        }
    }

    /* GPX recording state + Start/Stop button (main/gpx_log.c) - a
     * time-based (every 30s) SD file logger, unrelated to tracking.c's
     * step-gated pedometer (that stays disabled, see TRACKING_ENABLED). */
    if (s_gps_track_label && s_gps_track_btn) {
        bool active = gpx_log_is_active();
        if (active) {
            snprintf(buf, sizeof(buf), "Recording: %lu pts",
                     (unsigned long)gpx_log_point_count());
        } else {
            snprintf(buf, sizeof(buf), "GPX: off");
        }
        lv_label_set_text(s_gps_track_label, buf);
        lv_obj_t *bl = lv_obj_get_child(s_gps_track_btn, 0);
        if (bl) {
            lv_label_set_text(bl, active ? "Stop" : "Start");
        }
        lv_obj_set_style_bg_color(s_gps_track_btn,
                                  active ? lv_color_hex(0x8B0000) : lv_color_hex(0x1B5E20), 0);
    }

    update_status_bar(&s_status_bar[1]);
}

static void lvgl_build_gps_screen(void)
{
    s_gps_screen = screen_new();
    lv_obj_set_style_bg_color(s_gps_screen, lv_color_hex(0x102010), 0);

    build_status_bar(s_gps_screen, &s_status_bar[1]);

    lv_obj_t *title = lv_label_create(s_gps_screen);
    lv_label_set_text(title, "GPS");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    /* GNSS on/off switch. Explicit larger size (matches the Settings
     * screens' switches, AGENT.md "Display density & UI sizing") - this
     * screen's skyplot + stacked telemetry rows already use the full
     * panel height with no slack, so unlike Settings this pass is limited
     * to touch targets and the few short standalone labels (title, this
     * one); the info-row stack and the diagnostics line stay at their
     * current sizes - promoting them would need reflowing/shrinking the
     * skyplot, a bigger change than a sizing pass. */
    lv_obj_t *pwr_lbl = lv_label_create(s_gps_screen);
    lv_label_set_text(pwr_lbl, "GNSS");
    lv_obj_set_style_text_font(pwr_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(pwr_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(pwr_lbl, LV_ALIGN_TOP_LEFT, 90, 22);
    s_gps_pwr_switch = lv_switch_create(s_gps_screen);
    lv_obj_set_size(s_gps_pwr_switch, 66, 36);
    lv_obj_align(s_gps_pwr_switch, LV_ALIGN_TOP_RIGHT, -90, 14);
    lv_obj_add_event_cb(s_gps_pwr_switch, gps_pwr_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Skyplot: horizon ring + elevation rings. */
    gps_ring(GPS_SKY_RADIUS);
    gps_ring(GPS_SKY_RADIUS * 2 / 3);
    gps_ring(GPS_SKY_RADIUS / 3);

    /* Cardinal labels. */
    lv_obj_t *n = lv_label_create(s_gps_screen);
    lv_label_set_text(n, "N");
    lv_obj_set_style_text_font(n, s_font_small, 0);
    lv_obj_set_style_text_color(n, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(n, GPS_SKY_CX - 6, GPS_SKY_CY - GPS_SKY_RADIUS - 18);
    lv_obj_t *e = lv_label_create(s_gps_screen);
    lv_label_set_text(e, "E");
    lv_obj_set_style_text_font(e, s_font_small, 0);
    lv_obj_set_style_text_color(e, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(e, GPS_SKY_CX + GPS_SKY_RADIUS - 4, GPS_SKY_CY - 14);
    lv_obj_t *s = lv_label_create(s_gps_screen);
    lv_label_set_text(s, "S");
    lv_obj_set_style_text_font(s, s_font_small, 0);
    lv_obj_set_style_text_color(s, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(s, GPS_SKY_CX - 6, GPS_SKY_CY + GPS_SKY_RADIUS + 2);
    lv_obj_t *w = lv_label_create(s_gps_screen);
    lv_label_set_text(w, "W");
    lv_obj_set_style_text_font(w, s_font_small, 0);
    lv_obj_set_style_text_color(w, lv_color_hex(0x66AA66), 0);
    lv_obj_set_pos(w, GPS_SKY_CX - GPS_SKY_RADIUS - 12, GPS_SKY_CY - 14);

    /* Satellite dots: 14 px circles with PRN label inside. */
    for (int i = 0; i < M10Q_MAX_SATS; i++) {
        lv_obj_t *dot = lv_obj_create(s_gps_screen);
        lv_obj_set_size(dot, 14, 14);
        lv_obj_set_style_radius(dot, 7, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(0x888888), 0);
        lv_obj_set_style_border_width(dot, 2, 0);
        lv_obj_set_style_border_color(dot, lv_color_hex(0xAAAAAA), 0);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t *prn = lv_label_create(dot);
        lv_label_set_text(prn, "");
        lv_obj_set_style_text_font(prn, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(prn, lv_color_hex(0x000000), 0);
        lv_obj_center(prn);
        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
        s_gps_dots[i] = dot;
    }

    /* Info rows below the skyplot. */
    s_gps_status_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_status_label, "");
    lv_obj_set_style_text_font(s_gps_status_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_status_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_gps_status_label, LV_ALIGN_TOP_MID, 0, 306);

    s_gps_pos_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_pos_label, "");
    lv_obj_set_style_text_font(s_gps_pos_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_pos_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_gps_pos_label, LV_ALIGN_TOP_MID, 0, 330);

    s_gps_speed_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_speed_label, "");
    lv_obj_set_style_text_font(s_gps_speed_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_speed_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_gps_speed_label, LV_ALIGN_TOP_MID, 0, 380);

    s_gps_sats_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_sats_label, "");
    lv_obj_set_style_text_font(s_gps_sats_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_sats_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_gps_sats_label, LV_ALIGN_TOP_MID, 0, 406);

    s_gps_diag_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_diag_label, "");
    lv_obj_set_style_text_font(s_gps_diag_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_gps_diag_label, lv_color_hex(0x8A9BA8), 0);
    lv_obj_align(s_gps_diag_label, LV_ALIGN_TOP_MID, 0, 56);

    /* GPX recording state + start/stop (main/gpx_log.c). */
    s_gps_track_label = lv_label_create(s_gps_screen);
    lv_label_set_text(s_gps_track_label, "");
    lv_obj_set_style_text_font(s_gps_track_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_gps_track_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_gps_track_label, LV_ALIGN_TOP_MID, 0, 434);

    s_gps_track_btn = lv_btn_create(s_gps_screen);
    lv_obj_set_size(s_gps_track_btn, 120, 34);
    lv_obj_align(s_gps_track_btn, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_add_event_cb(s_gps_track_btn, gps_track_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_lbl = lv_label_create(s_gps_track_btn);
    lv_label_set_text(btn_lbl, "Start");
    lv_obj_set_style_text_font(btn_lbl, s_font_small, 0);
    lv_obj_center(btn_lbl);

    gps_screen_update(NULL);
    lv_timer_create(gps_screen_update, 1000, NULL);
    if (s_gps_ctrl_task == NULL) {
        xTaskCreate(gps_ctrl_task, "gps_ctrl", 8192, NULL,
                    ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY, &s_gps_ctrl_task);
    }
}

/* Mesh screen: last few received Meshtastic text messages (RAM ring buffer,
 * mesh_log.c). The background listener task runs always-on, independent of
 * whether this screen is open; this timer only pulls a snapshot to display. */
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
     * (which only stores a monotonic esp_timer_get_time() stamp) just for
     * this row's display. */
    pcf85063a_time_t rtc;
    bool have_rtc = sensor_cache_get_rtc(&rtc);
    time_t now_epoch = have_rtc ? pcf85063a_time_to_epoch(&rtc) : 0;

    int64_t now_us = esp_timer_get_time();
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

    update_status_bar(&s_status_bar[2]);
}

/* Preset buttons are visually complete per docs/application.md 7.2 point 5
 * but functionally inert: LoRa sending is out of scope until a
 * separately-scoped TX-stack project ships (see the Phase 1 planning
 * notes) - tapping one is a no-op, not a stub that pretends to send. */
static void mesh_preset_btn_cb(lv_event_t *e)
{
    (void)e;
}

/* Fling-vs-scroll gesture on the message-row container: a fast, long
 * upward drag switches to the Node-Overview screen; a slower/shorter one
 * just scrolls the list via LVGL's own native drag-scroll on this same
 * container (this handler observes, it doesn't consume the event, so
 * native scrolling always happens independently - a fling may visibly
 * nudge-scroll the list before the screen switches, an acceptable minor
 * side effect). See docs/application.md 7.2 point 4 for the exact
 * "speed/distance threshold" wording this implements. */
#define MESH_FLING_MIN_DIST_PX 60
#define MESH_FLING_MAX_MS      400
#define MESH_FLING_MIN_VEL     0.5f   /* px/ms */

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

    build_status_bar(s_mesh_screen, &s_status_bar[2]);

    lv_obj_t *title = lv_label_create(s_mesh_screen);
    lv_label_set_text(title, "MESH");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_mesh_conn_label = lv_label_create(s_mesh_screen);
    lv_label_set_text(s_mesh_conn_label, "");
    lv_obj_set_style_text_font(s_mesh_conn_label, s_font_micro, 0);
    lv_obj_set_style_text_color(s_mesh_conn_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_mesh_conn_label, LV_ALIGN_TOP_MID, 0, 46);

    s_mesh_empty_label = lv_label_create(s_mesh_screen);
    lv_label_set_text(s_mesh_empty_label, "No messages yet");
    lv_obj_set_style_text_font(s_mesh_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_mesh_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_mesh_empty_label, LV_ALIGN_CENTER, 0, 0);

    /* Scrollable row container - both future-proofs the list if
     * MESH_LOG_COUNT ever grows past 8 (little practical effect today) and
     * hosts the fling-vs-scroll gesture above. */
    s_mesh_list_cont = lv_obj_create(s_mesh_screen);
    lv_obj_set_size(s_mesh_list_cont, 380, 250);
    lv_obj_align(s_mesh_list_cont, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_flex_flow(s_mesh_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_mesh_list_cont, 4, 0);
    lv_obj_set_style_bg_opa(s_mesh_list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_mesh_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_mesh_list_cont, 2, 0);
    lv_obj_add_event_cb(s_mesh_list_cont, mesh_gesture_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_mesh_list_cont, mesh_gesture_cb, LV_EVENT_RELEASED, NULL);

    /* One single-line label per ring-buffer slot, newest first, stacked top
     * to bottom. Fixed size + CLIP long-mode so an over-length message is
     * silently cropped within its own row instead of wrapping/spilling into
     * the next one. LV_LABEL_LONG_DOT was tried first but hangs the software
     * render thread forever (confirmed live via JTAG/GDB: CPU1 gets stuck
     * permanently inside lv_draw_label_iterate_characters, looping the same
     * line without progress) when a label is shorter than its content height
     * and contains an explicit "\n" - the DOT ellipsis-placement math doesn't
     * handle that combination. A 26px row (one cascadia_18 line, no forced
     * "\n" in the formatted text - see mesh_screen_update()) avoids both the
     * DOT hang and the two-line-in-a-one-line-box overlap that followed it. */
    for (int i = 0; i < MESH_LOG_COUNT; i++) {
        lv_obj_t *l = lv_label_create(s_mesh_list_cont);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_micro, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
        lv_obj_set_size(l, 370, 26);
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
        lv_obj_set_size(btn, 180, 36);
        lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 15 + col * 195, 330 + row * 44);
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

    int64_t now_us = esp_timer_get_time();
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
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, node_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_node_screen);
    lv_label_set_text(title, "NODES");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_node_empty_label = lv_label_create(s_node_screen);
    lv_label_set_text(s_node_empty_label, "No nodes seen yet");
    lv_obj_set_style_text_font(s_node_empty_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_node_empty_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_node_empty_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *list_cont = lv_obj_create(s_node_screen);
    lv_obj_set_size(list_cont, 380, 400);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 60);
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
    /* Load first, refresh after: node_screen_update() no-ops unless
     * s_node_screen is already the active screen (same guard every other
     * screen's update function uses), so calling it before the load here
     * would silently do nothing and leave stale/empty rows up to 1s until
     * the periodic timer corrects it. */
    lv_scr_load(s_node_screen);
    node_screen_update(NULL);
}

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
    if (!s_mesh_screen) {
        lvgl_build_mesh_screen();
    }
    lv_scr_load(s_mesh_screen);
    mesh_screen_update(NULL);
    esp_lv_adapter_request_wake();
    /* Give the freshly-shown screen its own inactivity window - otherwise
     * menu_timeout_cb (last touch could be minutes old) bounces straight
     * back to the watch face on its next 500 ms tick. */
    s_last_touch_tick = lv_tick_get();
    esp_lv_adapter_unlock();
}

/* ---- Settings screen (docs/application.md section 9) ----
 * Category list (nav-ring slot 3) + 5 local sub-pages: Zeit & Zeitzone,
 * Display, Peripherie, Ton & Vibration, Info. "Presets verwalten" and
 * "Ultra-Sparmodus" are deferred (see Phase 5 plan) - LoRa preset text is
 * still editable, just via the `presetset` debug command instead of a
 * Settings category (this project deliberately has no on-watch text
 * keyboard). Everything here applies immediately except the display
 * timeout, which is flagged as taking effect after a restart. */
static void settings_screen_status_bar_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_settings_screen) {
        return;
    }
    update_status_bar(&s_status_bar[3]);
}

static void settings_sub_swipe_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_active();
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    static lv_point_t start;
    static bool active;

    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        start = p;
        active = true;
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_RELEASED || !active) {
        return;
    }
    active = false;
    int dx = p.x - start.x;
    int dy = p.y - start.y;
    if (dy < SWIPE_DIST || abs(dy) <= abs(dx)) {
        return;   /* only a top-to-bottom swipe counts as "back" */
    }
    void (*back_cb)(lv_event_t *) = (void (*)(lv_event_t *))lv_event_get_user_data(e);
    back_cb(e);
}

/* Category row -> sub-page dispatch, shared by all 5 rows (index in
 * user_data). Each sub-page is lazily built on first visit, same pattern as
 * every other local sub-screen in this file. */
static void settings_row_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    switch (idx) {
    case 0:
        if (!s_set_tz_screen) { lvgl_build_settings_tz_screen(); }
        lv_scr_load(s_set_tz_screen);
        settings_tz_refresh();
        break;
    case 1:
        lvgl_show_settings_disp();
        break;
    case 2:
        if (!s_set_periph_screen) { lvgl_build_settings_periph_screen(); }
        lv_scr_load(s_set_periph_screen);
        settings_periph_refresh(NULL);
        break;
    case 3:
        if (!s_set_sound_screen) { lvgl_build_settings_sound_screen(); }
        lv_scr_load(s_set_sound_screen);
        settings_sound_refresh();
        break;
    case 4:
        if (!s_set_info_screen) { lvgl_build_settings_info_screen(); }
        lv_scr_load(s_set_info_screen);
        settings_info_refresh(NULL);
        break;
    default:
        break;
    }
}

static void settings_back_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_settings_screen);
    settings_screen_status_bar_update(NULL);
}

static void lvgl_build_settings_screen(void)
{
    s_settings_screen = screen_new();
    lv_obj_set_style_bg_color(s_settings_screen, lv_color_hex(0x000000), 0);

    build_status_bar(s_settings_screen, &s_status_bar[3]);

    /* High-DPI sizing (see AGENT.md "Display density & UI sizing"): title
     * and row labels use cascadia_36 (s_font_sec) instead of the old
     * cascadia_22 - at ~315 PPI, 22px body text reads under 2mm tall, too
     * small for comfortable reading/tapping. Rows are 64px tall (matches
     * the alarm-edit screen's stepper-button precedent for a
     * fingertip-sized touch target) and the list uses most of the safe
     * width instead of leaving margin on both sides. */
    lv_obj_t *title = lv_label_create(s_settings_screen);
    lv_label_set_text(title, "SETTINGS");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    static const char *cat_names[5] = {
        "Zeit & Zeitzone", "Display", "Peripherie", "Ton & Vibration", "Info",
    };

    lv_obj_t *list_cont = lv_obj_create(s_settings_screen);
    lv_obj_set_size(list_cont, 386, 380);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 96);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 10, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 0, 0);

    for (int i = 0; i < 5; i++) {
        lv_obj_t *row = lv_obj_create(list_cont);
        lv_obj_set_size(row, LV_PCT(100), 64);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x202020), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 10, 0);
        lv_obj_set_style_pad_all(row, 14, 0);
        lv_obj_add_event_cb(row, settings_row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, cat_names[i]);
        lv_obj_set_style_text_font(l, s_font_sec, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
    }

    update_status_bar(&s_status_bar[3]);
    lv_timer_create(settings_screen_status_bar_update, 1000, NULL);
}

/* ---- Zeit & Zeitzone (info-only: no TZ auto-detect infra exists, see
 * Phase 5 plan) ---- */

static void settings_tz_refresh(void)
{
    if (!s_set_tz_abbrev_label) {
        return;
    }
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char abbrev[16];
    strftime(abbrev, sizeof(abbrev), "%Z", &tmv);
    char buf[32];
    snprintf(buf, sizeof(buf), "Zone: %s", abbrev);
    lv_label_set_text(s_set_tz_abbrev_label, buf);

    /* struct tm on this toolchain has no tm_gmtoff (picolibc) and there's
     * no timegm() either - derive the UTC offset from the local vs. UTC
     * wall-clock fields directly instead, day-wrap handled via tm_yday
     * (works for any offset in -24h..+24h, which covers every real zone). */
    struct tm utcv;
    gmtime_r(&now, &utcv);
    long local_secs = tmv.tm_hour * 3600L + tmv.tm_min * 60L + tmv.tm_sec;
    long utc_secs = utcv.tm_hour * 3600L + utcv.tm_min * 60L + utcv.tm_sec;
    long day_diff = tmv.tm_yday - utcv.tm_yday;
    if (tmv.tm_year != utcv.tm_year) {
        day_diff = (tmv.tm_year > utcv.tm_year) ? 1 : -1;
    } else if (day_diff > 1) {
        day_diff = -1;
    } else if (day_diff < -1) {
        day_diff = 1;
    }
    long off_s = (local_secs - utc_secs) + day_diff * 86400L;
    snprintf(buf, sizeof(buf), "UTC%+03ld:%02ld", off_s / 3600, labs(off_s % 3600) / 60);
    lv_label_set_text(s_set_tz_offset_label, buf);
}

static void lvgl_build_settings_tz_screen(void)
{
    s_set_tz_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_tz_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_tz_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_tz_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_tz_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    /* Title sits below the back button (not centered at the very top like
     * the category list) so a wide title never overlaps it - see AGENT.md
     * "Display density & UI sizing". */
    lv_obj_t *title = lv_label_create(s_set_tz_screen);
    lv_label_set_text(title, "TIME & TIMEZONE");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    s_set_tz_abbrev_label = lv_label_create(s_set_tz_screen);
    lv_obj_set_style_text_font(s_set_tz_abbrev_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_set_tz_abbrev_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_tz_abbrev_label, LV_ALIGN_TOP_MID, 0, 252);

    s_set_tz_offset_label = lv_label_create(s_set_tz_screen);
    lv_obj_set_style_text_font(s_set_tz_offset_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_set_tz_offset_label, lv_color_hex(0x9E9E9E), 0);
    lv_obj_align(s_set_tz_offset_label, LV_ALIGN_TOP_MID, 0, 312);

    settings_tz_refresh();
}

/* ---- Display: timeout (persists, takes effect after restart) + brightness
 * (applies immediately) ---- */

static const uint32_t s_disp_timeout_opts[] = { 5, 10, 15, 20, 30, 60 };
#define DISP_TIMEOUT_OPT_COUNT (sizeof(s_disp_timeout_opts) / sizeof(s_disp_timeout_opts[0]))
static const uint8_t s_disp_bright_opts[] = { 64, 128, 192, 255 };
#define DISP_BRIGHT_OPT_COUNT (sizeof(s_disp_bright_opts) / sizeof(s_disp_bright_opts[0]))

static void settings_disp_refresh(void)
{
    if (!s_set_disp_timeout_label) {
        return;
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "Timeout: %us (after restart)",
             (unsigned)power_mgmt_get_display_timeout_s());
    lv_label_set_text(s_set_disp_timeout_label, buf);

    uint8_t level = power_mgmt_get_brightness();
    snprintf(buf, sizeof(buf), "Brightness: %u%%", (unsigned)(level * 100 / 255));
    lv_label_set_text(s_set_disp_bright_label, buf);
}

static void settings_disp_timeout_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint32_t cur = power_mgmt_get_display_timeout_s();
    int idx = 0;
    for (size_t i = 0; i < DISP_TIMEOUT_OPT_COUNT; i++) {
        if (s_disp_timeout_opts[i] == cur) { idx = (int)i; break; }
    }
    idx = (idx + (int)DISP_TIMEOUT_OPT_COUNT + delta) % (int)DISP_TIMEOUT_OPT_COUNT;
    power_mgmt_set_display_timeout_s(s_disp_timeout_opts[idx]);
    settings_disp_refresh();
}

static void settings_disp_bright_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint8_t cur = power_mgmt_get_brightness();
    int idx = 0;
    for (size_t i = 0; i < DISP_BRIGHT_OPT_COUNT; i++) {
        if (s_disp_bright_opts[i] == cur) { idx = (int)i; break; }
    }
    idx = (idx + (int)DISP_BRIGHT_OPT_COUNT + delta) % (int)DISP_BRIGHT_OPT_COUNT;
    power_mgmt_set_brightness(s_disp_bright_opts[idx]);
    settings_disp_refresh();
}

static void lvgl_build_settings_disp_screen(void)
{
    s_set_disp_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_disp_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_disp_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_disp_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_disp_screen);
    lv_label_set_text(title, "DISPLAY");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    s_set_disp_timeout_label = lv_label_create(s_set_disp_screen);
    lv_obj_set_style_text_font(s_set_disp_timeout_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_disp_timeout_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_disp_timeout_label, LV_ALIGN_TOP_MID, 0, 164);

    lv_obj_t *t_minus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(t_minus, 165, 72);
    lv_obj_align(t_minus, LV_ALIGN_TOP_LEFT, 20, 204);
    lv_obj_t *tml = lv_label_create(t_minus);
    lv_label_set_text(tml, "-");
    lv_obj_set_style_text_font(tml, s_font_sec, 0);
    lv_obj_center(tml);
    lv_obj_add_event_cb(t_minus, settings_disp_timeout_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *t_plus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(t_plus, 165, 72);
    lv_obj_align(t_plus, LV_ALIGN_TOP_RIGHT, -20, 204);
    lv_obj_t *tpl = lv_label_create(t_plus);
    lv_label_set_text(tpl, "+");
    lv_obj_set_style_text_font(tpl, s_font_sec, 0);
    lv_obj_center(tpl);
    lv_obj_add_event_cb(t_plus, settings_disp_timeout_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    s_set_disp_bright_label = lv_label_create(s_set_disp_screen);
    lv_obj_set_style_text_font(s_set_disp_bright_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_disp_bright_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_disp_bright_label, LV_ALIGN_TOP_MID, 0, 320);

    lv_obj_t *b_minus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(b_minus, 165, 72);
    lv_obj_align(b_minus, LV_ALIGN_TOP_LEFT, 20, 360);
    lv_obj_t *bml = lv_label_create(b_minus);
    lv_label_set_text(bml, "-");
    lv_obj_set_style_text_font(bml, s_font_sec, 0);
    lv_obj_center(bml);
    lv_obj_add_event_cb(b_minus, settings_disp_bright_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *b_plus = lv_button_create(s_set_disp_screen);
    lv_obj_set_size(b_plus, 165, 72);
    lv_obj_align(b_plus, LV_ALIGN_TOP_RIGHT, -20, 360);
    lv_obj_t *bpl = lv_label_create(b_plus);
    lv_label_set_text(bpl, "+");
    lv_obj_set_style_text_font(bpl, s_font_sec, 0);
    lv_obj_center(bpl);
    lv_obj_add_event_cb(b_plus, settings_disp_bright_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    settings_disp_refresh();
}

/* Reachable both from the Settings category list and via tap-and-hold on
 * the watch face (docs/application.md section 9.3). */
static void lvgl_show_settings_disp(void)
{
    if (!s_set_disp_screen) {
        lvgl_build_settings_disp_screen();
    }
    lv_scr_load(s_set_disp_screen);
    settings_disp_refresh();
    s_last_touch_tick = lv_tick_get();
}

/* ---- Peripherie: GPS/Bluetooth real toggles, LoRa/WiFi shown disabled
 * (no toggle capability exists for either yet, see Phase 5 plan) ---- */

static void settings_gps_switch_cb(lv_event_t *e)
{
    (void)e;
    lvgl_gps_set_enabled(lv_obj_has_state(s_set_periph_gps_switch, LV_STATE_CHECKED));
}

static void settings_bt_switch_cb(lv_event_t *e)
{
    (void)e;
    ble_debug_set_advertising(lv_obj_has_state(s_set_periph_bt_switch, LV_STATE_CHECKED));
}

/* Mirrors the GPS screen's own switch state formula (s_gps_powered &&
 * m10q_get_state() != M10Q_STATE_OFF, lvgl_app.c's gps_screen_update()) so
 * the two switches never disagree about what "on" means. */
static void settings_periph_refresh(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_set_periph_screen) {
        return;
    }
    bool gps_on = s_gps_powered && m10q_get_state() != M10Q_STATE_OFF;
    if (lv_obj_has_state(s_set_periph_gps_switch, LV_STATE_CHECKED) != gps_on) {
        if (gps_on) { lv_obj_add_state(s_set_periph_gps_switch, LV_STATE_CHECKED); }
        else { lv_obj_clear_state(s_set_periph_gps_switch, LV_STATE_CHECKED); }
    }
    bool bt_on = ble_debug_is_advertising();
    if (lv_obj_has_state(s_set_periph_bt_switch, LV_STATE_CHECKED) != bt_on) {
        if (bt_on) { lv_obj_add_state(s_set_periph_bt_switch, LV_STATE_CHECKED); }
        else { lv_obj_clear_state(s_set_periph_bt_switch, LV_STATE_CHECKED); }
    }
}

static void lvgl_build_settings_periph_screen(void)
{
    s_set_periph_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_periph_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_periph_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_periph_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_periph_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_periph_screen);
    lv_label_set_text(title, "PERIPHERIE");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    /* Each toggle sits in its own dark rounded card (same treatment as the
     * category list's rows) instead of floating label+switch pairs on bare
     * black - fills the panel's width properly and gives every row a
     * fingertip-sized tap target, not just the switch itself. */
    lv_obj_t *gps_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(gps_row, 386, 58);
    lv_obj_align(gps_row, LV_ALIGN_TOP_MID, 0, 152);
    lv_obj_clear_flag(gps_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(gps_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(gps_row, 0, 0);
    lv_obj_set_style_radius(gps_row, 10, 0);
    lv_obj_set_style_pad_all(gps_row, 14, 0);
    lv_obj_t *gps_lbl = lv_label_create(gps_row);
    lv_label_set_text(gps_lbl, "GPS");
    lv_obj_set_style_text_font(gps_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(gps_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(gps_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_periph_gps_switch = lv_switch_create(gps_row);
    lv_obj_set_size(s_set_periph_gps_switch, 66, 36);
    lv_obj_align(s_set_periph_gps_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_periph_gps_switch, settings_gps_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *bt_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(bt_row, 386, 58);
    lv_obj_align(bt_row, LV_ALIGN_TOP_MID, 0, 216);
    lv_obj_clear_flag(bt_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bt_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(bt_row, 0, 0);
    lv_obj_set_style_radius(bt_row, 10, 0);
    lv_obj_set_style_pad_all(bt_row, 14, 0);
    lv_obj_t *bt_lbl = lv_label_create(bt_row);
    lv_label_set_text(bt_lbl, "Bluetooth");
    lv_obj_set_style_text_font(bt_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(bt_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(bt_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_periph_bt_switch = lv_switch_create(bt_row);
    lv_obj_set_size(s_set_periph_bt_switch, 66, 36);
    lv_obj_align(s_set_periph_bt_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_periph_bt_switch, settings_bt_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *lora_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(lora_row, 386, 84);
    lv_obj_align(lora_row, LV_ALIGN_TOP_MID, 0, 280);
    lv_obj_clear_flag(lora_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(lora_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(lora_row, 0, 0);
    lv_obj_set_style_radius(lora_row, 10, 0);
    lv_obj_set_style_pad_all(lora_row, 14, 0);
    lv_obj_t *lora_lbl = lv_label_create(lora_row);
    lv_label_set_text(lora_lbl, "LoRa");
    lv_obj_set_style_text_font(lora_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(lora_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(lora_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *lora_cap = lv_label_create(lora_row);
    lv_label_set_text(lora_cap, "hardwired, always on");
    lv_obj_set_style_text_font(lora_cap, s_font_small, 0);
    lv_obj_set_style_text_color(lora_cap, lv_color_hex(0x707070), 0);
    lv_obj_align(lora_cap, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_t *lora_sw = lv_switch_create(lora_row);
    lv_obj_set_size(lora_sw, 66, 36);
    lv_obj_align(lora_sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(lora_sw, LV_STATE_CHECKED | LV_STATE_DISABLED);

    lv_obj_t *wifi_row = lv_obj_create(s_set_periph_screen);
    lv_obj_set_size(wifi_row, 386, 84);
    lv_obj_align(wifi_row, LV_ALIGN_TOP_MID, 0, 370);
    lv_obj_clear_flag(wifi_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(wifi_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(wifi_row, 0, 0);
    lv_obj_set_style_radius(wifi_row, 10, 0);
    lv_obj_set_style_pad_all(wifi_row, 14, 0);
    lv_obj_t *wifi_lbl = lv_label_create(wifi_row);
    lv_label_set_text(wifi_lbl, "WiFi");
    lv_obj_set_style_text_font(wifi_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(wifi_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(wifi_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *wifi_cap = lv_label_create(wifi_row);
    lv_label_set_text(wifi_cap, "not available yet");
    lv_obj_set_style_text_font(wifi_cap, s_font_small, 0);
    lv_obj_set_style_text_color(wifi_cap, lv_color_hex(0x707070), 0);
    lv_obj_align(wifi_cap, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_t *wifi_sw = lv_switch_create(wifi_row);
    lv_obj_set_size(wifi_sw, 66, 36);
    lv_obj_align(wifi_sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(wifi_sw, LV_STATE_DISABLED);

    settings_periph_refresh(NULL);
    lv_timer_create(settings_periph_refresh, 1000, NULL);
}

/* ---- Ton & Vibration ---- */

static void settings_alarm_sound_switch_cb(lv_event_t *e)
{
    (void)e;
    alarm_set_sound_enabled(lv_obj_has_state(s_set_sound_alarm_switch, LV_STATE_CHECKED));
}

static void settings_notify_switch_cb(lv_event_t *e)
{
    (void)e;
    mesh_log_set_notify_enabled(lv_obj_has_state(s_set_sound_notify_switch, LV_STATE_CHECKED));
}

static void settings_sound_refresh(void)
{
    if (!s_set_sound_alarm_switch) {
        return;
    }
    if (alarm_get_sound_enabled()) { lv_obj_add_state(s_set_sound_alarm_switch, LV_STATE_CHECKED); }
    else { lv_obj_clear_state(s_set_sound_alarm_switch, LV_STATE_CHECKED); }
    if (mesh_log_get_notify_enabled()) { lv_obj_add_state(s_set_sound_notify_switch, LV_STATE_CHECKED); }
    else { lv_obj_clear_state(s_set_sound_notify_switch, LV_STATE_CHECKED); }
}

static void lvgl_build_settings_sound_screen(void)
{
    s_set_sound_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_sound_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_sound_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_sound_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_sound_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_sound_screen);
    lv_label_set_text(title, "TON & VIBRATION");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    lv_obj_t *alarm_row = lv_obj_create(s_set_sound_screen);
    lv_obj_set_size(alarm_row, 386, 76);
    lv_obj_align(alarm_row, LV_ALIGN_TOP_MID, 0, 172);
    lv_obj_clear_flag(alarm_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(alarm_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(alarm_row, 0, 0);
    lv_obj_set_style_radius(alarm_row, 10, 0);
    lv_obj_set_style_pad_all(alarm_row, 16, 0);
    lv_obj_t *alarm_lbl = lv_label_create(alarm_row);
    lv_label_set_text(alarm_lbl, "Alarm sound");
    lv_obj_set_style_text_font(alarm_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(alarm_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(alarm_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_sound_alarm_switch = lv_switch_create(alarm_row);
    lv_obj_set_size(s_set_sound_alarm_switch, 66, 36);
    lv_obj_align(s_set_sound_alarm_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_sound_alarm_switch, settings_alarm_sound_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *notify_row = lv_obj_create(s_set_sound_screen);
    lv_obj_set_size(notify_row, 386, 76);
    lv_obj_align(notify_row, LV_ALIGN_TOP_MID, 0, 272);
    lv_obj_clear_flag(notify_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(notify_row, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(notify_row, 0, 0);
    lv_obj_set_style_radius(notify_row, 10, 0);
    lv_obj_set_style_pad_all(notify_row, 16, 0);
    lv_obj_t *notify_lbl = lv_label_create(notify_row);
    lv_label_set_text(notify_lbl, "LoRa notify vibr.");
    lv_obj_set_style_text_font(notify_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(notify_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(notify_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    s_set_sound_notify_switch = lv_switch_create(notify_row);
    lv_obj_set_size(s_set_sound_notify_switch, 66, 36);
    lv_obj_align(s_set_sound_notify_switch, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(s_set_sound_notify_switch, settings_notify_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    settings_sound_refresh();
}

/* ---- Info ---- */

static void settings_info_refresh(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_set_info_screen) {
        return;
    }
    sensor_cache_t cache;
    sensor_cache_get(&cache);
    char buf[48];
    if (cache.valid) {
        snprintf(buf, sizeof(buf), "Battery: %u%%", (unsigned)cache.batt_pct);
    } else {
        snprintf(buf, sizeof(buf), "Battery: --");
    }
    lv_label_set_text(s_set_info_batt_label, buf);

    uint64_t total = 0, free = 0;
    if (sd_log_get_space(&total, &free) == ESP_OK) {
        snprintf(buf, sizeof(buf), "SD: %.1f / %.1f GB free",
                 free / 1073741824.0, total / 1073741824.0);
    } else {
        snprintf(buf, sizeof(buf), "SD: not available");
    }
    lv_label_set_text(s_set_info_sd_label, buf);
}

static void lvgl_build_settings_info_screen(void)
{
    s_set_info_screen = screen_new();
    lv_obj_set_style_bg_color(s_set_info_screen, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(s_set_info_screen, settings_sub_swipe_cb, LV_EVENT_PRESSED, (void *)settings_back_cb);
    lv_obj_add_event_cb(s_set_info_screen, settings_sub_swipe_cb, LV_EVENT_RELEASED, (void *)settings_back_cb);

    lv_obj_t *back = lv_button_create(s_set_info_screen);
    lv_obj_set_size(back, 92, 46);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 34, 36);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_small, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_set_info_screen);
    lv_label_set_text(title, "INFO");
    lv_obj_set_style_text_font(title, s_font_sec, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 96);

    lv_obj_t *fw_label = lv_label_create(s_set_info_screen);
    char fw_buf[48];
    snprintf(fw_buf, sizeof(fw_buf), "Firmware: v%s", esp_app_get_description()->version);
    lv_label_set_text(fw_label, fw_buf);
    lv_obj_set_style_text_font(fw_label, s_font_small, 0);
    lv_obj_set_style_text_color(fw_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(fw_label, LV_ALIGN_TOP_LEFT, 20, 172);

    s_set_info_batt_label = lv_label_create(s_set_info_screen);
    lv_obj_set_style_text_font(s_set_info_batt_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_info_batt_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_info_batt_label, LV_ALIGN_TOP_LEFT, 20, 228);

    s_set_info_sd_label = lv_label_create(s_set_info_screen);
    lv_obj_set_style_text_font(s_set_info_sd_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_set_info_sd_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_set_info_sd_label, LV_ALIGN_TOP_LEFT, 20, 284);

    settings_info_refresh(NULL);
    lv_timer_create(settings_info_refresh, 1000, NULL);
}

/* ---- NFC screen (chained off BHI: swipe left again) ---- */

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
        esp_lv_adapter_report_activity();
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

static void lvgl_build_nfc_screen(void)
{
    s_nfc_screen = screen_new();
    lv_obj_set_style_bg_color(s_nfc_screen, lv_color_hex(0x201030), 0);

    lv_obj_t *title = lv_label_create(s_nfc_screen);
    lv_label_set_text(title, "NFC");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_nfc_status_label = lv_label_create(s_nfc_screen);
    lv_label_set_text(s_nfc_status_label, "");
    lv_obj_set_style_text_font(s_nfc_status_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_nfc_status_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_nfc_status_label, LV_ALIGN_CENTER, 0, -60);

    s_nfc_uid_label = lv_label_create(s_nfc_screen);
    lv_label_set_text(s_nfc_uid_label, "");
    lv_obj_set_style_text_font(s_nfc_uid_label, s_font_micro, 0);
    lv_label_set_long_mode(s_nfc_uid_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_nfc_uid_label, 360);
    lv_obj_set_style_text_align(s_nfc_uid_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_nfc_uid_label, LV_ALIGN_CENTER, 0, -10);

    s_nfc_start_btn = lv_btn_create(s_nfc_screen);
    lv_obj_set_size(s_nfc_start_btn, 140, 44);
    lv_obj_align(s_nfc_start_btn, LV_ALIGN_CENTER, 0, 80);
    lv_obj_add_event_cb(s_nfc_start_btn, nfc_start_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_lbl = lv_label_create(s_nfc_start_btn);
    lv_label_set_text(btn_lbl, "Start");
    lv_obj_set_style_text_font(btn_lbl, s_font_small, 0);
    lv_obj_center(btn_lbl);

    lv_obj_t *hint = lv_label_create(s_nfc_screen);
    lv_label_set_text(hint, "swipe right: BHI sensor");
    lv_obj_set_style_text_font(hint, s_font_micro, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);

    nfc_screen_update(NULL);
    lv_timer_create(nfc_screen_update, 500, NULL);
    if (s_nfc_ctrl_task == NULL) {
        xTaskCreate(nfc_ctrl_task, "nfc_ctrl", 4096, NULL,
                    ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY, &s_nfc_ctrl_task);
    }
}

/* Worker task that actually powers the GNSS receiver. m10q_power() blocks for
 * up to a few seconds (baud probe + configuration), so it must not run on the
 * LVGL task or the UI freezes. */
static void gps_ctrl_task(void *arg)
{
    (void)arg;
    for (;;) {
        int req = s_gps_ctrl_req;
        s_gps_ctrl_req = GPS_CTRL_NONE;
        if (req == GPS_CTRL_ON && !s_gps_powered) {
            s_gps_acq_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            m10q_power(true);
            s_gps_powered = true;
            ESP_LOGI(TAG, "GNSS powered on");
        } else if (req == GPS_CTRL_OFF && s_gps_powered) {
            m10q_power(false);
            s_gps_powered = false;
            ESP_LOGI(TAG, "GNSS powered off");
        } else if (req == GPS_CTRL_REFRESH) {
            /* Boot LKP check: wait for a 3D fix so m10q's gate can persist the
             * last-known position. GNSS stays on (always-on mode); the fix just
             * updates the LKP for the next session's position aiding. */
            if (!s_gps_powered) {
                s_gps_acq_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                m10q_power(true);
                s_gps_powered = true;
            }
            ESP_LOGI(TAG, "GNSS position refresh: acquiring 3D fix...");
            uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            bool got_fix = false;
            for (;;) {
                m10q_fix_t fix;
                m10q_get_fix(&fix);
                if (fix.valid && fix.fix_3d) {
                    got_fix = true;
                    break;
                }
                uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                if (now - start >= GPS_REFRESH_TIMEOUT_MS) {
                    break;
                }
                /* Keep the watch awake so auto-sleep can't cut the rail. */
                esp_lv_adapter_report_activity();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            if (got_fix) {
                m10q_fix_t fix;
                m10q_get_fix(&fix);
                ESP_LOGI(TAG, "GNSS position refresh: 3D fix (%.5f, %.5f)",
                         fix.lat, fix.lon);
            } else {
                ESP_LOGW(TAG, "GNSS position refresh: no 3D fix within %u s",
                         GPS_REFRESH_TIMEOUT_MS / 1000);
            }
            /* GNSS stays on (always-on mode); do NOT power it off here. */
        }
#if TRACKING_ENABLED
        else if (tracking_fix_due()) {
            /* Step-gated tracking: a tracking fix is due (every N steps).
             * GNSS is already on in always-on mode, so just wait for a 3D fix
             * and hand the position to tracking. */
            double seed_lat = 0, seed_lon = 0;
            bool have_seed = tracking_get_estimated_position(&seed_lat, &seed_lon);
            if (!s_gps_powered) {
                s_gps_acq_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                m10q_power(true);
                s_gps_powered = true;
            }
            /* IMU-assisted re-acquisition: seed the receiver with the estimated
             * position (last fix + steps x stride along the last course) so the
             * warm start is faster and the on-time shorter. */
            if (have_seed) {
                m10q_seed_position(seed_lat, seed_lon);
            }
            ESP_LOGI(TAG, "GNSS tracking fix: acquiring 3D fix...");
            uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            bool got_fix = false;
            for (;;) {
                m10q_fix_t fix;
                m10q_get_fix(&fix);
                if (fix.valid && fix.fix_3d) {
                    tracking_on_fix(fix.lat, fix.lon);
                    m10q_update_last_position(fix.lat, fix.lon);
                    got_fix = true;
                    break;
                }
                uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                if (now - start >= GPS_REFRESH_TIMEOUT_MS) {
                    break;
                }
                esp_lv_adapter_report_activity();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            if (!got_fix) {
                ESP_LOGW(TAG, "GNSS tracking fix: no 3D fix within %u s",
                         GPS_REFRESH_TIMEOUT_MS / 1000);
                /* No fix: don't block forever; try again after more steps. */
                tracking_fix_clear();
            }
            /* GNSS stays on (always-on mode); do NOT power it off here. */
        }
#endif
        else if (s_gps_powered && m10q_get_state() == M10Q_STATE_OFF &&
                 !bhi260ap_is_suspended()) {
            /* Auto-sleep cut the GNSS rail underneath us (power_mgmt told the
             * driver, which set state=OFF). On wake the rail is restored but
             * the module needs a fresh power-on + config, so re-arm it.
             * Gated on !bhi260ap_is_suspended(): while the host is entering or
             * in light sleep (AP-suspend set), the rail is being cut on
             * purpose and must stay off - re-powering here (the task polls
             * every 50 ms) would undo the power-down ~50 ms after it and leave
             * the GNSS powered during sleep. */
            s_gps_acq_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            m10q_power(true);
            ESP_LOGI(TAG, "GNSS re-powered after wake");
        }

        /* Opportunistic MGA-INI position seed refresh - independent of
         * TRACKING_ENABLED. m10q_update_last_position() (the only thing
         * that persists a fresh last-known-position for next session's
         * MGA-INI POS aiding) used to be called ONLY from the tracking
         * fix-due branch above, which is compiled out entirely when
         * tracking is disabled - meaning ordinary GPS screen use or the
         * boot-time refresh got a perfectly good fix, displayed it, and
         * then never wrote it back. The next session kept re-seeding
         * whatever position was last set by hand (gnssseed), however old
         * or far away that had become - confirmed on hardware: a fix in
         * Germany, MGA-INI POS still acking a UK coordinate hundreds of km
         * off, months stale. A confidently-wrong position hint (radius
         * 1km) is worse for acquisition than no hint at all.
         *
         * Any valid 3D fix, any time GNSS is powered, now refreshes it -
         * throttled to once per LKP_REFRESH_INTERVAL_MS so a fix streaming
         * at 1 Hz doesn't turn into an NVS write every second. */
        if (s_gps_powered) {
            m10q_fix_t fix;
            m10q_get_fix(&fix);
            if (fix.valid && fix.fix_3d) {
                uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                if (now - s_gps_lkp_saved_ms >= LKP_REFRESH_INTERVAL_MS) {
                    m10q_update_last_position(fix.lat, fix.lon);
                    s_gps_lkp_saved_ms = now;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Power the GNSS receiver on/off. The actual m10q power transition happens on
 * the background control task so the UI never blocks. */
static void gps_power(bool on)
{
    s_gps_ctrl_req = on ? GPS_CTRL_ON : GPS_CTRL_OFF;
}

/* GNSS on/off switch on the GPS screen. Persists the choice so the next boot
 * powers GNSS on only if it was left enabled. */
static void gps_pwr_switch_cb(lv_event_t *e)
{
    (void)e;
    if (!s_gps_pwr_switch) {
        return;
    }
    lvgl_gps_set_enabled(lv_obj_has_state(s_gps_pwr_switch, LV_STATE_CHECKED));
}

/* GPS screen Start/Stop tracking button. */
static void gps_track_btn_cb(lv_event_t *e)
{
    (void)e;
    if (gpx_log_is_active()) {
        gpx_log_stop();
    } else {
        gpx_log_start();
    }
    gps_screen_update(NULL);
}

/* One-shot boot-time GNSS position check (background). */
static void gps_refresh(void)
{
    s_gps_ctrl_req = GPS_CTRL_REFRESH;
}

/* Public wrapper for console/other modules. */
void lvgl_gps_refresh(void)
{
    gps_refresh();
}

/* True while GNSS was deliberately enabled (GPS screen switch). Used by the
 * power manager to keep the GNSS rail alive across a sleep session instead of
 * cutting + re-powering it on every wake. */
bool lvgl_gps_enabled(void)
{
    return s_gps_enabled;
}

/* Enable/disable GNSS (single session entry point: UI switch, console, boot).
 * Persists the choice and requests the power transition on the GPS control
 * task, so every path feeds the same on/off state the power manager checks. */
void lvgl_gps_set_enabled(bool on)
{
    s_gps_enabled = on;
    gps_save_enabled(on);
    gps_power(on);
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
    ESP_LOGI(TAG, "tracking started (step-gated GNSS, display stays on)");
}

void lvgl_tracking_stop(void)
{
    if (tracking_stop() != ESP_OK) {
        ESP_LOGE(TAG, "tracking stop failed");
        return;
    }
    ESP_LOGI(TAG, "tracking stopped");
}

/* ---- Alarms/Timers list screen + create/edit + timer-start sub-screens +
 * ringing screen (docs/application.md section 8) ---- */

/* "Daily"/"Mon-Fri"/comma list, mirrors debug_cmds.c's print_weekday_mask()
 * but into a buffer instead of stdout. */
static void weekday_summary(uint8_t wmask, char *out, size_t outlen)
{
    static const char *names[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    if (wmask == ALARM_WEEKDAY_ALL) {
        snprintf(out, outlen, "Daily");
        return;
    }
    if (wmask == 0x3E) {   /* Mon..Fri */
        snprintf(out, outlen, "Mon-Fri");
        return;
    }
    out[0] = '\0';
    bool first = true;
    for (int i = 0; i < 7; i++) {
        if (wmask & (1u << i)) {
            size_t len = strlen(out);
            snprintf(out + len, outlen - len, "%s%s", first ? "" : ",", names[i]);
            first = false;
        }
    }
}

/* Re-populates the 8 fixed alarm rows and the active-timer display. Called
 * on build, from the list screen's periodic status-bar timer, and whenever
 * an edit/add/remove/timer-start returns to this screen. */
static void alarm_list_refresh(void)
{
    alarm_entry_t list[ALARM_MAX_COUNT];
    alarm_get_all(list, ALARM_MAX_COUNT);
    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        if (!list[i].in_use) {
            lv_obj_add_flag(s_alarm_row[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(s_alarm_row[i], LV_OBJ_FLAG_HIDDEN);
        char wbuf[16];
        weekday_summary(list[i].weekday_mask, wbuf, sizeof(wbuf));
        char buf[32];
        snprintf(buf, sizeof(buf), "%02u:%02u  %s", (unsigned)list[i].hour,
                 (unsigned)list[i].min, wbuf);
        lv_label_set_text(s_alarm_row_time_label[i], buf);
        if (list[i].enabled) {
            lv_obj_add_state(s_alarm_row_switch[i], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(s_alarm_row_switch[i], LV_STATE_CHECKED);
        }
    }

    if (cdtimer_is_active()) {
        uint32_t s = cdtimer_remaining_seconds();
        char buf[16];
        snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
        lv_label_set_text(s_alarm_timer_label, buf);
        lv_obj_clear_flag(s_alarm_timer_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_alarm_timer_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_alarm_timer_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_alarm_timer_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void alarm_timer_cancel_btn_cb(lv_event_t *e)
{
    (void)e;
    cdtimer_cancel();
    alarm_list_refresh();
}

static void alarm_row_switch_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    bool en = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    alarm_set_enabled(idx, en);
}

/* Tapping a row's label area (not its switch - LVGL only delivers CLICKED
 * to the row container for events that land on the container itself, not
 * on the child switch) opens that entry in the edit screen. */
static void alarm_row_click_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    alarm_entry_t list[ALARM_MAX_COUNT];
    alarm_get_all(list, ALARM_MAX_COUNT);
    if (!list[idx].in_use) {
        return;
    }
    s_alarm_edit_idx = idx;
    s_alarm_edit_hour = list[idx].hour;
    s_alarm_edit_min = list[idx].min;
    s_alarm_edit_mode = list[idx].ring_mode;
    s_alarm_edit_wmask = list[idx].weekday_mask;
    if (!s_alarm_edit_screen) {
        lvgl_build_alarm_edit_screen();
    }
    lv_obj_clear_flag(s_alarm_edit_delete_btn, LV_OBJ_FLAG_HIDDEN);
    lv_scr_load(s_alarm_edit_screen);
}

static void alarm_add_btn_cb(lv_event_t *e)
{
    (void)e;
    s_alarm_edit_idx = -1;
    s_alarm_edit_hour = 7;
    s_alarm_edit_min = 0;
    s_alarm_edit_mode = ALARM_RING_BEEP;
    s_alarm_edit_wmask = ALARM_WEEKDAY_ALL;
    if (!s_alarm_edit_screen) {
        lvgl_build_alarm_edit_screen();
    }
    lv_obj_add_flag(s_alarm_edit_delete_btn, LV_OBJ_FLAG_HIDDEN);   /* nothing to delete yet */
    lv_scr_load(s_alarm_edit_screen);
}

static void timer_add_btn_cb(lv_event_t *e)
{
    (void)e;
    if (!s_timer_screen) {
        lvgl_build_timer_screen();
    }
    lv_scr_load(s_timer_screen);
}

/* The list screen's other controls (switches, +Alarm/+Timer, edit/timer
 * sub-screens) all commit immediately, so - like the old single-alarm
 * screen - this timer exists mainly to keep the status bar and the active
 * countdown/row states live while the screen is open. */
static void alarm_screen_status_bar_update(lv_timer_t *timer)
{
    (void)timer;
    if (lv_screen_active() != s_alarm_screen) {
        return;
    }
    update_status_bar(&s_status_bar[4]);
    alarm_list_refresh();
}

static void lvgl_build_alarm_screen(void)
{
    s_alarm_screen = screen_new();
    lv_obj_set_style_bg_color(s_alarm_screen, lv_color_hex(0x201020), 0);

    build_status_bar(s_alarm_screen, &s_status_bar[4]);

    lv_obj_t *title = lv_label_create(s_alarm_screen);
    lv_label_set_text(title, "ALARMS");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    /* Active countdown, hidden unless a timer is running (alarm_list_refresh()). */
    s_alarm_timer_label = lv_label_create(s_alarm_screen);
    lv_label_set_text(s_alarm_timer_label, "");
    lv_obj_set_style_text_font(s_alarm_timer_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_alarm_timer_label, lv_color_hex(0x80D8FF), 0);
    lv_obj_align(s_alarm_timer_label, LV_ALIGN_TOP_LEFT, 30, 82);
    lv_obj_add_flag(s_alarm_timer_label, LV_OBJ_FLAG_HIDDEN);

    s_alarm_timer_cancel_btn = lv_button_create(s_alarm_screen);
    lv_obj_set_size(s_alarm_timer_cancel_btn, 90, 44);
    lv_obj_align(s_alarm_timer_cancel_btn, LV_ALIGN_TOP_RIGHT, -30, 78);
    lv_obj_t *ctl = lv_label_create(s_alarm_timer_cancel_btn);
    lv_label_set_text(ctl, "Cancel");
    lv_obj_set_style_text_font(ctl, s_font_small, 0);
    lv_obj_center(ctl);
    lv_obj_add_event_cb(s_alarm_timer_cancel_btn, alarm_timer_cancel_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_alarm_timer_cancel_btn, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *add_alarm = lv_button_create(s_alarm_screen);
    lv_obj_set_size(add_alarm, 170, 44);
    lv_obj_align(add_alarm, LV_ALIGN_TOP_LEFT, 20, 135);
    lv_obj_t *aal = lv_label_create(add_alarm);
    lv_label_set_text(aal, "+ Alarm");
    lv_obj_set_style_text_font(aal, s_font_small, 0);
    lv_obj_center(aal);
    lv_obj_add_event_cb(add_alarm, alarm_add_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *add_timer = lv_button_create(s_alarm_screen);
    lv_obj_set_size(add_timer, 170, 44);
    lv_obj_align(add_timer, LV_ALIGN_TOP_RIGHT, -20, 135);
    lv_obj_t *atl = lv_label_create(add_timer);
    lv_label_set_text(atl, "+ Timer");
    lv_obj_set_style_text_font(atl, s_font_small, 0);
    lv_obj_center(atl);
    lv_obj_add_event_cb(add_timer, timer_add_btn_cb, LV_EVENT_CLICKED, NULL);

    /* Scrollable list, up to ALARM_MAX_COUNT rows (fixed-count, hidden/shown
     * per slot - same pattern as the mesh screen's message rows). */
    lv_obj_t *list_cont = lv_obj_create(s_alarm_screen);
    lv_obj_set_size(list_cont, 370, 280);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 190);
    lv_obj_set_flex_flow(list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_cont, 6, 0);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 4, 0);

    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        lv_obj_t *row = lv_obj_create(list_cont);
        lv_obj_set_size(row, LV_PCT(100), 40);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x301830), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 6, 0);
        lv_obj_set_style_pad_all(row, 6, 0);
        lv_obj_add_event_cb(row, alarm_row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        s_alarm_row[i] = row;

        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, s_font_small, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
        s_alarm_row_time_label[i] = l;

        lv_obj_t *sw = lv_switch_create(row);
        lv_obj_set_size(sw, 56, 30);
        lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_add_event_cb(sw, alarm_row_switch_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);
        s_alarm_row_switch[i] = sw;

        lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);   /* shown by alarm_list_refresh() if in_use */
    }

    lv_obj_t *hint = lv_label_create(s_alarm_screen);
    lv_label_set_text(hint, "tap a row to edit");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -6);

    alarm_list_refresh();
    lv_timer_create(alarm_screen_status_bar_update, 1000, NULL);
}

/* ---- Alarm create/edit sub-screen (local, not in the nav ring) ---- */

static void alarm_edit_screen_refresh(void)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)s_alarm_edit_hour, (unsigned)s_alarm_edit_min);
    lv_label_set_text(s_alarm_edit_time_label, buf);

    lv_obj_clear_state(s_alarm_edit_mode_beep, LV_STATE_CHECKED);
    lv_obj_clear_state(s_alarm_edit_mode_vib, LV_STATE_CHECKED);
    lv_obj_clear_state(s_alarm_edit_mode_both, LV_STATE_CHECKED);
    lv_obj_add_state(s_alarm_edit_mode == ALARM_RING_BEEP ? s_alarm_edit_mode_beep :
                     s_alarm_edit_mode == ALARM_RING_VIB ? s_alarm_edit_mode_vib : s_alarm_edit_mode_both,
                     LV_STATE_CHECKED);

    for (int i = 0; i < 7; i++) {
        if (s_alarm_edit_wmask & (1u << i)) {
            lv_obj_add_state(s_alarm_edit_wday_btn[i], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(s_alarm_edit_wday_btn[i], LV_STATE_CHECKED);
        }
    }
}

static void alarm_edit_hour_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit_hour = (uint8_t)((s_alarm_edit_hour + 24 + delta) % 24);
    alarm_edit_screen_refresh();
}

/* Steps by 30 min (not 1) - the doc specifies full/half-hour presets, not
 * freetext/digit entry, and a 30 min step is a tap-based preset in
 * everything but name while reusing the existing +/- stepper widget instead
 * of a new scrollable time-list. */
static void alarm_edit_min_btn_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit_min = (uint8_t)((s_alarm_edit_min + 60 + delta) % 60);
    alarm_edit_screen_refresh();
}

static void alarm_edit_mode_btn_cb(lv_event_t *e)
{
    s_alarm_edit_mode = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    alarm_edit_screen_refresh();
}

static void alarm_edit_wday_btn_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    s_alarm_edit_wmask ^= (uint8_t)(1u << i);
}

static void alarm_edit_save_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t wmask = s_alarm_edit_wmask ? s_alarm_edit_wmask : ALARM_WEEKDAY_ALL;
    if (s_alarm_edit_idx < 0) {
        alarm_add(s_alarm_edit_hour, s_alarm_edit_min, s_alarm_edit_mode, wmask);
    } else {
        alarm_update(s_alarm_edit_idx, s_alarm_edit_hour, s_alarm_edit_min, s_alarm_edit_mode, wmask);
    }
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void alarm_edit_delete_btn_cb(lv_event_t *e)
{
    (void)e;
    if (s_alarm_edit_idx >= 0) {
        alarm_remove(s_alarm_edit_idx);
    }
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void alarm_edit_back_btn_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void lvgl_build_alarm_edit_screen(void)
{
    s_alarm_edit_screen = screen_new();
    lv_obj_set_style_bg_color(s_alarm_edit_screen, lv_color_hex(0x201020), 0);

    lv_obj_t *back = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, alarm_edit_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_alarm_edit_screen);
    lv_label_set_text(title, "ALARM");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_alarm_edit_time_label = lv_label_create(s_alarm_edit_screen);
    lv_obj_set_style_text_font(s_alarm_edit_time_label, s_font_sec, 0);
    lv_obj_set_style_text_color(s_alarm_edit_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_alarm_edit_time_label, LV_ALIGN_TOP_MID, 0, 55);

    lv_obj_t *h_minus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(h_minus, 150, 64);
    lv_obj_align(h_minus, LV_ALIGN_TOP_LEFT, 20, 110);
    lv_obj_t *hml = lv_label_create(h_minus);
    lv_label_set_text(hml, "H-");
    lv_obj_set_style_text_font(hml, s_font_small, 0);
    lv_obj_center(hml);
    lv_obj_add_event_cb(h_minus, alarm_edit_hour_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-1);

    lv_obj_t *h_plus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(h_plus, 150, 64);
    lv_obj_align(h_plus, LV_ALIGN_TOP_RIGHT, -20, 110);
    lv_obj_t *hpl = lv_label_create(h_plus);
    lv_label_set_text(hpl, "H+");
    lv_obj_set_style_text_font(hpl, s_font_small, 0);
    lv_obj_center(hpl);
    lv_obj_add_event_cb(h_plus, alarm_edit_hour_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)1);

    lv_obj_t *m_minus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(m_minus, 150, 64);
    lv_obj_align(m_minus, LV_ALIGN_TOP_LEFT, 20, 185);
    lv_obj_t *mml = lv_label_create(m_minus);
    lv_label_set_text(mml, "M-30");
    lv_obj_set_style_text_font(mml, s_font_small, 0);
    lv_obj_center(mml);
    lv_obj_add_event_cb(m_minus, alarm_edit_min_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-30);

    lv_obj_t *m_plus = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(m_plus, 150, 64);
    lv_obj_align(m_plus, LV_ALIGN_TOP_RIGHT, -20, 185);
    lv_obj_t *mpl = lv_label_create(m_plus);
    lv_label_set_text(mpl, "M+30");
    lv_obj_set_style_text_font(mpl, s_font_small, 0);
    lv_obj_center(mpl);
    lv_obj_add_event_cb(m_plus, alarm_edit_min_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)30);

    lv_obj_t *wday_lbl = lv_label_create(s_alarm_edit_screen);
    lv_label_set_text(wday_lbl, "Repeat");
    lv_obj_set_style_text_font(wday_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(wday_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(wday_lbl, LV_ALIGN_TOP_LEFT, 20, 260);

    static const char *wday_names[7] = { "Su", "Mo", "Tu", "We", "Th", "Fr", "Sa" };
    for (int i = 0; i < 7; i++) {
        lv_obj_t *wb = lv_button_create(s_alarm_edit_screen);
        lv_obj_set_size(wb, 48, 40);
        lv_obj_align(wb, LV_ALIGN_TOP_LEFT, 20 + i * 52, 285);
        lv_obj_t *wl = lv_label_create(wb);
        lv_label_set_text(wl, wday_names[i]);
        lv_obj_set_style_text_font(wl, s_font_micro, 0);
        lv_obj_center(wl);
        lv_obj_add_flag(wb, LV_OBJ_FLAG_CHECKABLE);
        lv_obj_add_event_cb(wb, alarm_edit_wday_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        s_alarm_edit_wday_btn[i] = wb;
    }

    lv_obj_t *mode_lbl = lv_label_create(s_alarm_edit_screen);
    lv_label_set_text(mode_lbl, "Ring");
    lv_obj_set_style_text_font(mode_lbl, s_font_small, 0);
    lv_obj_set_style_text_color(mode_lbl, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(mode_lbl, LV_ALIGN_TOP_LEFT, 20, 345);

    s_alarm_edit_mode_beep = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_mode_beep, 105, 56);
    lv_obj_align(s_alarm_edit_mode_beep, LV_ALIGN_TOP_LEFT, 20, 370);
    lv_obj_t *mb = lv_label_create(s_alarm_edit_mode_beep);
    lv_label_set_text(mb, "Beep");
    lv_obj_set_style_text_font(mb, s_font_small, 0);
    lv_obj_center(mb);
    lv_obj_add_flag(s_alarm_edit_mode_beep, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_edit_mode_beep, alarm_edit_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_BEEP);

    s_alarm_edit_mode_vib = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_mode_vib, 105, 56);
    lv_obj_align(s_alarm_edit_mode_vib, LV_ALIGN_TOP_MID, 0, 370);
    lv_obj_t *mv = lv_label_create(s_alarm_edit_mode_vib);
    lv_label_set_text(mv, "Vib");
    lv_obj_set_style_text_font(mv, s_font_small, 0);
    lv_obj_center(mv);
    lv_obj_add_flag(s_alarm_edit_mode_vib, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_edit_mode_vib, alarm_edit_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_VIB);

    s_alarm_edit_mode_both = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_mode_both, 105, 56);
    lv_obj_align(s_alarm_edit_mode_both, LV_ALIGN_TOP_RIGHT, -20, 370);
    lv_obj_t *mbt = lv_label_create(s_alarm_edit_mode_both);
    lv_label_set_text(mbt, "Both");
    lv_obj_set_style_text_font(mbt, s_font_small, 0);
    lv_obj_center(mbt);
    lv_obj_add_flag(s_alarm_edit_mode_both, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(s_alarm_edit_mode_both, alarm_edit_mode_btn_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)ALARM_RING_BOTH);

    lv_obj_t *save = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(save, 170, 50);
    lv_obj_align(save, LV_ALIGN_BOTTOM_LEFT, 20, -14);
    lv_obj_t *svl = lv_label_create(save);
    lv_label_set_text(svl, "Save");
    lv_obj_set_style_text_font(svl, s_font_small, 0);
    lv_obj_center(svl);
    lv_obj_add_event_cb(save, alarm_edit_save_btn_cb, LV_EVENT_CLICKED, NULL);

    s_alarm_edit_delete_btn = lv_button_create(s_alarm_edit_screen);
    lv_obj_set_size(s_alarm_edit_delete_btn, 170, 50);
    lv_obj_align(s_alarm_edit_delete_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -14);
    lv_obj_set_style_bg_color(s_alarm_edit_delete_btn, lv_color_hex(0x802020), 0);
    lv_obj_t *dl = lv_label_create(s_alarm_edit_delete_btn);
    lv_label_set_text(dl, "Delete");
    lv_obj_set_style_text_font(dl, s_font_small, 0);
    lv_obj_center(dl);
    lv_obj_add_event_cb(s_alarm_edit_delete_btn, alarm_edit_delete_btn_cb, LV_EVENT_CLICKED, NULL);

    alarm_edit_screen_refresh();
}

/* ---- Timer-start sub-screen (local, not in the nav ring) ---- */

static const uint16_t s_timer_presets_min[8] = { 1, 5, 10, 15, 30, 60, 90, 120 };

static void timer_preset_btn_cb(lv_event_t *e)
{
    uint16_t minutes = (uint16_t)(uintptr_t)lv_event_get_user_data(e);
    cdtimer_start((uint32_t)minutes * 60);
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void timer_back_btn_cb(lv_event_t *e)
{
    (void)e;
    lv_scr_load(s_alarm_screen);
    alarm_list_refresh();
}

static void lvgl_build_timer_screen(void)
{
    s_timer_screen = screen_new();
    lv_obj_set_style_bg_color(s_timer_screen, lv_color_hex(0x102020), 0);

    lv_obj_t *back = lv_button_create(s_timer_screen);
    lv_obj_set_size(back, 70, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, "< Back");
    lv_obj_set_style_text_font(bl, s_font_micro, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, timer_back_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(s_timer_screen);
    lv_label_set_text(title, "TIMER");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    for (int i = 0; i < 8; i++) {
        int col = i % 2;
        int row = i / 2;
        lv_obj_t *btn = lv_button_create(s_timer_screen);
        lv_obj_set_size(btn, 170, 70);
        lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 20 + col * 190, 80 + row * 90);
        lv_obj_t *l = lv_label_create(btn);
        char buf[16];
        snprintf(buf, sizeof(buf), "%u min", s_timer_presets_min[i]);
        lv_label_set_text(l, buf);
        lv_obj_set_style_text_font(l, s_font_small, 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(btn, timer_preset_btn_cb, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)s_timer_presets_min[i]);
    }
}

/* ---- Ringing screen: shown while an alarm or timer rings ---- */

static void alarm_dismiss_btn_cb(lv_event_t *e)
{
    (void)e;
    alarm_dismiss();
}

static void alarm_snooze_btn_cb(lv_event_t *e)
{
    (void)e;
    alarm_snooze();
}

static void lvgl_build_ring_screen(void)
{
    s_ring_screen = screen_new();
    lv_obj_set_style_bg_color(s_ring_screen, lv_color_hex(0x300000), 0);

    s_ring_title_label = lv_label_create(s_ring_screen);
    lv_label_set_text(s_ring_title_label, "ALARM");
    lv_obj_set_style_text_font(s_ring_title_label, s_font_small, 0);
    lv_obj_set_style_text_color(s_ring_title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_ring_title_label, LV_ALIGN_TOP_MID, 0, 18);

    s_ring_time_label = lv_label_create(s_ring_screen);
    lv_obj_set_style_text_font(s_ring_time_label, s_font_time, 0);
    lv_obj_set_style_text_color(s_ring_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_ring_time_label, LV_ALIGN_TOP_MID, 0, 60);

    lv_obj_t *dismiss = lv_button_create(s_ring_screen);
    lv_obj_set_size(dismiss, 330, 112);
    lv_obj_align(dismiss, LV_ALIGN_TOP_MID, 0, 220);
    lv_obj_t *dl = lv_label_create(dismiss);
    lv_label_set_text(dl, "Dismiss");
    lv_obj_set_style_text_font(dl, s_font_small, 0);
    lv_obj_center(dl);
    /* Fire on touch-down so the first tap acts immediately, regardless of
     * click state or timing. Kept as a touch fallback alongside the
     * physical-button dismiss (main/power_mgmt.c's button callback -> see
     * alarm.c's alarm_button_cb()) - the doc doesn't say touch-dismiss must
     * be removed, only that stopping is button-driven. */
    lv_obj_add_event_cb(dismiss, alarm_dismiss_btn_cb, LV_EVENT_PRESSED, NULL);

    s_ring_snooze_btn = lv_button_create(s_ring_screen);
    lv_obj_set_size(s_ring_snooze_btn, 330, 112);
    lv_obj_align(s_ring_snooze_btn, LV_ALIGN_TOP_MID, 0, 340);
    lv_obj_t *sl = lv_label_create(s_ring_snooze_btn);
    lv_label_set_text(sl, "Snooze 10 min");
    lv_obj_set_style_text_font(sl, s_font_small, 0);
    lv_obj_center(sl);
    lv_obj_add_event_cb(s_ring_snooze_btn, alarm_snooze_btn_cb, LV_EVENT_PRESSED, NULL);
}

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
                lv_label_set_text(s_ring_time_label, "Time's up");
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

/* ---- 5-screen nav ring (docs/application.md section 4.3) ----
 *
 * A single closed ring, not the tree this file used to have: swipe left
 * advances (Main -> GPS -> LoRa -> Settings -> Alarms -> Main -> ...), swipe
 * right retreats - the spec's own closing line ("swiping continuously in one
 * direction passes through all five before returning to Main") is exactly a
 * modular index walk, so that's what this is, replacing the old per-screen
 * if/else tree entirely. Vertical swipes are unassigned on all five ring
 * screens (spec: reserved, not built yet).
 *
 * The BHI (sensor), NFC, and Power screens are NOT in this ring (the spec
 * doesn't mention them) - their build functions and debug-console entry
 * points (lvgl_show_bhi_screen(), the nfcpoll/nfcprobe path) still work, but
 * nothing routes to them by swipe any more. menu_timeout_cb()'s existing
 * per-screen timeouts for them are left in place unchanged: reaching them
 * via a debug command still needs a way back to the watch face, and the
 * timeout already provides one. */
typedef struct {
    lv_obj_t **screen;      /* &s_watch_screen, &s_gps_screen, ... */
    void (*build)(void);    /* lazy builder; NULL for the watch face (built once at boot) */
} nav_ring_entry_t;

static const nav_ring_entry_t s_nav_ring[] = {
    { &s_watch_screen,    NULL },
    { &s_gps_screen,      lvgl_build_gps_screen },
    { &s_mesh_screen,     lvgl_build_mesh_screen },
    { &s_settings_screen, lvgl_build_settings_screen },
    { &s_alarm_screen,    lvgl_build_alarm_screen },
};
#define NAV_RING_COUNT (sizeof(s_nav_ring) / sizeof(s_nav_ring[0]))

static int nav_ring_index_of(lv_obj_t *screen)
{
    for (size_t i = 0; i < NAV_RING_COUNT; i++) {
        if (*s_nav_ring[i].screen == screen) {
            return (int)i;
        }
    }
    return -1;
}

static void nav_ring_go(int index)
{
    const nav_ring_entry_t *entry = &s_nav_ring[index];
    if (entry->screen == &s_watch_screen) {
        lvgl_show_watch_face();   /* also forces an immediate label refresh */
        return;
    }
    if (!*entry->screen && entry->build) {
        entry->build();
    }
    if (*entry->screen) {
        lv_scr_load(*entry->screen);
    }
}

/* ---- Swipe navigation (indev-level: fires for every touch) ---- */

static void swipe_event_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_active();
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        s_swipe_start = p;
        s_swipe_active = true;
        s_last_touch_tick = lv_tick_get();
        return;
    }
    if (lv_event_get_code(e) != LV_EVENT_RELEASED || !s_swipe_active) {
        return;
    }
    s_swipe_active = false;
    s_last_touch_tick = lv_tick_get();

    int dy = p.y - s_swipe_start.y;
    int dx = p.x - s_swipe_start.x;
    if (abs(dx) < SWIPE_DIST && abs(dy) < SWIPE_DIST) {
        return;
    }
    bool horiz = abs(dx) > abs(dy);
    if (!horiz) {
        return;   /* vertical: unassigned on every ring screen, see above */
    }

    int idx = nav_ring_index_of(lv_screen_active());
    if (idx < 0) {
        return;   /* not on a ring screen (e.g. BHI/NFC reached via debug command) - no gesture nav */
    }
    int next = (dx < 0) ? (int)((idx + 1) % NAV_RING_COUNT)
                        : (int)((idx + NAV_RING_COUNT - 1) % NAV_RING_COUNT);
    nav_ring_go(next);
}

static void watch_face_long_press_cb(lv_event_t *e)
{
    (void)e;
    if (lv_screen_active() != s_watch_screen) {
        return;
    }
    lvgl_show_settings_disp();
}

/* ---- Menu inactivity timeout ----
 * Any non-watch-face screen returns to the watch face after MENU_TIMEOUT_MS
 * without a touch. The BHI sensor screen keeps its orientation cube up for
 * BHI_TIMEOUT_MS (10 s); the GPS and alarm/ring screens are exempt (longer
 * observation). */
static void menu_timeout_cb(lv_timer_t *timer)
{
    (void)timer;
    lv_obj_t *cur = lv_screen_active();
    if (cur == s_gps_screen || cur == s_alarm_screen || cur == s_ring_screen ||
        cur == s_alarm_edit_screen || cur == s_timer_screen || cur == s_node_screen ||
        cur == s_settings_screen || cur == s_set_tz_screen || cur == s_set_disp_screen ||
        cur == s_set_periph_screen || cur == s_set_sound_screen || cur == s_set_info_screen) {
        return;
    }
    if (cur == s_watch_screen) {
        return;
    }
    uint32_t timeout = (cur == s_bhi_screen) ? BHI_TIMEOUT_MS :
                        (cur == s_nfc_screen) ? NFC_MENU_TIMEOUT_MS : MENU_TIMEOUT_MS;
    if (lv_tick_get() - s_last_touch_tick >= timeout) {
        lvgl_show_watch_face();
    }
}

/* Load the watch face and refresh the clock labels. Full-screen invalidation
 * here is avoided: a 502-row full redraw queues ~11 band flushes in one cycle,
 * which transiently spikes internal DMA heap usage and can fail the SPI flush
 * (ESP_ERR_NO_MEM / screen corruption). Refreshing the labels re-reads the RTC
 * (time may have changed while a menu was open) at low cost. */
static void lvgl_show_watch_face(void)
{
    lv_scr_load(s_watch_screen);
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        watch_face_update(NULL);
        esp_lv_adapter_unlock();
    }
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

static void bhi260_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(50));
    bool initialized = (bhi260ap_init(twatch_imu_dev) == ESP_OK);
    if (!initialized) {
        ESP_LOGW(TAG, "BHI260AP init failed; retrying");
    }
    for (;;) {
        /* While the chip is in AP-suspend (host sleeping), no data flows by
         * design: skip polling and the stale re-init so the wake-up gesture
         * stream stays armed (re-init would leave AP-suspend and flood the
         * wake task with gesture events). */
        if (bhi260ap_is_suspended()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
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
                ESP_LOGW(TAG, "BHI260AP data stale, re-initializing");
                bhi260ap_deinit();
                initialized = false;
            }
        } else if (bhi260ap_init(twatch_imu_dev) == ESP_OK) {
            initialized = true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
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

esp_err_t lvgl_app_start(void)
{
    mount_assets();

    /* Load display timeout/brightness (+ the rest of power_mgmt's persisted
     * settings) now: esp_lv_adapter_init() below needs the timeout
     * immediately, well before power_mgmt_init()'s own (heavier, GPIO/task)
     * setup runs later in this function. Safe to load twice. */
    power_mgmt_load_config();

    const esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    esp_lv_adapter_config_t adapter_cfg_mut = adapter_cfg;
    /* Auto light sleep: pause LVGL after idle, then tickless light sleep. */
    adapter_cfg_mut.auto_sleep.enable = true;
    adapter_cfg_mut.auto_sleep.mode = ESP_LV_ADAPTER_AUTO_SLEEP_MODE_PAUSE;
    adapter_cfg_mut.auto_sleep.idle_timeout_ms = power_mgmt_get_display_timeout_s() * 1000;
    adapter_cfg_mut.auto_sleep.callbacks.on_enter_sleep = power_mgmt_enter_sleep;
    adapter_cfg_mut.auto_sleep.callbacks.on_exit_sleep = power_mgmt_exit_sleep;
    ESP_RETURN_ON_ERROR(esp_lv_adapter_init(&adapter_cfg_mut), TAG, "adapter init");

    /* Draw buffers in internal DMA-capable RAM: PSRAM buffers would need a
     * temp internal-DMA copy per band flush, and rapid redraws exhaust the
     * ~113 KB internal DMA pool (ESP_ERR_NO_MEM -> corrupted screen). A single
     * 48-row band (38 KB) fits internal RAM and DMA reads it directly. */
    esp_lv_adapter_display_config_t display_cfg = ESP_LV_ADAPTER_DISPLAY_SPI_WITHOUT_PSRAM_DEFAULT_CONFIG(
        co5300_get_panel(),
        co5300_get_panel_io(),
        CO5300_RES_X,
        CO5300_RES_Y,
        ESP_LV_ADAPTER_ROTATE_0);   /* rotation not supported for QSPI */
    display_cfg.profile.buffer_height = 48;   /* partial bands; full frame exceeds SPI DMA max */

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

    /* Touch input (CST9217). */
    esp_lcd_touch_handle_t tp = cst9217_get_handle();
    if (tp) {
        esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, tp);
        uint16_t nx = 0, ny = 0;
        if (cst9217_get_resolution(&nx, &ny) == ESP_OK && nx && ny) {
            touch_cfg.scale.x = (float)CO5300_RES_X / nx;
            touch_cfg.scale.y = (float)CO5300_RES_Y / ny;
        }
        s_touch_indev = esp_lv_adapter_register_touch(&touch_cfg);
        if (s_touch_indev) {
            ESP_LOGI(TAG, "touch registered");
            /* Indev-level swipe detection: fires for every touch regardless of
             * which widget/screen is active. */
            lv_indev_add_event_cb(s_touch_indev, swipe_event_cb, LV_EVENT_PRESSED, NULL);
            lv_indev_add_event_cb(s_touch_indev, swipe_event_cb, LV_EVENT_RELEASED, NULL);
            /* Tap-and-hold on the watch face -> Display settings (section
             * 9.3). LVGL's default long-press time (~400ms) is used as-is.
             * A near-zero-movement long-press never crosses SWIPE_DIST, so
             * this never fires the ring-swipe logic above too. */
            lv_indev_add_event_cb(s_touch_indev, watch_face_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
        } else {
            ESP_LOGE(TAG, "touch registration failed");
        }
        /* The touch driver re-enables its GPIO interrupt; re-apply night mode
         * so touch stays disabled (no accidental wake) during night hours. */
        power_mgmt_recheck_night_mode();
    }

    ESP_LOGI(TAG, "LVGL started (watch face)");

    /* BHI260AP sensor task (needs SPIFFS assets, already mounted above). */
    xTaskCreate(bhi260_task, "bhi260", 4096, NULL, 5, NULL);

    /* GNSS control task. GNSS is off at startup unless the GPS screen switch
     * is on (default off); enabling it starts one long-lived power-on session
     * that survives sleep (see lvgl_gps_enabled()). */
    if (s_gps_ctrl_task == NULL) {
        xTaskCreate(gps_ctrl_task, "gps_ctrl", 8192, NULL,
                    ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY, &s_gps_ctrl_task);
    }
    s_gps_enabled = gps_load_enabled();
    gps_power(s_gps_enabled);
    if (s_gps_enabled) {
        /* One-shot boot-time position check to refresh the aided-start seed
         * (last-known position in NVS) so the next power-on fixes fast. */
        gps_refresh();
    }

    /* Step-gated distance tracking (lifetime distance + steps). No-op while
     * TRACKING_ENABLED is 0. */
    tracking_init();

    return ESP_OK;
}

/* ---- Screenshot dump (debug) ----
 * Captures the active LVGL screen and prints it to the console as base64 of
 * RGB565 (little-endian) pixels, framed for the host decode script:
 *   ==SHOT:<w>x<h>==  <base64>  ==ENDSHOT==
 * Byte order matches the native RGB565 snapshot; the host script byte-swaps
 * for the big-endian panel if needed. Call in LVGL task context (with the
 * adapter lock held). */

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
