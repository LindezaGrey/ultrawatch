/*
 * status_bar.c - see status_bar.h. Shared between the firmware and the
 * host sim - it only touches LVGL plus the driver-facing headers that
 * sim/mock_hw.h already mirrors 1:1 (sd_log.h, twatch_board.h, m10q.h,
 * gpx_log.h, ble_debug.h, sensor_cache.h, axp2101.h), so it compiles
 * unchanged on both sides. No ESP_LOG use here, so no portable_log.h
 * dependency.
 *
 * A small row of state icons shown identically on all five nav-ring
 * screens. LVGL objects can't be shared across screens, so each screen
 * gets its own instance, built by build_status_bar()/build_status_bar_big()
 * and refreshed by update_status_bar() from that screen's own update timer -
 * the same "screen owns its own timer, function is a no-op when that screen
 * isn't active" pattern used throughout this app.
 *
 * Colors follow the spec's own convention (docs/application.md section
 * 3.1): grey = off/absent, orange = transitioning/warning, green = active
 * at target state, red = active-but-notable (kept per-icon, not applied
 * uniformly - see below). Icons are short colored text labels, not symbol
 * font glyphs: several of these (GPX-tracking, LoRa) have no good built-in
 * LVGL symbol, and a uniform text style avoids guessing at symbol-font
 * availability for the ones that might (SD/GPS/BT/WiFi).
 */
#include "status_bar.h"
#include <stdio.h>
#include "sd_log.h"
#include "twatch_board.h"
#include "m10q.h"
#include "gpx_log.h"
#include "ble_debug.h"
#include "sensor_cache.h"
#include "axp2101.h"

#define STATUS_COLOR_GREY   lv_color_hex(0x888888)
#define STATUS_COLOR_ORANGE lv_color_hex(0xFFB300)
#define STATUS_COLOR_GREEN  lv_color_hex(0x00E676)
#define STATUS_COLOR_RED    lv_color_hex(0xFF5252)

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
static lv_obj_t *status_icon_create_ex(lv_obj_t *parent, const char *text, lv_align_t align,
                                        lv_coord_t x, lv_coord_t y, const lv_font_t *font)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, STATUS_COLOR_GREY, 0);
    lv_obj_align(l, align, x, y);
    return l;
}

static lv_obj_t *status_icon_create(lv_obj_t *parent, const char *text, lv_align_t align, lv_coord_t x)
{
    return status_icon_create_ex(parent, text, align, x, STATUS_BAR_Y, &lv_font_montserrat_14);
}

void build_status_bar(lv_obj_t *parent, status_bar_t *out)
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

/* Watch-face-only variant: double-size icons (montserrat_28 vs. the other
 * four nav-ring screens' montserrat_14), spread over two rows since the
 * doubled glyphs/text no longer fit one row within the safe area. Feeds
 * the same status_bar_t/update_status_bar() - only construction differs,
 * refresh logic is font-size-agnostic. */
#define STATUS_BAR_Y_ROW2 (STATUS_BAR_Y + 44)
void build_status_bar_big(lv_obj_t *parent, status_bar_t *out)
{
    const lv_font_t *f = &lv_font_montserrat_28;
    out->sd   = status_icon_create_ex(parent, LV_SYMBOL_SD_CARD,   LV_ALIGN_TOP_LEFT,  40, STATUS_BAR_Y,      f);
    out->gps  = status_icon_create_ex(parent, LV_SYMBOL_GPS,       LV_ALIGN_TOP_LEFT,  120, STATUS_BAR_Y,     f);
    out->gpx  = status_icon_create_ex(parent, "GPX",               LV_ALIGN_TOP_LEFT,  200, STATUS_BAR_Y,     f);
    out->chg  = status_icon_create_ex(parent, LV_SYMBOL_CHARGE,    LV_ALIGN_TOP_RIGHT, -90, STATUS_BAR_Y,    f);
    out->lora = status_icon_create_ex(parent, "LoRa",              LV_ALIGN_TOP_LEFT,  40, STATUS_BAR_Y_ROW2, f);
    out->bt   = status_icon_create_ex(parent, LV_SYMBOL_BLUETOOTH, LV_ALIGN_TOP_LEFT,  120, STATUS_BAR_Y_ROW2, f);
    out->wifi = status_icon_create_ex(parent, LV_SYMBOL_WIFI,      LV_ALIGN_TOP_LEFT,  200, STATUS_BAR_Y_ROW2, f);
    out->batt = status_icon_create_ex(parent, LV_SYMBOL_BATTERY_EMPTY " --%", LV_ALIGN_TOP_RIGHT, -40, STATUS_BAR_Y_ROW2, f);
}

void update_status_bar(const status_bar_t *bar)
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

void status_bar_set_hidden(const status_bar_t *bar, bool hidden)
{
    if (!bar->sd) {
        return;
    }
    lv_obj_t *icons[] = { bar->sd, bar->gps, bar->gpx, bar->lora,
                           bar->bt, bar->wifi, bar->batt, bar->chg };
    for (size_t i = 0; i < sizeof(icons) / sizeof(icons[0]); i++) {
        if (hidden) {
            lv_obj_add_flag(icons[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(icons[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
