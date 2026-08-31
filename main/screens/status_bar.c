/*
 * status_bar.c - see status_bar.h. Shared between the firmware and the
 * host sim - it only touches LVGL plus the driver-facing headers that
 * sim/mock_hw.h already mirrors 1:1 (sd_log.h, twatch_board.h, m10q.h,
 * gpx_log.h, ble_debug.h, sensor_cache.h, axp2101.h), so it compiles
 * unchanged on both sides. No ESP_LOG use here, so no portable_log.h
 * dependency.
 *
 * A small row of state icons shown on the watch face (see below - it used to
 * repeat on every nav-ring screen, removed per explicit feedback). Built by
 * build_status_bar_big() and refreshed by update_status_bar() from the
 * watch face's own update timer.
 *
 * Colors follow the spec's own convention (docs/application.md section
 * 3.1): grey = off/absent, orange = transitioning/warning, green = active
 * at target state, red = active-but-notable (kept per-icon, not applied
 * uniformly - see below). Most icons are short colored text labels/symbol
 * glyphs; GNSS and LoRa are small hand-drawn bitmaps (status_icons.h) tinted
 * the same way via lv_obj_set_style_image_recolor() instead of text color -
 * LVGL's built-in symbol font has no satellite or mesh-network glyph.
 */
#include "status_bar.h"
#include "status_icons.h"
#include <stdio.h>
#include "sd_log.h"
#include "twatch_board.h"
#include "m10q.h"
#include "gpx_log.h"
#include "mesh_log.h"
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
 * lv_symbol_def.h) ships real icons for SD/BT/WiFi. GPX-tracking has no good
 * built-in glyph either, but as of this pass reuses the old GPS pin/arrow
 * glyph (freed up once GNSS moved to its own satellite bitmap below) rather
 * than staying plain text - close enough to "a location is being recorded"
 * to read fine at a glance. */
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

/* GNSS/LoRa: small hand-drawn bitmaps (status_icons.h), tinted via image
 * recolor instead of text color - same grey/orange/green/red states, just a
 * different LVGL style property since these are lv_image_t, not labels. */
static lv_obj_t *status_image_create(lv_obj_t *parent, const lv_image_dsc_t *src,
                                      lv_align_t align, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *img = lv_image_create(parent);
    lv_image_set_src(img, src);
    lv_obj_set_style_image_recolor_opa(img, LV_OPA_COVER, 0);
    lv_obj_set_style_image_recolor(img, STATUS_COLOR_GREY, 0);
    lv_obj_align(img, align, x, y);
    return img;
}

/* Watch-face-only status bar: double-size icons over two rows since the
 * doubled glyphs/text don't fit one row within the safe area (see
 * STATUS_BAR_Y's comment). Used to also appear (single-row, smaller) on
 * every other nav-ring screen - removed there per explicit feedback (read
 * as visual noise repeated across screens); this is now the only place a
 * status bar is built at all.
 *
 * Row grouping (per explicit feedback): row 1 is every RF-carrying
 * subsystem (Bluetooth, WiFi, GNSS, LoRa/Meshtastic) so a glance at the top
 * row alone answers "is anything transmitting/receiving"; row 2 is
 * everything else (SD card, GPX file logging, charge state, battery). */
#define STATUS_BAR_Y_ROW2 (STATUS_BAR_Y + 44)
void build_status_bar_big(lv_obj_t *parent, status_bar_t *out)
{
    const lv_font_t *f = &lv_font_montserrat_28;
    /* Requested order: Bluetooth, WiFi, GNSS, LoRa. LoRa's mesh-icon bitmap
     * (28x28, the widest thing in this row) goes last, where it has the
     * most room before the right safe-area edge (x<=370, see STATUS_BAR_Y's
     * comment). */
    out->bt   = status_icon_create_ex(parent, LV_SYMBOL_BLUETOOTH, LV_ALIGN_TOP_LEFT,  40, STATUS_BAR_Y,       f);
    out->wifi = status_icon_create_ex(parent, LV_SYMBOL_WIFI,      LV_ALIGN_TOP_LEFT,  120, STATUS_BAR_Y,      f);
    out->gps  = status_image_create(parent, &status_icon_satellite, LV_ALIGN_TOP_LEFT, 200, STATUS_BAR_Y);
    out->lora = status_image_create(parent, &status_icon_mesh,      LV_ALIGN_TOP_LEFT, 280, STATUS_BAR_Y);
    /* CHG left-aligned with SD/GPX rather than paired right-aligned next to
     * Batt (the original layout, before this pass's row split, had them on
     * separate rows entirely so this never collided): Batt's text is
     * variable-width ("<icon> 100%" is noticeably wider than "<icon> 8%"),
     * and a fixed right-aligned CHG offset overlapped it at the wide end -
     * reported live ("the usb connected flash is now overlapping the
     * battery"). Left-aligning CHG at a fixed position and giving Batt sole
     * ownership of the right side removes the collision regardless of how
     * wide the percentage text gets. */
    out->sd   = status_icon_create_ex(parent, LV_SYMBOL_SD_CARD,   LV_ALIGN_TOP_LEFT,  40, STATUS_BAR_Y_ROW2,  f);
    out->gpx  = status_icon_create_ex(parent, LV_SYMBOL_GPS,       LV_ALIGN_TOP_LEFT,  120, STATUS_BAR_Y_ROW2, f);
    out->chg  = status_icon_create_ex(parent, LV_SYMBOL_CHARGE,    LV_ALIGN_TOP_LEFT,  200, STATUS_BAR_Y_ROW2, f);
    out->batt = status_icon_create_ex(parent, LV_SYMBOL_BATTERY_EMPTY " --%", LV_ALIGN_TOP_RIGHT, -40, STATUS_BAR_Y_ROW2, f);
}

void update_status_bar(const status_bar_t *bar)
{
    if (!bar->sd) {
        return;
    }

    /* Green = card seated and idle - the normal resting state under the
     * on-demand SD lifecycle (sd_log_session_begin()/end(), see sd_log.h):
     * the card is mounted only for the brief window a write is actually in
     * flight, specifically so a card left mounted for the whole awake
     * session can't be corrupted by an unclean power loss while idle. Red =
     * actively mounted/writing right now (sd_log_available() true only
     * during that window). Grey = no card seated. (Earlier attempt at this
     * had it backwards because the card was, at the time, still mounted
     * for the entire awake session rather than on demand - fixed properly
     * now, see sd_log.c.) */
    lv_obj_set_style_text_color(bar->sd, sd_log_available() ? STATUS_COLOR_RED :
                                (twatch_sd_card_seated() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY), 0);

    m10q_state_t gps_st = m10q_get_state();
    lv_obj_set_style_image_recolor(bar->gps,
        (gps_st == M10Q_STATE_FIXED) ? STATUS_COLOR_GREEN :
        (gps_st == M10Q_STATE_ACQUIRING) ? STATUS_COLOR_ORANGE : STATUS_COLOR_GREY, 0);

    /* Reflects gpx_log.c (time-based GPX file logging), not tracking.c's
     * unrelated step-gated pedometer (which stays disabled - see
     * TRACKING_ENABLED in tracking.h). */
    lv_obj_set_style_text_color(bar->gpx, gpx_log_is_active() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);

    /* Reflects the Mesh screen's LoRa on/off switch (mesh_log_set_enabled(),
     * default off) - green while the radio is switched on, grey while off
     * (rail cut entirely, see mesh_log.c). */
    lv_obj_set_style_image_recolor(bar->lora, mesh_log_get_enabled() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);

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

        /* Always visible, colored like every other status icon here (grey
         * = off/absent, green = active) - was hidden outright when not
         * charging, the only icon in this row that did that instead of
         * just going grey (reported live: unplugging USB made the icon
         * vanish instead of leaving a grey flash behind). */
        bool charging = (cache.chg_state != AXP2101_CHG_STOP);
        lv_obj_set_style_text_color(bar->chg, charging ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);
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
