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
 * glyphs; GNSS and LoRa are small bitmap icons (status_icons.h) tinted
 * the same way via lv_obj_set_style_image_recolor() instead of text color -
 * LVGL's built-in symbol font has no satellite or mesh-network glyph.
 */
#include "status_bar.h"
#include "status_icons.h"
#include "sd_log.h"
#include "twatch_board.h"
#include "m10q.h"
#include "gpx_log.h"
#include "mesh_log.h"
#include "wifi_scan.h"
#include "ble_debug.h"
#include "sensor_cache.h"
#include "axp2101.h"
#include "alarm.h"

#define STATUS_COLOR_GREY   lv_color_hex(0x888888)
#define STATUS_COLOR_ORANGE lv_color_hex(0xFFB300)
#define STATUS_COLOR_GREEN  lv_color_hex(0x00E676)
#define STATUS_COLOR_RED    lv_color_hex(0xFF5252)

/* Row y and left/right-aligned x offsets are chosen from a measured safe-area
 * scan of assets/ui/safe_area_transparent.png (410x502 panel, rounded
 * corners physically clip/hide content there): at y~30 the corner cutout
 * requires roughly x >= 45 from either edge (straight-edge margin is ~16px,
 * but the corner radius is ~90-100px and dominates this close to the top) -
 * both rows' first column now starts at x=55 (see build_status_bar_big()),
 * clearing this with a few px to spare - see docs/application.md's "Abgerundete
 * Ecken beachten" section. Moved up further per explicit feedback ("move
 * them up") - the old value's other justification (clearing ring-screen
 * titles/the GPS switch row) is moot now that this is watch-face-only, no
 * other screen builds a status bar at all. Verified clear of the corner
 * mask at this height via the sim's safe-area overlay. */
#define STATUS_BAR_Y 30

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

/* GNSS/LoRa: small bitmap icons (status_icons.h), tinted via image
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
 * everything else (SD card, GPX file logging, charge state, battery).
 * Tightened from +44 to +34 per explicit feedback ("a small but distinct
 * margin") - the montserrat_28 icons/28px bitmap icons are ~30px tall, so
 * this still leaves a few px of visible gap, just not the more generous
 * spacing the original number left. */
#define STATUS_BAR_Y_ROW2 (STATUS_BAR_Y + 34)
void build_status_bar_big(lv_obj_t *parent, status_bar_t *out)
{
    const lv_font_t *f = &lv_font_montserrat_28;
    /* Row 1: 4-column grid, 90px pitch, per explicit feedback (BT was too
     * tight against the left edge at the original x=40/pitch=80). LoRa's
     * right edge at column 4 (325+28=353) stays clear of the x<=370
     * safe-area limit (see STATUS_BAR_Y's comment). */
    /* NFC joins this row (it is an RF-carrying radio, which is exactly what
     * this row groups) and sits in the middle per explicit feedback. Five
     * icons now instead of four, so the pitch drops 90 -> 68px: the outer
     * columns stay at the measured-safe x=55 / x=327 (mirror image, 28px
     * icon, 410px panel), and the middle column lands on x=191, whose icon
     * centre (191+14=205) is the exact panel centre - the same trick row 2's
     * bell uses. */
    out->bt   = status_icon_create_ex(parent, LV_SYMBOL_BLUETOOTH, LV_ALIGN_TOP_LEFT,  55, STATUS_BAR_Y,      f);
    out->wifi = status_icon_create_ex(parent, LV_SYMBOL_WIFI,      LV_ALIGN_TOP_LEFT, 123, STATUS_BAR_Y,      f);
    out->nfc  = status_image_create(parent, &status_icon_nfc,       LV_ALIGN_TOP_LEFT, 191, STATUS_BAR_Y);
    out->gps  = status_image_create(parent, &status_icon_satellite, LV_ALIGN_TOP_LEFT, 259, STATUS_BAR_Y);
    out->lora = status_image_create(parent, &status_icon_mesh,      LV_ALIGN_TOP_LEFT, 327, STATUS_BAR_Y);

    /* Row 2: 5-column grid (SD, GPX, Alarm, CHG, Batt), widened per explicit
     * feedback (SD/Batt allowed to sit close to the edges, like row 1's
     * original BT/LoRa spacing) - 73px pitch, SD's left edge at x=45 and
     * Batt's right edge at 337+28=365. Row 2 sits further from the top
     * corners than row 1 (STATUS_BAR_Y_ROW2 > STATUS_BAR_Y), so the corner
     * cutout demands less x-clearance here than row 1's measured x>=45 at
     * y~30 (see STATUS_BAR_Y's comment) - verified clear of the mask via
     * the sim's safe-area overlay. The middle column (Alarm) sits at
     * x=191, whose icon center (191+14=205) lands exactly on the panel's
     * own horizontal center (410/2). LVGL has no literal alarm-clock
     * glyph; LV_SYMBOL_BELL is the closest built-in. */
    out->sd    = status_icon_create_ex(parent, LV_SYMBOL_SD_CARD,   LV_ALIGN_TOP_LEFT, 45,  STATUS_BAR_Y_ROW2, f);
    out->gpx   = status_icon_create_ex(parent, LV_SYMBOL_GPS,       LV_ALIGN_TOP_LEFT, 118, STATUS_BAR_Y_ROW2, f);
    out->alarm = status_icon_create_ex(parent, LV_SYMBOL_BELL,      LV_ALIGN_TOP_LEFT, 191, STATUS_BAR_Y_ROW2, f);
    out->chg   = status_icon_create_ex(parent, LV_SYMBOL_CHARGE,    LV_ALIGN_TOP_LEFT, 264, STATUS_BAR_Y_ROW2, f);
    out->batt  = status_icon_create_ex(parent, LV_SYMBOL_BATTERY_EMPTY, LV_ALIGN_TOP_LEFT, 337, STATUS_BAR_Y_ROW2, f);
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

    /* NFC: reflects whether the reader's rail (DLDO1) is powered, which is
     * the only NFC state currently observable from here - nothing exposes a
     * "polling now" flag, and st25r3916.h is not part of the driver subset the
     * sim mirrors, so this deliberately does not reach into the driver. In
     * practice DLDO1 is left on for the whole session (see
     * axp2101_set_default_power()), so today this reads green permanently;
     * it turns grey by itself if that rail ever starts being gated. */
    bool nfc_on = false;
    axp2101_is_rail_enabled(twatch_pmu_dev, AXP2101_DLDO1, &nfc_on);
    lv_obj_set_style_image_recolor(bar->nfc, nfc_on ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);

    /* Reflects the WiFi screen's power switch (wifi_scan_set_enabled(),
     * default off - see wifi_scan.h). Green once switched on and at least
     * one network has been seen, orange while switched on but still
     * scanning/no networks found yet, grey while off - same
     * grey/orange/green convention as GPS's fix-state icon above. */
    wifi_scan_result_t wifi_probe[1];
    bool wifi_on = wifi_scan_get_enabled();
    bool wifi_has_results = wifi_on && wifi_scan_get_results(wifi_probe, 1) > 0;
    lv_obj_set_style_text_color(bar->wifi,
        wifi_has_results ? STATUS_COLOR_GREEN :
        wifi_on ? STATUS_COLOR_ORANGE : STATUS_COLOR_GREY, 0);

    sensor_cache_t cache;
    sensor_cache_get(&cache);
    if (cache.valid) {
        /* Icon only, no numeric percentage - per explicit feedback, the
         * number is moving elsewhere. The fill-level glyph alone still
         * conveys a rough state at a glance. */
        const char *icon = cache.batt_pct > 87 ? LV_SYMBOL_BATTERY_FULL :
                            cache.batt_pct > 62 ? LV_SYMBOL_BATTERY_3 :
                            cache.batt_pct > 37 ? LV_SYMBOL_BATTERY_2 :
                            cache.batt_pct > 12 ? LV_SYMBOL_BATTERY_1 : LV_SYMBOL_BATTERY_EMPTY;
        lv_label_set_text(bar->batt, icon);
        lv_obj_set_style_text_color(bar->batt, cache.batt_pct <= 15 ? STATUS_COLOR_RED : lv_color_hex(0xE0E0E0), 0);

        /* Always visible, colored like every other status icon here (grey
         * = off/absent, green = active) - was hidden outright when not
         * charging, the only icon in this row that did that instead of
         * just going grey (reported live: unplugging USB made the icon
         * vanish instead of leaving a grey flash behind). */
        bool charging = (cache.chg_state != AXP2101_CHG_STOP);
        lv_obj_set_style_text_color(bar->chg, charging ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY, 0);
    }

    /* Alarm: red while snoozing (most notable - it's about to ring again),
     * green while at least one alarm is armed (whether or not it's the one
     * currently snoozing, that case is already covered by red), grey when
     * none are set at all. Checked in that order since snoozing implies
     * armed. */
    lv_obj_set_style_text_color(bar->alarm, alarm_is_snoozing() ? STATUS_COLOR_RED :
                                (alarm_is_armed() ? STATUS_COLOR_GREEN : STATUS_COLOR_GREY), 0);
}

void status_bar_set_hidden(const status_bar_t *bar, bool hidden)
{
    if (!bar->sd) {
        return;
    }
    lv_obj_t *icons[] = { bar->sd, bar->gps, bar->gpx, bar->lora, bar->nfc,
                           bar->bt, bar->wifi, bar->batt, bar->chg, bar->alarm };
    for (size_t i = 0; i < sizeof(icons) / sizeof(icons[0]); i++) {
        if (hidden) {
            lv_obj_add_flag(icons[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(icons[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
