/* Behavioral tests run against real LVGL and the shared screen/navigation code. */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "screens/screens.h"
#include "screens/navigation.h"
#include "mock_hw.h"

extern unsigned sim_display_activity_reports;
static uint32_t now;
static lv_point_t point;
static bool pressed;
static lv_indev_t *input;
static uint32_t clock_ms(void) { return now; }
static void read_input(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    data->point = point;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}
static void flush(lv_display_t *display, const lv_area_t *area, uint8_t *pixels)
{
    (void)area;
    (void)pixels;
    lv_display_flush_ready(display);
}
static void sample(int x, int y, bool down, uint32_t elapsed)
{
    now += elapsed;
    point = (lv_point_t){ x, y };
    pressed = down;
    lv_indev_read(input);
    lv_timer_handler();
}
static void swipe(int x, int y, int dx, int dy)
{
    lv_obj_update_layout(lv_screen_active());
    sample(x, y, true, 20);
    sample(x + dx, y + dy, true, 40);
    sample(x + dx, y + dy, false, 20);
}
static void expect(nav_screen_t expected)
{
    if (navigation_current() != expected) {
        fprintf(stderr, "expected screen %d, got %d at %u ms\n", expected, navigation_current(), now);
        assert(false);
    }
}

int navigation_test_run(void)
{
    lv_init();
    /* These code points previously resolved to missing-glyph boxes. */
    const uint32_t glyphs[] = {0xc4,0xd6,0xdc,0xe4,0xf6,0xfc,0xdf,
                              0x1f600,0x1f44d,0x2764,0x1f642,0x1f609,0x1f602,0x1f389};
    const lv_font_t *fonts[] = {&mesh_text_22, &mesh_unicode_18};
    for (unsigned f = 0; f < sizeof(fonts)/sizeof(fonts[0]); f++) {
        for (unsigned i = 0; i < sizeof(glyphs)/sizeof(glyphs[0]); i++) {
            lv_font_glyph_dsc_t glyph = {0};
            assert(lv_font_get_glyph_dsc(fonts[f], &glyph, glyphs[i], 0));
            assert(!glyph.is_placeholder && glyph.box_w && glyph.box_h);
        }
    }

    lv_tick_set_cb(clock_ms);
    lv_display_t *display = lv_display_create(410, 502);
    static uint8_t buffer[410 * 32 * 4];
    lv_display_set_buffers(display, buffer, NULL, sizeof(buffer), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, flush);
    input = lv_indev_create();
    lv_indev_set_type(input, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(input, read_input);
    lvgl_build_watch_face();
    navigation_init(input, clock_ms);
    navigation_open(NAV_WATCH);
    swipe(300, 250, -180, 0); expect(NAV_ACTIVITY);
    swipe(300, 250, -180, 0); expect(NAV_GPS);
    swipe(300, 250, -180, 0); expect(NAV_MESH);
    swipe(300, 250, -180, 0); expect(NAV_WATCH);
    swipe(100, 250, 180, 0); expect(NAV_MESH);
    swipe(205, 350, 0, -180); expect(NAV_NODE);
    swipe(100, 250, 180, 0); expect(NAV_MESH);
    navigation_open(NAV_WATCH);
    swipe(205, 350, 0, -180); expect(NAV_APPS);
    /* A real tap must open the first launcher row. */
    lv_obj_update_layout(lv_screen_active());
    sample(205, 138, true, 20);
    sample(205, 138, false, 20); expect(NAV_ALARMS);
    swipe(100, 250, 180, 0); expect(NAV_APPS);
    swipe(205, 150, 0, 180); expect(NAV_WATCH);
    swipe(205, 150, 0, 180); expect(NAV_SETTINGS);
    lvgl_show_settings_disp();
    swipe(100, 275, 180, 0); expect(NAV_SETTINGS);
    settings_open_subpage(3);
    settings_open_vib_screen();
    swipe(100, 280, 180, 0); expect(NAV_SOUND);
    swipe(100, 280, 180, 0); expect(NAV_SETTINGS);
    swipe(205, 350, 0, -180); expect(NAV_WATCH);

    /* Long press opens Display, then a real Back tap returns to Settings. */
    navigation_open(NAV_WATCH);
    lv_obj_update_layout(lv_screen_active());
    sample(205, 250, true, 20);
    sample(205, 250, true, 500); expect(NAV_DISPLAY);
    sample(205, 250, false, 20);
    lv_obj_update_layout(lv_screen_active());
    sample(75, 58, true, 20);
    sample(75, 58, false, 20); expect(NAV_SETTINGS);
    /* Visiting GPS while GNSS is off does not renew display activity. */
    navigation_open(NAV_GPS);
    unsigned gps_reports = sim_display_activity_reports;
    gps_screen_update(NULL);
    assert(sim_display_activity_reports == gps_reports);

    navigation_open(NAV_ACTIVITY);
    now += 29999;
    navigation_poll(); expect(NAV_ACTIVITY);
    now++;
    unsigned reports = sim_display_activity_reports;
    navigation_poll(); expect(NAV_WATCH);
    assert(sim_display_activity_reports == reports);
    /* Quick wake resumes; late wake returns before the next rendered frame. */
    navigation_open(NAV_GPS);
    now += 10000;
    navigation_resume(); expect(NAV_GPS);
    now += 20000;
    navigation_resume(); expect(NAV_WATCH);
    settings_open_subpage(5);
    now += 120000;
    navigation_resume(); expect(NAV_SPARMODUS);
    navigation_open(NAV_ALARMS);
    lvgl_build_alarm_edit_screen();
    lv_screen_load(s_alarm_edit_screen);
    now += 120000;
    navigation_resume(); expect(NAV_ALARM_EDIT);
    navigation_back(); expect(NAV_ALARMS);
    lvgl_build_timer_screen();
    lv_screen_load(s_timer_screen);
    now += 120000;
    navigation_resume(); expect(NAV_TIMER);
    navigation_back(); expect(NAV_ALARMS);
    navigation_open(NAV_NFC);
    swipe(100, 250, 180, 0); expect(NAV_APPS);

    /* Runtime return setting and clock rollover. */
    power_mgmt_set_return_timeout_s(15);
    now = UINT32_MAX - 5000;
    navigation_open(NAV_WIFI);
    now += 14999;
    navigation_poll(); expect(NAV_WIFI);
    now++;
    navigation_poll(); expect(NAV_WATCH);
    /* A held touch must never count as idle. */
    navigation_open(NAV_ACTIVITY);
    sample(205, 250, true, 20);
    now += 20000;
    navigation_poll(); expect(NAV_ACTIVITY);
    sample(205, 250, false, 20);
    navigation_poll(); expect(NAV_ACTIVITY);
    puts("navigation: gestures, launcher tap, parent back, timeout, wake, preservation, hold and rollover passed");
    return 0;
}
