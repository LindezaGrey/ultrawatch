#include "navigation.h"
#include "screens.h"
#include "power_mgmt.h"
#include "cascadia_fonts.h"
#include <stdlib.h>

static lv_obj_t *s_apps_screen;
static uint32_t (*s_clock)(void);
static uint32_t s_last_input;
static lv_point_t s_start;
static bool s_pressed;
static lv_indev_t *s_input;
static nav_screen_t s_press_screen;
static void build_apps(void);

typedef struct {
    lv_obj_t **screen;
    void (*build)(void);
    nav_screen_t parent;
    bool preserve;
} screen_entry_t;

/* One registry owns hierarchy and return-home policy in both builds. */
static const screen_entry_t screens[NAV_COUNT] = {
    [NAV_WATCH] = { &s_watch_screen, lvgl_build_watch_face, NAV_WATCH, true },
    [NAV_ACTIVITY] = { &s_activity_screen, lvgl_build_activity_screen, NAV_WATCH, false },
    [NAV_GPS] = { &s_gps_screen, lvgl_build_gps_screen, NAV_WATCH, false },
    [NAV_MESH] = { &s_mesh_screen, lvgl_build_mesh_screen, NAV_WATCH, false },
    [NAV_APPS] = { &s_apps_screen, build_apps, NAV_WATCH, false },
    [NAV_ALARMS] = { &s_alarm_screen, lvgl_build_alarm_screen, NAV_APPS, false },
    [NAV_NFC] = { &s_nfc_screen, lvgl_build_nfc_screen, NAV_APPS, false },
    [NAV_WIFI] = { &s_wifi_screen, lvgl_build_wifi_screen, NAV_APPS, false },
    [NAV_BLE] = { &s_ble_screen, lvgl_build_ble_screen, NAV_APPS, false },
    [NAV_SETTINGS] = { &s_settings_screen, lvgl_build_settings_screen, NAV_WATCH, true },
    [NAV_NODE] = { &s_node_screen, NULL, NAV_MESH, false },
    [NAV_ALARM_EDIT] = { &s_alarm_edit_screen, NULL, NAV_ALARMS, true },
    [NAV_TIMER] = { &s_timer_screen, NULL, NAV_ALARMS, true },
    [NAV_RING] = { &s_ring_screen, NULL, NAV_WATCH, true },
    [NAV_BHI] = { &s_bhi_screen, lvgl_build_bhi_screen, NAV_WATCH, false },
    [NAV_TZ] = { &s_set_tz_screen, NULL, NAV_SETTINGS, true },
    [NAV_DISPLAY] = { &s_set_disp_screen, NULL, NAV_SETTINGS, true },
    [NAV_PERIPHERALS] = { &s_set_periph_screen, NULL, NAV_SETTINGS, true },
    [NAV_SOUND] = { &s_set_sound_screen, NULL, NAV_SETTINGS, true },
    [NAV_INFO] = { &s_set_info_screen, NULL, NAV_SETTINGS, true },
    [NAV_SPARMODUS] = { &s_set_sparmodus_screen, NULL, NAV_SETTINGS, true },
    [NAV_VIBRATION] = { &s_set_vib_screen, NULL, NAV_SOUND, true },
};
static const nav_screen_t ring[] = { NAV_WATCH, NAV_ACTIVITY, NAV_GPS, NAV_MESH };

nav_screen_t navigation_current(void)
{
    lv_obj_t *active = lv_screen_active();
    for (int i = 0; i < NAV_COUNT; i++) {
        if (*screens[i].screen && active == *screens[i].screen) return i;
    }
    return NAV_COUNT;
}

void navigation_note_activity(void)
{
    if (s_clock) s_last_input = s_clock();
    power_mgmt_report_activity();
}

void navigation_open(nav_screen_t id)
{
    if ((unsigned)id >= NAV_COUNT) return;
    if (id == NAV_NODE) {
        lvgl_show_node_overview();
    } else {
        const screen_entry_t *entry = &screens[id];
        if (!*entry->screen && entry->build) entry->build();
        if (!*entry->screen) return;
        lv_screen_load(*entry->screen);
    }
    switch (id) {
    case NAV_WATCH: watch_face_update(NULL); break;
    case NAV_ACTIVITY: activity_screen_update(NULL); break;
    case NAV_GPS: gps_screen_update(NULL); break;
    case NAV_MESH: mesh_screen_update(NULL); break;
    case NAV_WIFI: wifi_screen_update(NULL); break;
    case NAV_BLE: ble_screen_update(NULL); break;
    case NAV_ALARMS: alarm_list_refresh(); break;
    case NAV_BHI: bhi_screen_update(NULL); break;
    default: break;
    }
    navigation_note_activity();
}

void navigation_back(void)
{
    nav_screen_t id = navigation_current();
    if (id < NAV_COUNT && id != NAV_RING) navigation_open(screens[id].parent);
}

static void app_click(lv_event_t *event)
{
    navigation_open((nav_screen_t)(uintptr_t)lv_event_get_user_data(event));
}

static void build_apps(void)
{
    s_apps_screen = screen_new();
    lv_obj_set_style_bg_color(s_apps_screen, lv_color_hex(0), 0);
    lv_obj_t *title = lv_label_create(s_apps_screen);
    lv_label_set_text(title, "APPS");
    lv_obj_set_style_text_font(title, &cascadia_36, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 36);
    const char *names[] = { "Alarms / Timer", "NFC", "Wi-Fi", "Bluetooth" };
    const nav_screen_t ids[] = { NAV_ALARMS, NAV_NFC, NAV_WIFI, NAV_BLE };
    for (int i = 0; i < 4; i++) {
        lv_obj_t *button = lv_button_create(s_apps_screen);
        lv_obj_set_size(button, 342, 70);
        lv_obj_align(button, LV_ALIGN_TOP_MID, 0, 104 + i * 82);
        lv_obj_set_style_bg_color(button, lv_color_hex(0x202830), 0);
        lv_obj_add_event_cb(button, app_click, LV_EVENT_CLICKED, (void *)(uintptr_t)ids[i]);
        lv_obj_t *label = lv_label_create(button);
        lv_label_set_text(label, names[i]);
        lv_obj_set_style_text_font(label, &cascadia_22, 0);
        lv_obj_center(label);
    }
    lv_obj_t *hint = lv_label_create(s_apps_screen);
    lv_label_set_text(hint, "Swipe down: Watch");
    lv_obj_set_style_text_font(hint, &cascadia_18, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0xaaaaaa), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -30);
}

bool navigation_swipe(int dx, int dy)
{
    if (abs(dx) < 60 && abs(dy) < 60) return false;
    nav_screen_t id = navigation_current();
    if (id == NAV_COUNT || id == NAV_RING) return false;
    if (abs(dx) > abs(dy)) {
        for (unsigned i = 0; i < sizeof(ring) / sizeof(ring[0]); i++) {
            if (id == ring[i]) {
                navigation_open(ring[(i + (dx < 0 ? 1 : 3)) % 4]);
                return true;
            }
        }
        if (dx > 0) {
            navigation_back();
            return true;
        }
    } else {
        if (id == NAV_WATCH) navigation_open(dy < 0 ? NAV_APPS : NAV_SETTINGS);
        else if (id == NAV_MESH && dy < 0) navigation_open(NAV_NODE);
        else if ((id == NAV_APPS && dy > 0) || (id == NAV_SETTINGS && dy < 0)) navigation_open(NAV_WATCH);
        else return false; /* Vertical scrolling belongs to widgets. */
        return true;
    }
    return false;
}

static void input_event(lv_event_t *event)
{
    lv_indev_t *input = lv_event_get_target(event);
    lv_point_t point;
    lv_indev_get_point(input, &point);
    lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_PRESSED) {
        s_start = point;
        s_pressed = true;
        s_press_screen = navigation_current();
        navigation_note_activity();
    } else if (code == LV_EVENT_LONG_PRESSED && s_press_screen == NAV_WATCH &&
               navigation_current() == NAV_WATCH && abs(point.x - s_start.x) < 20 &&
               abs(point.y - s_start.y) < 20) {
        lvgl_show_settings_disp();
        navigation_note_activity();
    } else if (code == LV_EVENT_RELEASED && s_pressed) {
        s_pressed = false;
        navigation_note_activity();
        if (s_press_screen == navigation_current() &&
            navigation_swipe(point.x - s_start.x, point.y - s_start.y)) {
            lv_indev_reset(input, NULL);
        }
    }
}

void navigation_poll(void)
{
    if (!s_clock) return;
    if (s_input && lv_indev_get_state(s_input) == LV_INDEV_STATE_PRESSED) {
        navigation_note_activity();
        return;
    }
    nav_screen_t id = navigation_current();
    if (id < NAV_COUNT && !screens[id].preserve &&
        s_clock() - s_last_input >= power_mgmt_get_return_timeout_s() * 1000) {
        /* An automatic transition must not extend the display's idle timer. */
        lv_screen_load(s_watch_screen);
        watch_face_update(NULL);
    }
}

void navigation_resume(void)
{
    s_pressed = false;
    navigation_poll(); /* Check the elapsed wall time before the first wake frame. */
}

static void timer_cb(lv_timer_t *timer)
{
    (void)timer;
    navigation_poll();
    power_mgmt_idle_poll();
}

void navigation_init(lv_indev_t *input, uint32_t (*clock_ms)(void))
{
    s_input = input;
    s_clock = clock_ms;
    navigation_note_activity();
    if (input) {
        lv_indev_add_event_cb(input, input_event, LV_EVENT_PRESSED, NULL);
        lv_indev_add_event_cb(input, input_event, LV_EVENT_RELEASED, NULL);
        lv_indev_add_event_cb(input, input_event, LV_EVENT_LONG_PRESSED, NULL);
    }
    lv_timer_create(timer_cb, 100, NULL);
}
