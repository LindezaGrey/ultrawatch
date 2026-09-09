/* Screenshot shortcuts; all user navigation lives in screens/navigation.c. */
#include <stdio.h>
#include <SDL2/SDL.h>
#include "screens.h"
#include "screens/screens.h"
#include "screens/navigation.h"
#include "nav.h"

void sim_show_watch_face(void)
{
    navigation_open(NAV_WATCH);
}

void sim_activity_screen_build(void)
{
    navigation_open(NAV_ACTIVITY);
}

void sim_gps_screen_build(void)
{
    navigation_open(NAV_GPS);
}

void sim_mesh_screen_build(void)
{
    navigation_open(NAV_MESH);
}

void sim_wifi_screen_build(void)
{
    navigation_open(NAV_WIFI);
}

void sim_ble_screen_build(void)
{
    navigation_open(NAV_BLE);
}

void sim_node_screen_build(void)
{
    navigation_open(NAV_NODE);
}

void sim_settings_screen_build(void)
{
    navigation_open(NAV_SETTINGS);
}

void sim_settings_disp_screen_build(void)
{
    lvgl_show_settings_disp();
}

void sim_settings_tz_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(0);
}

void sim_settings_periph_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(2);
}

void sim_settings_sound_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(3);
}

void sim_settings_vib_screen_build(void)
{
    sim_settings_sound_screen_build();
    settings_open_vib_screen();
}

void sim_settings_info_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(4);
}

void sim_settings_sparmodus_screen_build(void)
{
    sim_settings_screen_build();
    settings_open_subpage(5);
}

void sim_bhi_screen_build(void)
{
    navigation_open(NAV_BHI);
}

void sim_alarm_screen_build(void)
{
    navigation_open(NAV_ALARMS);
}

void sim_alarm_edit_screen_build(void)
{
    if (!s_alarm_edit_screen) {
        lvgl_build_alarm_edit_screen();
    }
    lv_scr_load(s_alarm_edit_screen);
}

void sim_timer_screen_build(void)
{
    if (!s_timer_screen) {
        lvgl_build_timer_screen();
    }
    lv_scr_load(s_timer_screen);
}

void sim_ring_screen_build(alarm_ring_source_t source)
{
    if (!s_ring_screen) {
        lvgl_build_ring_screen();
    }
    if (source == ALARM_RING_SOURCE_TIMER) {
        lv_label_set_text(s_ring_title_label, "TIMER");
        lv_label_set_text(s_ring_time_label, "Done");
        lv_obj_add_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_label_set_text(s_ring_title_label, "ALARM");
                alarm_entry_t list[ALARM_MAX_COUNT];
        alarm_get_all(list, ALARM_MAX_COUNT);
        char buf[8] = "--:--";
        for (int i = 0; i < ALARM_MAX_COUNT; i++) {
            if (list[i].in_use) {
                snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)list[i].hour,
                         (unsigned)list[i].min);
                break;
            }
        }
        lv_label_set_text(s_ring_time_label, buf);
        lv_obj_clear_flag(s_ring_snooze_btn, LV_OBJ_FLAG_HIDDEN);
    }
    lv_scr_load(s_ring_screen);
}

void sim_nav_init(lv_indev_t *input)
{
    navigation_init(input, SDL_GetTicks);
}

static void nfc_preview_start(lv_event_t *event)
{
    (void)event;
    lv_label_set_text(s_nfc_status_label, "Preview only");
    lv_label_set_text(s_nfc_uid_label, "NFC scanning requires the watch");
}

void lvgl_build_nfc_screen(void)
{
    nfc_screen_create(nfc_preview_start);
    lv_label_set_text(s_nfc_status_label, "Ready");
    lv_label_set_text(s_nfc_uid_label, "Tap Start, then hold a tag near the watch");
}
