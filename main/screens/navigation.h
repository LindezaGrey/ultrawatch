#pragma once
#include "lvgl.h"

typedef enum {
    NAV_WATCH, NAV_ACTIVITY, NAV_GPS, NAV_MESH, NAV_APPS,
    NAV_ALARMS, NAV_NFC, NAV_WIFI, NAV_BLE, NAV_SETTINGS,
    NAV_NODE, NAV_ALARM_EDIT, NAV_TIMER, NAV_RING, NAV_BHI,
    NAV_TZ, NAV_DISPLAY, NAV_PERIPHERALS, NAV_SOUND, NAV_INFO,
    NAV_SPARMODUS, NAV_VIBRATION, NAV_COUNT
} nav_screen_t;

/* All functions run under the LVGL lock. The clock must include time asleep. */
void navigation_init(lv_indev_t *input, uint32_t (*clock_ms)(void));
void navigation_open(nav_screen_t screen);
void navigation_back(void);
void navigation_note_activity(void);
void navigation_resume(void);
void navigation_poll(void);
nav_screen_t navigation_current(void);
bool navigation_swipe(int dx, int dy);
