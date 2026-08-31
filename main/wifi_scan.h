/*
 * wifi_scan.h - on-demand WiFi scanning (WiFi screen's power switch +
 * network list). Off by default, same convention as the LoRa/mesh radio
 * (mesh_log.h) - the ESP32-S3's WiFi driver reserves internal DMA-capable
 * RAM the display's own draw buffers already compete for (see
 * night_mode_draw_bitmap()'s DMA-retry comment in lvgl_app.c, and the
 * BLE debug bridge's own similar note - "the BT controller reserves DMA
 * that the 48-row display buffer needs"), so this is deliberately never
 * running unless the user explicitly switches it on.
 *
 * Read-only: this scans and lists networks, it does not associate/connect
 * to any of them - no password entry, no station config beyond what
 * scanning itself needs.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_SCAN_MAX_RESULTS 16
#define WIFI_SCAN_SSID_MAX    32

typedef struct {
    char ssid[WIFI_SCAN_SSID_MAX + 1];
    int8_t rssi_dbm;
    bool open;   /* true if no auth (WIFI_AUTH_OPEN), false if secured */
} wifi_scan_result_t;

/* Call once at boot. Only sets up the (cheap, radio-inactive) netif/event
 * loop scaffolding esp_wifi_init() itself needs later - does not touch the
 * radio or start scanning. */
void wifi_scan_init(void);

bool wifi_scan_get_enabled(void);

/* Async, like mesh_log_set_enabled()/gps_power() - only sets the target
 * state; the background task applies it and does the actual
 * esp_wifi_init()/deinit() + start/stop transition, never blocking the
 * caller (UI thread). */
void wifi_scan_set_enabled(bool on);

/* True while a scan is actively in progress (for a "Scanning..." label). */
bool wifi_scan_is_scanning(void);

/* True if the last power-on attempt was refused for lack of free
 * DMA-capable RAM (see WIFI_SCAN_MIN_FREE_DMA_BYTES in wifi_scan.c) - the
 * switch is bounced back off in that case, same convention as
 * ble_scan_low_mem(). For a "Not enough memory right now" label. */
bool wifi_scan_low_mem(void);

/* Copies up to `max` most-recently-seen networks (by RSSI, strongest
 * first) into `out`. Returns the number actually copied. Safe to call at
 * any time, including while disabled (returns 0 - the cache is cleared on
 * disable). */
size_t wifi_scan_get_results(wifi_scan_result_t *out, size_t max);

#ifdef __cplusplus
}
#endif
