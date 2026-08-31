/*
 * ble_scan.h - on-demand BLE device scan (Bluetooth screen's power switch +
 * device list). Off by default. Independent of ble_debug.c's GATT-server
 * bridge (currently disabled - see uwatch_main.c's comment: the BT
 * controller reserves DMA that used to conflict with the display's draw
 * buffers). Enabling this switch brings the NimBLE host stack up the same
 * way ble_debug.c would have, so it carries the same real risk until
 * proven otherwise on hardware - not a theoretical one, this is the exact
 * failure class that got ble_debug disabled in the first place.
 *
 * Read-only: this scans and lists nearby devices (name if advertised,
 * address, RSSI), it does not connect/pair with any of them.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SCAN_MAX_RESULTS 20
#define BLE_SCAN_NAME_MAX    31

typedef struct {
    char name[BLE_SCAN_NAME_MAX + 1];   /* "" if not advertised */
    uint8_t addr[6];
    int8_t rssi_dbm;
} ble_scan_result_t;

/* Call once at boot. Only sets s_enabled from NVS (default off) - does not
 * touch the radio or bring up the NimBLE host stack. */
void ble_scan_init(void);

bool ble_scan_get_enabled(void);

/* Async, like wifi_scan_set_enabled()/mesh_log_set_enabled() - only sets
 * the target state; the background task applies it. Turning on starts a
 * single BLE_SCAN_DURATION_MS scan window and auto-turns back off (and
 * reflects that back via ble_scan_get_enabled()) once it completes -
 * unlike WiFi's continuous rescan, this is a one-shot bounded scan. */
void ble_scan_set_enabled(bool on);

/* True while a scan window is actively in progress (for a "Scanning..."
 * label). */
bool ble_scan_is_scanning(void);

/* True if the last enable attempt was refused because free internal
 * DMA-capable RAM was below BLE_SCAN_MIN_FREE_DMA_BYTES (see ble_scan.c) -
 * the BT controller's own init isn't PSRAM-eligible and fails outright
 * (confirmed live: BLE_INIT: Malloc failed -> watchdog panic) when it
 * doesn't have room, so this is checked before ever attempting it. Clears
 * on the next successful enable. For a "not enough memory, try again"
 * label - ble_scan_get_enabled() reverts to false in this case too, so
 * the switch itself doesn't stay stuck "on". */
bool ble_scan_low_mem(void);

/* Copies up to `max` devices seen during the most recent scan into `out`
 * (dedup'd by address). Returns the number actually copied. Safe to call
 * at any time, including while disabled (returns 0 - the cache is cleared
 * each time a new scan starts). */
size_t ble_scan_get_results(ble_scan_result_t *out, size_t max);

#ifdef __cplusplus
}
#endif
