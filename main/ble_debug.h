/*
 * ble_debug.h - BLE debug bridge (NimBLE peripheral).
 *
 * Advertises as "UWatch" and exposes a GATT service for wireless debugging:
 *   - "cmd" characteristic (write): accepts any console command
 *   - "resp" characteristic (notify): console/command output is mirrored here
 *   - "telemetry" characteristic (read/notify): steps + GNSS snapshot
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the BLE stack and begin advertising. */
esp_err_t ble_debug_init(void);

/* Queue a console line to be handled by the command processor (called from the
 * GATT write callback). */
void ble_debug_run_command(const char *cmd);

/* Send a telemetry notification to a connected peer (if any). */
void ble_debug_notify_telemetry(void);

/* Debug: print BLE state (advertising / connection). */
void ble_debug_print_status(void);

/* True while a central is connected - the status bar's Bluetooth icon reuses
 * this (see docs/application.md's UI redesign): this bridge is a debug tool,
 * not a pairing feature, but its connection state is the only real BLE
 * signal that exists in this codebase today. */
bool ble_debug_is_connected(void);

/* Enable/disable BLE advertising at runtime (debug: BLE RF can desensitise
 * the GNSS front-end on this board). */
void ble_debug_set_advertising(bool on);

#ifdef __cplusplus
}
#endif
