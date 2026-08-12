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

#ifdef __cplusplus
}
#endif
