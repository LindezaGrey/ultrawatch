/*
 * uwatch_main.h - UWatch application entry point + shared helpers.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Execute one console command (shared by the USB console and the BLE debug
 * bridge). */
void uwatch_debug_process_cmd(const char *cmd);

#ifdef __cplusplus
}
#endif
