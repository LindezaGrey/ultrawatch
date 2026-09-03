/*
 * system/crash_dump.h - decode a crash core dump from flash and save it to SD.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Check for a pending core dump in flash; if one exists and the SD card is
 * available, write a readable report + the raw ELF to /sdcard/log/crash/ and
 * erase the flash copy. Mounts the card itself on demand (sd_log_session_*,
 * see sd_log.h) - no prior sd_log_mount() call needed. */
esp_err_t crash_dump_save(void);

/* Print pending-core-dump status to the console (debug helper). */
void crash_dump_print_status(void);

#ifdef __cplusplus
}
#endif
