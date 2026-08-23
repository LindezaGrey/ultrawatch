#pragma once

#include <stdbool.h>

#include "esp_err.h"

/* Acquire the shared FAT32 mount at /sdcard and keep ALDO1 powered. */
esp_err_t sd_storage_acquire(void);

/* Release one client. The last client unmounts the card and disables ALDO1. */
void sd_storage_release(void);

/* Read the active-low card-detect input without mounting the card. */
bool sd_storage_card_present(void);

