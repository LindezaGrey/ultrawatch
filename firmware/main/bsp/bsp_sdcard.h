#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Power the SD rail (AXP2101 ALDO1), detect the card and mount the FAT
 * filesystem at BSP_SD_MOUNT_POINT. No-op if already mounted. */
esp_err_t bsp_sd_init(void);

esp_err_t bsp_sd_unmount(void);

bool bsp_sd_mounted(void);

/* True if a card is physically present (XL9555 detect pin, active low) */
bool bsp_sd_detect(void);

/* Last init step that failed, for diagnostics ("not attempted" initially) */
const char *bsp_sd_last_error(void);

uint32_t bsp_sd_capacity_mb(void);
uint32_t bsp_sd_free_mb(void);

#ifdef __cplusplus
}
#endif
