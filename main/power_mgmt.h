/*
 * power_mgmt.h - watch power management (DFS + automatic light sleep).
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void power_mgmt_init(void);

/* Adapter auto-sleep callbacks (PAUSE mode). */
esp_err_t power_mgmt_enter_sleep(void *ctx);
esp_err_t power_mgmt_exit_sleep(void *ctx);

#ifdef __cplusplus
}
#endif
