#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the power logger. Mount is attempted lazily by powerlog_tick(). */
esp_err_t powerlog_init(void);

/* Call from the UI timer every second; logs one line per interval. */
void powerlog_tick(void);

#ifdef __cplusplus
}
#endif
