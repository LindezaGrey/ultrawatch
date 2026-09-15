#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the pinned ESP-Brookesia System Super stack over the registered watch HAL.
 * The bring-up app does not call this yet; native LVGL remains the runtime baseline
 * until the system stack passes its own device test. */
bool twatch_brookesia_system_start(void);
uint32_t twatch_brookesia_system_last_stage(void);

#ifdef __cplusplus
}
#endif
