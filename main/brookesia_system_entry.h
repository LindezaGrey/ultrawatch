#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the pinned ESP-Brookesia System Super stack over the registered watch HAL.
 * The bring-up app does not call this yet; native LVGL remains the runtime baseline
 * until the system stack passes its own device test. */
bool twatch_brookesia_system_start(void);
uint32_t twatch_brookesia_system_last_stage(void);
/* Startup-only service bind/release test; completes before native LVGL starts. */
bool twatch_brookesia_services_probe(void);

#ifdef __cplusplus
}
#endif
