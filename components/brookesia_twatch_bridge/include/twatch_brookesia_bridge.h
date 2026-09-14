#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Acquire both verified display interfaces once to validate registration. */
bool twatch_brookesia_bridge_ready(void);

#ifdef __cplusplus
}
#endif
