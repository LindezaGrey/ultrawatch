#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    bool mounted;
    bool space_valid;
    int32_t last_error;
    uint64_t total_bytes, free_bytes;
} sd_log_status_t;
