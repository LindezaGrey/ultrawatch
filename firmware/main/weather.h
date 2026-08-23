#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define WEATHER_DAY_COUNT 7

typedef enum {
    WEATHER_IDLE = 0,
    WEATHER_LOADING,
    WEATHER_READY_ONLINE,
    WEATHER_READY_CACHE,
    WEATHER_NO_POSITION,
    WEATHER_NO_API_KEY,
    WEATHER_API_ACCESS_ERROR,
    WEATHER_NO_CACHE,
    WEATHER_NETWORK_ERROR,
    WEATHER_DATA_ERROR,
    WEATHER_SD_ERROR,
} weather_state_t;

typedef struct {
    int64_t timestamp;
    int16_t temperature_tenths;
    int16_t minimum_tenths;
    int16_t maximum_tenths;
    uint16_t condition_id;
} weather_day_t;

typedef struct {
    weather_state_t state;
    bool position_is_live;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t timezone_offset_seconds;
    int64_t cache_updated_at;
    uint8_t day_count;
    weather_day_t days[WEATHER_DAY_COUNT];
} weather_snapshot_t;

/* Start one asynchronous cache/network refresh. */
esp_err_t weather_start(void);

/* Copy the last published state without blocking display rendering. */
void weather_get_snapshot(weather_snapshot_t *snapshot);
