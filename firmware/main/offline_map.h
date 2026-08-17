#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define OFFLINE_MAP_WIDTH 410
#define OFFLINE_MAP_HEIGHT 502

typedef enum {
    OFFLINE_MAP_STOPPED,
    OFFLINE_MAP_LOADING,
    OFFLINE_MAP_GPS_SEARCH,
    OFFLINE_MAP_READY,
    OFFLINE_MAP_ERROR,
} offline_map_state_t;

typedef struct {
    offline_map_state_t state;
    bool following;
    bool gps_fix;
    bool can_zoom_in;
    bool can_zoom_out;
    uint8_t zoom;
    int marker_x;
    int marker_y;
    char error[32];
} offline_map_snapshot_t;

esp_err_t offline_map_start(void);
void offline_map_stop(void);
bool offline_map_is_active(void);
void offline_map_pan(int screen_dx, int screen_dy);
void offline_map_pan_step(int direction_x, int direction_y);
void offline_map_zoom(int direction);
void offline_map_recenter(void);
bool offline_map_copy_frame(uint16_t *destination,
                            offline_map_snapshot_t *snapshot);
