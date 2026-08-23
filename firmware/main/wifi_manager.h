#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    WATCH_WIFI_OFF = 0,
    WATCH_WIFI_LOADING = 1,
    WATCH_WIFI_CONNECTING = 2,
    WATCH_WIFI_CONNECTED = 3,
    WATCH_WIFI_DISCONNECTED = 4,
    WATCH_WIFI_SD_ERROR = 5,
    WATCH_WIFI_CONFIG_ERROR = 6,
    WATCH_WIFI_DRIVER_ERROR = 7,
} watch_wifi_state_t;

typedef enum {
    WATCH_WIFI_ERROR_NONE = 0,
    WATCH_WIFI_ERROR_NO_CARD = 1,
    WATCH_WIFI_ERROR_FILE_MISSING = 2,
    WATCH_WIFI_ERROR_INVALID_CONFIG = 3,
    WATCH_WIFI_ERROR_NO_PROFILES = 4,
    WATCH_WIFI_ERROR_CONNECTIONS_FAILED = 5,
    WATCH_WIFI_ERROR_DRIVER = 6,
} watch_wifi_error_t;

typedef struct {
    bool requested;
    watch_wifi_state_t state;
    watch_wifi_error_t error;
    int8_t rssi;
    uint8_t ipv4[4];
    uint32_t active_profile;
} watch_wifi_status_t;

typedef struct {
    char ssid[33];
    char password[65];
} watch_wifi_profile_t;

typedef void (*watch_wifi_status_callback_t)(void);

void watch_wifi_get_status(watch_wifi_status_t *status);
void watch_wifi_set_status_callback(watch_wifi_status_callback_t callback);
/* Wait until the manager publishes another state. */
void watch_wifi_wait_for_status_change(void);
/* Start Wi-Fi when off. Do not restart an active connection or pass. */
esp_err_t watch_wifi_ensure_enabled(void);
esp_err_t watch_wifi_set_enabled(bool enabled);
/* Keep the Wi-Fi PHY awake while a short network transfer is in progress. */
esp_err_t watch_wifi_set_transfer_active(bool active);

esp_err_t watch_wifi_profile_count(uint32_t *count);
esp_err_t watch_wifi_profile_get(uint32_t index, watch_wifi_profile_t *profile);
esp_err_t watch_wifi_profile_put(uint32_t index, bool append,
                                 const watch_wifi_profile_t *profile);
esp_err_t watch_wifi_profile_delete(uint32_t index);
esp_err_t watch_wifi_profile_move(uint32_t index, uint32_t destination);
