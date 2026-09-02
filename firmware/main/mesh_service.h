#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "meshtastic_packet.h"

#define MESH_MESSAGE_COUNT 8
#define MESH_MESSAGE_TEXT_MAX 160
#define MESH_CONFIG_WIRE_VERSION 1
#define MESH_CONFIG_WIRE_LENGTH 145

typedef enum {
    MESH_MESSAGE_UNKNOWN = 0,
    MESH_MESSAGE_TEXT = 1,
    MESH_MESSAGE_OTHER = 2,
} mesh_message_kind_t;

typedef struct {
    uint32_t from;
    char sender[32];
    char text[MESH_MESSAGE_TEXT_MAX + 1];
    char channel[MESHTASTIC_CHANNEL_NAME_MAX + 1];
    uint8_t channel_hash;
    mesh_message_kind_t kind;
    int16_t rssi_dbm;
    int8_t snr_db;
    int64_t received_at_us;
} mesh_message_t;

typedef struct {
    bool enabled;
    bool notify;
    uint32_t frequency_hz;
    uint8_t spreading_factor;
    uint8_t bandwidth;
    uint8_t coding_rate;
    bool low_data_rate_optimize;
    uint16_t preamble_length;
    uint8_t sync_word;
    bool boosted_rx_gain;
    meshtastic_channel_t channels[MESHTASTIC_CHANNEL_COUNT];
} mesh_config_t;

esp_err_t mesh_service_init(void);
void mesh_service_get_config(mesh_config_t *config);
esp_err_t mesh_service_set_config(const mesh_config_t *config);
bool mesh_service_radio_active(void);
size_t mesh_service_get_recent(mesh_message_t *messages, size_t maximum);

void mesh_config_encode(const mesh_config_t *config,
                        uint8_t output[MESH_CONFIG_WIRE_LENGTH]);
bool mesh_config_decode(const uint8_t input[MESH_CONFIG_WIRE_LENGTH],
                        mesh_config_t *config);
