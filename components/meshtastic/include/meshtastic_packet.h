/*
 * meshtastic_packet.h - decode a raw over-the-air Meshtastic LoRa frame.
 *
 * PacketHeader layout verified against meshtastic/firmware's
 * RadioInterface.h (the struct is sent as raw bytes over the radio link,
 * comment: "has to exactly match the wire layout").
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "meshtastic_data.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t to;
    uint32_t from;
    uint32_t id;
    uint8_t hop_limit;              /* bottom 3 bits of the header's flags byte */
    uint8_t channel_hash;
    bool channel_hash_matches;      /* true if channel_hash matches one of our known channels */
    const char *channel_name;       /* name of the matched channel, or NULL if unmatched */
    meshtastic_data_t data;         /* only populated when channel_hash_matches and
                                      * decrypt+parse both succeeded; zeroed otherwise */
} meshtastic_packet_t;

/* Decodes `buf` (raw bytes as received over the air) in place - parses the
 * 16-byte PacketHeader always (to/from/id/hop_limit/channel_hash are valid
 * for every structurally-valid packet, whether or not we can decrypt it),
 * then, only if the header's channel byte matches one of our known channels
 * (see meshtastic_crypto.h), AES-CTR-decrypts the rest with that channel's
 * key and parses it as a Data protobuf message. Returns false only if buf is
 * shorter than the fixed header; true otherwise, even when data ends up
 * empty (unknown channel, or a decrypt that didn't parse as valid protobuf -
 * check channel_hash_matches and data.payload to tell those apart). */
bool meshtastic_packet_decode(uint8_t *buf, size_t len, meshtastic_packet_t *out);

#ifdef __cplusplus
}
#endif
