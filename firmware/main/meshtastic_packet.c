#include "meshtastic_packet.h"
#include "meshtastic_crypto.h"
#include <string.h>

#define HEADER_LEN 16

typedef struct {
    meshtastic_channel_t channel;
    uint8_t hash;
} known_channel_t;

static known_channel_t s_known_channels[MESHTASTIC_CHANNEL_COUNT];
static size_t s_known_channel_count;

static uint8_t xor_hash(const uint8_t *data, size_t length)
{
    uint8_t value = 0;
    for (size_t index = 0; index < length; index++) value ^= data[index];
    return value;
}

bool meshtastic_packet_set_channels(const meshtastic_channel_t *channels,
                                    size_t count)
{
    if (channels == NULL || count > MESHTASTIC_CHANNEL_COUNT) return false;
    for (size_t index = 0; index < count; index++) {
        const meshtastic_channel_t *source = &channels[index];
        if ((source->key_len != 16 && source->key_len != 32) ||
            source->name[0] == '\0') {
            return false;
        }
        s_known_channels[index].channel = *source;
        s_known_channels[index].channel.name[MESHTASTIC_CHANNEL_NAME_MAX] = '\0';
        s_known_channels[index].hash =
            xor_hash((const uint8_t *)source->name, strlen(source->name)) ^
            xor_hash(source->key, source->key_len);
    }
    s_known_channel_count = count;
    return true;
}

bool meshtastic_packet_decode(uint8_t *buf, size_t len, meshtastic_packet_t *out)
{
    memset(out, 0, sizeof(*out));
    if (len < HEADER_LEN) {
        return false;
    }

    /* PacketHeader: to,from,id (u32 LE each), flags, channel, next_hop,
     * relay_node - the sender's struct is native little-endian (same as
     * this ESP32-S3 target), sent as raw bytes, so a plain memcpy
     * reproduces the intended values with no byte-swapping needed. */
    memcpy(&out->to, buf + 0, 4);
    memcpy(&out->from, buf + 4, 4);
    memcpy(&out->id, buf + 8, 4);
    out->hop_limit = buf[12] & 0x07;
    out->channel_hash = buf[13];
    /* buf[14]=next_hop, buf[15]=relay_node - not surfaced yet, no current use */
    /* to/from/id/hop_limit/channel_hash above are now valid regardless of
     * whether a known channel matches below - callers that want to show
     * "saw a packet, don't know what channel" can do so from just these. */

    const known_channel_t *ch = NULL;
    for (size_t i = 0; i < s_known_channel_count; i++) {
        if (s_known_channels[i].hash == out->channel_hash) {
            ch = &s_known_channels[i];
            break;
        }
    }
    if (!ch) {
        return true;   /* valid header, but not a channel we have the key for */
    }
    out->channel_hash_matches = true;
    out->channel_name = ch->channel.name;

    uint8_t *payload = buf + HEADER_LEN;
    size_t payload_len = len - HEADER_LEN;
    if (meshtastic_crypto_ctr(ch->channel.key, ch->channel.key_len,
                             out->from, out->id, payload, payload_len) != ESP_OK) {
        return true;   /* valid header, but couldn't decrypt - data left empty */
    }

    meshtastic_data_decode(payload, payload_len, &out->data);   /* best-effort */
    return true;
}
