#include "meshtastic_packet.h"
#include "meshtastic_crypto.h"
#include <string.h>

#define HEADER_LEN 16

typedef struct {
    const char *name;
    const uint8_t *psk;
    size_t psk_len;
    uint8_t hash;
} known_channel_t;

/* Every channel this watch can decrypt, PSK + precomputed hash (see
 * meshtastic_crypto.h for how each hash was derived). Add more channels
 * here as entries - the loop below tries them in order. */
static const known_channel_t s_known_channels[] = {
    { "ShortSlow",   meshtastic_default_psk, sizeof(meshtastic_default_psk), MESHTASTIC_DEFAULT_CHANNEL_HASH },
    { "Mesh Hessen", meshtastic_hessen_psk,  sizeof(meshtastic_hessen_psk),  MESHTASTIC_HESSEN_CHANNEL_HASH },
};
#define NUM_KNOWN_CHANNELS (sizeof(s_known_channels) / sizeof(s_known_channels[0]))

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
    for (size_t i = 0; i < NUM_KNOWN_CHANNELS; i++) {
        if (s_known_channels[i].hash == out->channel_hash) {
            ch = &s_known_channels[i];
            break;
        }
    }
    if (!ch) {
        return true;   /* valid header, but not a channel we have the key for */
    }
    out->channel_hash_matches = true;
    out->channel_name = ch->name;

    uint8_t *payload = buf + HEADER_LEN;
    size_t payload_len = len - HEADER_LEN;
    if (meshtastic_crypto_ctr(ch->psk, ch->psk_len, out->from, out->id, payload, payload_len) != ESP_OK) {
        return true;   /* valid header, but couldn't decrypt - data left empty */
    }

    meshtastic_data_decode(payload, payload_len, &out->data);   /* best-effort */
    return true;
}
