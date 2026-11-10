/*
 * meshtastic_crypto.h - AES-CTR encrypt/decrypt for Meshtastic packets.
 *
 * Verified against meshtastic/firmware's CryptoEngine.cpp/Channels.h.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Meshtastic's default public channel PSK (16 bytes -> AES-128). Verified
 * against meshtastic/firmware's Channels.h `defaultpsk` array - used
 * unmodified for PSK index 1 ("AQ=="), the primary channel every device
 * ships with by default. */
extern const uint8_t meshtastic_default_psk[16];

/* This channel's PacketHeader.channel hash byte, precomputed from
 * xorHash(name) ^ xorHash(meshtastic_default_psk) - verified against
 * meshtastic/firmware's Channels::generateHash()/getName(). The default
 * channel has no explicit name, so `name` here is the active modem preset's
 * display name (Channels::getName()'s empty-name fallback) - "ShortSlow"
 * for the SHORT_SLOW preset this radio now runs (was "LongFast" -> 0x08
 * before switching presets; see meshtastic_radio.c). Recompute this value
 * again if the preset changes. A received packet whose header channel byte
 * doesn't match this almost certainly isn't using this key - decrypting it
 * anyway would just produce garbage, since AES-CTR has no built-in
 * integrity check to catch a wrong key on its own. */
#define MESHTASTIC_DEFAULT_CHANNEL_HASH 0x77

/* User's private "Mesh Hessen" channel PSK (32 bytes -> AES-256), same
 * PHY settings as the default channel (whatever modem preset is active -
 * see meshtastic_radio.c), different key. */
extern const uint8_t meshtastic_hessen_psk[32];

/* xorHash("Mesh Hessen") ^ xorHash(meshtastic_hessen_psk), same formula as
 * MESHTASTIC_DEFAULT_CHANNEL_HASH above. */
#define MESHTASTIC_HESSEN_CHANNEL_HASH 0x40

/* AES-CTR encrypt/decrypt in place - CTR mode is symmetric, so this is the
 * same operation either way, matching CryptoEngine::decrypt() just calling
 * encryptPacket() internally. from_node/packet_id come from the packet's
 * PacketHeader (from/id fields). Nonce = packet_id zero-extended to 8
 * bytes LE, then from_node 4 bytes LE, then a 4-byte block counter
 * starting at zero - verified against CryptoEngine::initNonce(). key_len
 * must be 16 (AES-128) or 32 (AES-256). */
esp_err_t meshtastic_crypto_ctr(const uint8_t *key, size_t key_len,
                                uint32_t from_node, uint32_t packet_id,
                                uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
