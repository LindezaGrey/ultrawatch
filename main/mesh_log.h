/*
 * mesh_log.h - always-on background Meshtastic message log.
 *
 * Starts the LoRa radio listening in the background at boot (RX Continuous,
 * ALDO3 stays powered across sleep - see power_mgmt.c) and keeps the last
 * few received packets in a RAM ring buffer for the UI (Mesh screen) to
 * read. Every packet with a valid header is kept, not just ones we can
 * decrypt and parse as text: `kind` tells the UI which case it is. Lost on
 * reboot - this is intentionally simple, not a persisted inbox.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MESH_LOG_COUNT     8
#define MESH_LOG_TEXT_MAX  200   /* generous vs. real Meshtastic text payloads */

/* Encryption is per-channel, not per-portnum (see meshtastic_packet.h) - a
 * NodeInfo or Telemetry packet is exactly as encrypted as a text message,
 * and we can only tell them apart *after* a successful decrypt. So the two
 * failure/success axes that actually matter for display are kept distinct:
 * "could we get anything readable out of this at all" (channel known and
 * decrypt+parse succeeded) vs. "was it specifically a text message". */
typedef enum {
    MESH_MSG_UNKNOWN = 0,   /* channel_hash matched no known channel, or the
                             * decrypt didn't parse as a valid Data message -
                             * text is empty, only from/channel_hash/rssi/snr
                             * are meaningful. */
    MESH_MSG_TEXT,          /* known channel, decrypted, TEXT_MESSAGE_APP -
                             * text is the message body. */
    MESH_MSG_OTHER,         /* known channel, decrypted, some other portnum -
                             * text is a short human-readable description
                             * (e.g. "NodeInfo: Kevin Hester", or a bare
                             * portnum name/number for anything we don't
                             * decode further). */
} mesh_msg_kind_t;

typedef struct {
    uint32_t from;                       /* sender node number */
    char text[MESH_LOG_TEXT_MAX + 1];    /* meaning depends on kind - see mesh_msg_kind_t */
    uint8_t channel_hash;                /* PacketHeader.channel byte, always valid */
    mesh_msg_kind_t kind;
    int16_t rssi_dbm;
    int8_t snr_db;
    int64_t received_at_us;              /* esp_timer_get_time() at receipt */
} mesh_msg_t;

/* Starts the background listener task. Call once after twatch_board_init(). */
void mesh_log_init(void);

/* Copies up to `max` of the most recent messages (newest first) into `out`.
 * Returns the number actually copied. */
size_t mesh_log_get_recent(mesh_msg_t *out, size_t max);

#ifdef __cplusplus
}
#endif
