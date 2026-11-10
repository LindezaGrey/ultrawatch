/*
 * meshtastic_data.h - pure protobuf field-walker for Meshtastic's Data
 * message (mesh.proto). No ESP-IDF/crypto dependency - operates on an
 * already-decrypted buffer, host-testable.
 *
 * Only extracts the two fields the console decoder needs (portnum,
 * payload); Data's other fields (want_response, dest, source, request_id,
 * reply_id, emoji, bitfield, xeddsa_signature) are correctly skipped per
 * their wire type, not because they don't exist but because nothing here
 * uses them yet - protobuf's forward-compatible wire format means skipping
 * an unknown/unused field is always safe, never a parse error.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t portnum;         /* PortNum enum value; UNKNOWN_APP(0) if absent */
    const uint8_t *payload;   /* points into the input buffer, not copied */
    size_t payload_len;
} meshtastic_data_t;

/* Parses a decrypted Meshtastic Data protobuf message. Returns false only
 * on a malformed/truncated encoding (bad varint, a length-delimited field
 * running past the buffer end, an invalid wire type); true otherwise, even
 * if portnum/payload were never present in the input (protobuf semantics:
 * every field is optional, so an empty message is valid, not an error). */
bool meshtastic_data_decode(const uint8_t *buf, size_t len, meshtastic_data_t *out);

/* Human-readable name for a PortNum enum value (meshtastic/protobufs'
 * portnums.proto) - just the values worth labeling; NULL for anything else
 * (caller falls back to printing the bare number). */
const char *meshtastic_portnum_name(uint32_t portnum);
