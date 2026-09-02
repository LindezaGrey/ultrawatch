/*
 * meshtastic_user.h - pure protobuf field-walker for Meshtastic's User
 * message (mesh.proto), the payload of a NODEINFO_APP (portnum 4) packet.
 * No ESP-IDF/crypto dependency - operates on an already-decrypted buffer,
 * same shape as meshtastic_data.c.
 *
 * Only extracts long_name/short_name; User's other fields (id, macaddr,
 * hw_model, is_licensed, role, public_key, is_unmessagable) are correctly
 * skipped per their wire type - nothing here uses them yet.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *long_name;    /* points into the input buffer, not copied; NULL if absent */
    size_t long_name_len;
    const char *short_name;   /* points into the input buffer, not copied; NULL if absent */
    size_t short_name_len;
} meshtastic_user_t;

/* Parses a decrypted Meshtastic User protobuf message (the NODEINFO_APP
 * payload). Returns false only on a malformed/truncated encoding; true
 * otherwise, even if long_name/short_name were never present (both are
 * optional per protobuf semantics). */
bool meshtastic_user_decode(const uint8_t *buf, size_t len, meshtastic_user_t *out);
