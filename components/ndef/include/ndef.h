/*
 * ndef.h - NDEF message parsing over a Type 2 Tag's TLV area.
 *
 * Chip-agnostic: operates only on raw bytes a caller has already read from
 * a tag (e.g. st25r3916_read_type2()) - any reader could hand this the same
 * buffer. Scope is deliberately narrow: Type 2 Tag memory layout (NFC Forum
 * Type 2 Tag Operation spec) and the two NDEF well-known record types that
 * cover the overwhelming majority of real tags - Text (RTD-TEXT) and URI
 * (RTD-URI, including the standard prefix-abbreviation table). Anything
 * else (Smart Poster, MIME, external types) is reported as NDEF_OTHER with
 * its raw type/payload, not decoded - not a data loss, a documented
 * boundary to extend later if needed.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NDEF_TEXT,    /* well-known type "T" - text field decoded (UTF-8 only) */
    NDEF_URI,     /* well-known type "U" - prefix abbreviation expanded */
    NDEF_OTHER,   /* anything else - type[] names it, text[] holds a raw preview */
} ndef_record_kind_t;

typedef struct {
    ndef_record_kind_t kind;
    uint8_t tnf;        /* raw Type Name Format (record header bits 2:0) */
    char    type[16];   /* record type ("T", "U", a MIME type, ...), truncated, NUL-terminated */
    char    text[128];  /* decoded display string (TEXT/URI) or raw preview (OTHER), truncated, NUL-terminated */
} ndef_record_t;

/* Walks the Type 2 Tag TLV area in `data` (page 4 of tag memory onward, as
 * read by st25r3916_read_type2() - byte offset 0 of `data` is page 4 byte
 * 0) looking for the NDEF Message TLV (tag byte 0x03), then decodes its
 * records into `out` (up to `max_records`).
 *
 * Returns the number of records decoded - 0 if no NDEF Message TLV is
 * present, the TLV area doesn't parse, or the message has no records.
 * A return of 0 is a normal outcome for a blank/non-NDEF tag, not an
 * error - there is no error return; malformed input is treated the same
 * as "nothing found" rather than trusted partially. */
size_t ndef_parse(const uint8_t *data, size_t len, ndef_record_t *out, size_t max_records);

#ifdef __cplusplus
}
#endif
