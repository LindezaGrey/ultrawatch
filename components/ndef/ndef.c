/*
 * ndef.c - see ndef.h for the whole-file rationale.
 *
 * Byte layouts below are per the NFC Forum Type 2 Tag Operation spec (TLV
 * area) and the NDEF / RTD-TEXT / RTD-URI specs (record framing, Text and
 * URI well-known payload formats) - all long-stable, unchanged for years.
 */
#include "ndef.h"
#include <stdbool.h>
#include <string.h>

/* ---- record header bits (byte 0 of every NDEF record) ---- */
#define NDEF_HDR_MB  0x80   /* Message Begin - unused here, single message assumed */
#define NDEF_HDR_ME  0x40   /* Message End - unused here */
#define NDEF_HDR_CF  0x20   /* Chunk Flag - chunked records not supported, see below */
#define NDEF_HDR_SR  0x10   /* Short Record: 1-byte payload length, not 4 */
#define NDEF_HDR_IL  0x08   /* ID Length field present */
#define NDEF_HDR_TNF 0x07   /* Type Name Format, bits 2:0 */

#define TNF_WELL_KNOWN 0x01

/* TLV tags (Type 2 Tag TLV area, page 4 onward). */
#define TLV_NULL       0x00   /* padding - no length/value field at all */
#define TLV_NDEF       0x03
#define TLV_TERMINATOR 0xFE   /* end of TLV area - no length/value field either */

/* RTD-URI 1.0 Table 3: URI Identifier Code -> prefix. 0x24-0xFF are RFU. */
static const char *const s_uri_prefix[] = {
    /* 0x00 */ "",
    /* 0x01 */ "http://www.",
    /* 0x02 */ "https://www.",
    /* 0x03 */ "http://",
    /* 0x04 */ "https://",
    /* 0x05 */ "tel:",
    /* 0x06 */ "mailto:",
    /* 0x07 */ "ftp://anonymous:anonymous@",
    /* 0x08 */ "ftp://ftp.",
    /* 0x09 */ "ftps://",
    /* 0x0A */ "sftp://",
    /* 0x0B */ "smb://",
    /* 0x0C */ "nfs://",
    /* 0x0D */ "ftp://",
    /* 0x0E */ "dav://",
    /* 0x0F */ "news:",
    /* 0x10 */ "telnet://",
    /* 0x11 */ "imap:",
    /* 0x12 */ "rtsp://",
    /* 0x13 */ "urn:",
    /* 0x14 */ "pop:",
    /* 0x15 */ "sip:",
    /* 0x16 */ "sips:",
    /* 0x17 */ "tftp:",
    /* 0x18 */ "btspp://",
    /* 0x19 */ "btl2cap://",
    /* 0x1A */ "btgoep://",
    /* 0x1B */ "tcpobex://",
    /* 0x1C */ "irdaobex://",
    /* 0x1D */ "file://",
    /* 0x1E */ "urn:epc:id:",
    /* 0x1F */ "urn:epc:tag:",
    /* 0x20 */ "urn:epc:pat:",
    /* 0x21 */ "urn:epc:raw:",
    /* 0x22 */ "urn:epc:",
    /* 0x23 */ "urn:nfc:",
};
#define URI_PREFIX_COUNT (sizeof(s_uri_prefix) / sizeof(s_uri_prefix[0]))

/* Copies up to dst_sz-1 bytes from a non-NUL-terminated byte range,
 * truncating silently (every field here is a fixed-size display buffer,
 * not a protocol boundary - truncation is the correct behavior, not an
 * error). Always NUL-terminates dst when dst_sz > 0. */
static void copy_truncate(char *dst, size_t dst_sz, const uint8_t *src, size_t src_len)
{
    if (dst_sz == 0) {
        return;
    }
    size_t n = (src_len < dst_sz - 1) ? src_len : dst_sz - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/* Locates the NDEF Message TLV's value in a Type 2 Tag TLV area. Returns
 * NULL if none is found (blank tag, corrupt TLV area, or genuinely absent -
 * all treated the same, per ndef.h's "no error return" contract). */
static const uint8_t *find_ndef_message(const uint8_t *data, size_t len, size_t *out_len)
{
    size_t pos = 0;
    while (pos < len) {
        uint8_t t = data[pos];
        if (t == TLV_NULL) {
            pos++;
            continue;
        }
        if (t == TLV_TERMINATOR) {
            break;
        }
        pos++;
        if (pos >= len) {
            break;
        }
        size_t l;
        if (data[pos] == 0xFF) {
            if (pos + 2 >= len) {
                break;
            }
            l = ((size_t)data[pos + 1] << 8) | data[pos + 2];
            pos += 3;
        } else {
            l = data[pos];
            pos += 1;
        }
        if (l > len - pos) {
            break;   /* declared length runs past what we actually have */
        }
        if (t == TLV_NDEF) {
            *out_len = l;
            return &data[pos];
        }
        pos += l;
    }
    *out_len = 0;
    return NULL;
}

/* RTD-TEXT 1.0: byte 0 = status (bit7 UTF16, bits5:0 language-code length),
 * then the language code, then the text itself. Only UTF-8 is decoded to a
 * display string - a UTF-16-flagged record is named, not mis-decoded. */
static void decode_text(const uint8_t *payload, size_t len, ndef_record_t *rec)
{
    if (len == 0) {
        rec->text[0] = '\0';
        return;
    }
    uint8_t status = payload[0];
    bool utf16 = (status & 0x80) != 0;
    size_t lang_len = status & 0x3F;
    size_t text_off = 1 + lang_len;
    if (text_off > len) {
        rec->text[0] = '\0';
        return;
    }
    if (utf16) {
        copy_truncate(rec->text, sizeof(rec->text),
                      (const uint8_t *)"(UTF-16 text, not decoded)", 26);
        return;
    }
    copy_truncate(rec->text, sizeof(rec->text), payload + text_off, len - text_off);
}

/* RTD-URI 1.0: byte 0 = URI Identifier Code (prefix abbreviation), the rest
 * is appended after the expanded prefix. */
static void decode_uri(const uint8_t *payload, size_t len, ndef_record_t *rec)
{
    if (len == 0) {
        rec->text[0] = '\0';
        return;
    }
    uint8_t code = payload[0];
    const char *prefix = (code < URI_PREFIX_COUNT) ? s_uri_prefix[code] : "";
    size_t prefix_len = strlen(prefix);
    size_t cap = sizeof(rec->text) - 1;
    size_t n = (prefix_len < cap) ? prefix_len : cap;
    memcpy(rec->text, prefix, n);
    size_t remaining = cap - n;
    size_t rest_len = len - 1;
    size_t take = (rest_len < remaining) ? rest_len : remaining;
    memcpy(rec->text + n, payload + 1, take);
    rec->text[n + take] = '\0';
}

size_t ndef_parse(const uint8_t *data, size_t len, ndef_record_t *out, size_t max_records)
{
    if (!data || !out || max_records == 0) {
        return 0;
    }

    size_t msg_len = 0;
    const uint8_t *msg = find_ndef_message(data, len, &msg_len);
    if (!msg || msg_len == 0) {
        return 0;
    }

    size_t pos = 0;
    size_t count = 0;
    while (pos < msg_len && count < max_records) {
        uint8_t hdr = msg[pos];
        pos++;

        /* Chunked records (a payload split across multiple record headers)
         * are not supported - genuinely rare for the short Text/URI
         * payloads this parser targets. Stop cleanly rather than
         * mis-assemble a chunk sequence as one wrong record. */
        if (hdr & NDEF_HDR_CF) {
            break;
        }

        if (pos >= msg_len) {
            break;
        }
        uint8_t type_len = msg[pos];
        pos++;

        size_t payload_len;
        if (hdr & NDEF_HDR_SR) {
            if (pos >= msg_len) {
                break;
            }
            payload_len = msg[pos];
            pos++;
        } else {
            if (4 > msg_len - pos) {
                break;
            }
            payload_len = ((size_t)msg[pos] << 24) | ((size_t)msg[pos + 1] << 16) |
                          ((size_t)msg[pos + 2] << 8) | (size_t)msg[pos + 3];
            pos += 4;
        }

        size_t id_len = 0;
        if (hdr & NDEF_HDR_IL) {
            if (pos >= msg_len) {
                break;
            }
            id_len = msg[pos];
            pos++;
        }

        if ((size_t)type_len > msg_len - pos) {
            break;
        }
        const uint8_t *type_ptr = &msg[pos];
        pos += type_len;

        if (id_len > msg_len - pos) {
            break;
        }
        pos += id_len;   /* ID is not used by this parser */

        if (payload_len > msg_len - pos) {
            break;
        }
        const uint8_t *payload_ptr = &msg[pos];
        pos += payload_len;

        ndef_record_t *rec = &out[count];
        memset(rec, 0, sizeof(*rec));
        rec->tnf = hdr & NDEF_HDR_TNF;
        copy_truncate(rec->type, sizeof(rec->type), type_ptr, type_len);

        if (rec->tnf == TNF_WELL_KNOWN && type_len == 1 && type_ptr[0] == 'T') {
            rec->kind = NDEF_TEXT;
            decode_text(payload_ptr, payload_len, rec);
        } else if (rec->tnf == TNF_WELL_KNOWN && type_len == 1 && type_ptr[0] == 'U') {
            rec->kind = NDEF_URI;
            decode_uri(payload_ptr, payload_len, rec);
        } else {
            rec->kind = NDEF_OTHER;
            copy_truncate(rec->text, sizeof(rec->text), payload_ptr, payload_len);
        }

        count++;
    }

    return count;
}
