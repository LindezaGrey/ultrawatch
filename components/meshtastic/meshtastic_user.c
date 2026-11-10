#include "meshtastic_user.h"
#include <string.h>

/* Standard protobuf varint (LEB128): 7 bits per byte, MSB = continuation. */
static bool read_varint(const uint8_t *buf, size_t len, size_t *pos, uint64_t *out)
{
    uint64_t result = 0;
    int shift = 0;
    while (*pos < len) {
        uint8_t b = buf[(*pos)++];
        result |= (uint64_t)(b & 0x7F) << shift;
        if ((b & 0x80) == 0) {
            *out = result;
            return true;
        }
        shift += 7;
        if (shift >= 64) {
            return false;   /* malformed: varint longer than 64 bits */
        }
    }
    return false;   /* truncated: ran out of buffer mid-varint */
}

bool meshtastic_user_decode(const uint8_t *buf, size_t len, meshtastic_user_t *out)
{
    memset(out, 0, sizeof(*out));
    size_t pos = 0;

    while (pos < len) {
        uint64_t tag;
        if (!read_varint(buf, len, &pos, &tag)) {
            return false;
        }
        uint32_t field_num = (uint32_t)(tag >> 3);
        uint32_t wire_type = (uint32_t)(tag & 0x7);

        switch (wire_type) {
        case 0: {   /* varint (hw_model, is_licensed, role, is_unmessagable) */
            uint64_t value;
            if (!read_varint(buf, len, &pos, &value)) {
                return false;
            }
            (void)value;   /* not used by any field this decoder cares about */
            break;
        }
        case 1:   /* 64-bit fixed - not present in User, but skip safely if seen */
            if (pos + 8 > len) {
                return false;
            }
            pos += 8;
            break;
        case 2: {   /* length-delimited (id/long_name/short_name/macaddr/public_key) */
            uint64_t field_len;
            if (!read_varint(buf, len, &pos, &field_len)) {
                return false;
            }
            if (field_len > len - pos) {
                return false;
            }
            if (field_num == 2) {   /* long_name */
                out->long_name = (const char *)(buf + pos);
                out->long_name_len = (size_t)field_len;
            } else if (field_num == 3) {   /* short_name */
                out->short_name = (const char *)(buf + pos);
                out->short_name_len = (size_t)field_len;
            }
            pos += (size_t)field_len;
            break;
        }
        case 5:   /* 32-bit fixed - not present in User, but skip safely if seen */
            if (pos + 4 > len) {
                return false;
            }
            pos += 4;
            break;
        default:
            return false;   /* wire types 3/4 (deprecated groups) - not valid here */
        }
    }
    return true;
}
