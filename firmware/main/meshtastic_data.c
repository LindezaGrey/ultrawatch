#include "meshtastic_data.h"
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

bool meshtastic_data_decode(const uint8_t *buf, size_t len, meshtastic_data_t *out)
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
        case 0: {   /* varint */
            uint64_t value;
            if (!read_varint(buf, len, &pos, &value)) {
                return false;
            }
            if (field_num == 1) {   /* portnum */
                out->portnum = (uint32_t)value;
            }
            break;
        }
        case 1:   /* 64-bit fixed (fixed64/sfixed64/double) */
            if (pos + 8 > len) {
                return false;
            }
            pos += 8;
            break;
        case 2: {   /* length-delimited (bytes/string/embedded message) */
            uint64_t field_len;
            if (!read_varint(buf, len, &pos, &field_len)) {
                return false;
            }
            if (field_len > len - pos) {
                return false;
            }
            if (field_num == 2) {   /* payload */
                out->payload = buf + pos;
                out->payload_len = (size_t)field_len;
            }
            pos += (size_t)field_len;
            break;
        }
        case 5:   /* 32-bit fixed (fixed32/sfixed32/float) */
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

/* meshtastic/protobufs' portnums.proto - just the values worth labeling in
 * a debug dump or log line; anything else prints as a bare number. */
const char *meshtastic_portnum_name(uint32_t portnum)
{
    switch (portnum) {
    case 0:  return "UNKNOWN_APP";
    case 1:  return "TEXT_MESSAGE_APP";
    case 2:  return "REMOTE_HARDWARE_APP";
    case 3:  return "POSITION_APP";
    case 4:  return "NODEINFO_APP";
    case 5:  return "ROUTING_APP";
    case 6:  return "ADMIN_APP";
    case 7:  return "TEXT_MESSAGE_COMPRESSED_APP";
    case 8:  return "WAYPOINT_APP";
    case 10: return "DETECTION_SENSOR_APP";
    case 67: return "TELEMETRY_APP";
    case 70: return "TRACEROUTE_APP";
    case 71: return "NEIGHBORINFO_APP";
    case 73: return "MAP_REPORT_APP";
    default: return NULL;
    }
}
