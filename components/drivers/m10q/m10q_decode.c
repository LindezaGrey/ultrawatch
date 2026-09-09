#include "m10q_decode.h"
#include <math.h>
#include <string.h>

static uint16_t u16(const uint8_t *p) { return (uint16_t)p[0] | (uint16_t)p[1] << 8; }
static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static int32_t i32(const uint8_t *p)
{
    uint32_t value = u32(p);
    return value <= INT32_MAX ? (int32_t)value : -1 - (int32_t)(UINT32_MAX - value);
}

bool m10q_decode_sat(m10q_fix_t *fix, const uint8_t *p, size_t length, uint32_t now)
{
    if (length < 8 || p[4] != 1 || length != 8u + 12u * p[5]) return false;
    uint32_t epoch = u32(p);
    if (fix->sats_valid && fix->sats_itow == epoch) return false;
    fix->sats_itow = epoch;
    fix->sats_updated_ms = now;
    fix->sats_valid = true;
    fix->sats_total = p[5];
    fix->sats_above_25 = fix->sats_above_30 = 0;
    fix->best_cno = 0;
    fix->sat_in_view = 0;
    memset(fix->sats, 0, sizeof(fix->sats));
    for (unsigned i = 0; i < p[5]; i++) {
        const uint8_t *sv = p + 8 + i * 12;
        uint32_t flags = u32(sv + 8);
        m10q_sat_t sat = {
            .gnss_id = sv[0], .prn = sv[1], .snr_db = sv[2] ? sv[2] : -1,
            .elevation_deg = sv[3] < 128 ? sv[3] : (int)sv[3] - 256,
            .azimuth_deg = (int16_t)u16(sv + 4),
            .quality = flags & 7, .used = (flags & (1u << 3)) != 0,
            .healthy = ((flags >> 4) & 3) != 2,
        };
        if (sat.healthy && sat.quality >= 4) {
            if (sv[2] >= 25) fix->sats_above_25++;
            if (sv[2] >= 30) fix->sats_above_30++;
        }
        if (sat.healthy && sv[2] > fix->best_cno) fix->best_cno = sv[2];
        if (!sv[2]) continue;
        /* Display capacity must not bias reception statistics. Keep top C/N0. */
        unsigned pos = 0;
        while (pos < fix->sat_in_view && fix->sats[pos].snr_db >= sat.snr_db) pos++;
        if (pos >= M10Q_MAX_SATS) continue;
        unsigned count = fix->sat_in_view;
        if (count < M10Q_MAX_SATS) count++;
        for (unsigned j = count - 1; j > pos; j--) fix->sats[j] = fix->sats[j - 1];
        fix->sats[pos] = sat;
        fix->sat_in_view = count;
    }
    return true;
}

bool m10q_decode_pvt(m10q_fix_t *fix, const uint8_t *p, size_t length, uint32_t now)
{
    if (length != 92) return false;
    uint32_t epoch = u32(p);
    if (fix->pvt_received && fix->pvt_itow == epoch) return false;
    fix->pvt_received = true;
    fix->pvt_itow = epoch;
    fix->updated_ms = now;
    int32_t lon = i32(p + 24), lat = i32(p + 28);
    fix->valid = (p[21] & 1) && (p[20] == 2 || p[20] == 3) &&
                 lat >= -900000000 && lat <= 900000000 &&
                 lon >= -1800000000 && lon <= 1800000000;
    fix->fix_3d = fix->valid && p[20] == 3;
    fix->lat = lat / 1e7;
    fix->lon = lon / 1e7;
    fix->alt_m = fix->fix_3d ? i32(p + 36) / 1000.0 : NAN;
    fix->sat_count = p[23];
    uint32_t accuracy = u32(p + 40) / 1000 + (u32(p + 40) % 1000 >= 500);
    fix->hacc_m = accuracy < UINT16_MAX ? accuracy : UINT16_MAX;
    int32_t speed = i32(p + 60), heading = i32(p + 64);
    fix->speed_kmh = speed > 0 ? (uint16_t)fmin(speed * 0.0036 + 0.5, UINT16_MAX) : 0;
    fix->heading_valid = fix->valid && speed >= 500 && heading >= 0 && heading <= 36000000 &&
                         u32(p + 72) < 18000000;
    fix->course_deg = fix->heading_valid ? ((heading + 50000) / 100000) % 360 : UINT16_MAX;
    fix->year = u16(p + 4); fix->month = p[6]; fix->day = p[7];
    fix->hour = p[8]; fix->minute = p[9]; fix->second = p[10];
    fix->time_valid = (p[11] & 3) == 3 && fix->year >= 1980 && fix->year <= 2099 &&
                     fix->month >= 1 && fix->month <= 12 && fix->day >= 1 && fix->day <= 31 &&
                     fix->hour < 24 && fix->minute < 60 && fix->second <= 60;
    return true;
}

bool m10q_decode_dop(m10q_fix_t *fix, const uint8_t *p, size_t length, uint32_t now)
{
    if (length != 18) return false;
    if (fix->dop_valid && fix->dop_itow == u32(p)) return false;
    fix->dop_itow = u32(p);
    fix->hdop = (u16(p + 12) + 5) / 10; /* UBX is 0.01, public API is 0.1. */
    fix->dop_updated_ms = now;
    fix->dop_valid = true;
    return true;
}

void m10q_expire_snapshot(m10q_fix_t *fix, uint32_t now)
{
    if (!fix->pvt_received || now - fix->updated_ms >= M10Q_FRESH_MS) {
        fix->valid = fix->fix_3d = fix->heading_valid = false;
        fix->alt_m = NAN;
    }
    if (!fix->sats_valid || now - fix->sats_updated_ms >= M10Q_FRESH_MS) {
        fix->sats_valid = false;
        fix->sat_in_view = fix->sats_total = fix->sats_above_25 = fix->sats_above_30 = 0;
        fix->best_cno = 0;
        memset(fix->sats, 0, sizeof(fix->sats));
    }
    if (!fix->dop_valid || now - fix->dop_updated_ms >= M10Q_FRESH_MS) {
        fix->hdop = UINT16_MAX;
        fix->dop_valid = false;
    }
}
