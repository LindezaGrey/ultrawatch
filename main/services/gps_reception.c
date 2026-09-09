#include "gps_reception.h"
#include <string.h>
void gps_reception_start(gps_reception_t *s, uint32_t now)
{
    memset(s, 0, sizeof(*s));
    s->last_fix_ms = s->last_reception_ms = now;
}
gps_reception_reason_t gps_reception_update(gps_reception_t *s, uint32_t now, const m10q_fix_t *f)
{
    bool fresh = f->valid && f->pvt_received && now - f->updated_ms < M10Q_FRESH_MS;
    bool telemetry = f->sats_valid && now - f->sats_updated_ms < M10Q_FRESH_MS;
    if (fresh && now - f->updated_ms < now - s->last_fix_ms) s->last_fix_ms = f->updated_ms;
    if (fresh && now - f->updated_ms < now - s->last_reception_ms) s->last_reception_ms = f->updated_ms;
    if (!telemetry) {
        s->count = s->next = 0;
        s->good = false;
    } else if (!s->seen || s->last_sats_ms != f->sats_updated_ms) {
        s->seen = true;
        s->last_sats_ms = f->sats_updated_ms;
        s->moderate[s->next] = f->sats_above_25;
        s->strong[s->next] = f->sats_above_30;
        s->next = (s->next + 1) % 5;
        if (s->count < 5) s->count++;
        unsigned moderate = 0, strong = 0;
        for (unsigned i = 0; i < s->count; i++) {
            moderate += s->moderate[i]; strong += s->strong[i];
        }
        if (s->count == 5) {
            if (strong >= 20) s->good = true;
            else if (moderate < 20) s->good = false;
        }
        if (s->good && now - f->sats_updated_ms < now - s->last_reception_ms)
            s->last_reception_ms = f->sats_updated_ms;
    }
    if (fresh) return GPS_RECEPTION_OK;
    if (now - s->last_fix_ms >= GPS_ACQUISITION_LIMIT_MS) return GPS_NO_FIX;
    if (now - s->last_reception_ms >= GPS_NO_FIX_TIMEOUT_MS)
        return telemetry ? GPS_POOR_SIGNAL : GPS_NO_TELEMETRY;
    return GPS_RECEPTION_OK;
}
