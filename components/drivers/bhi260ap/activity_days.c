#include "activity_days.h"
#include <string.h>
void activity_day_steps(activity_day_t *day, uint32_t lifetime)
{
    if (lifetime > day->lifetime) day->steps += lifetime - day->lifetime;
    day->lifetime = lifetime;
}
bool activity_days_advance(activity_days_t *d, uint32_t ymd, int64_t *since_us, int64_t now_us)
{
    if (!ymd || ymd < d->current.ymd) return false;
    if (ymd == d->current.ymd) return true;
    if (d->current.ymd) {
        if (d->pending_count == ACTIVITY_PENDING_DAYS) return false;
        d->pending[d->pending_count++] = d->current;
    }
    uint32_t lifetime = d->current.lifetime;
    memset(&d->current, 0, sizeof(d->current));
    d->current.ymd = ymd;
    d->current.lifetime = lifetime;
    *since_us = now_us;
    return true;
}
void activity_days_ack(activity_days_t *d)
{
    if (!d->pending_count) return;
    d->pending_count--;
    memmove(d->pending, d->pending + 1, d->pending_count * sizeof(d->pending[0]));
    memset(&d->pending[d->pending_count], 0, sizeof(d->pending[0]));
}

void activity_day_credit(activity_day_t *day, unsigned activity, int64_t *since_us, int64_t now_us)
{
    if (activity < ACTIVITY_CLASSES && now_us > *since_us) {
        uint64_t elapsed_ms = (uint64_t)(now_us - *since_us) / 1000;
        uint64_t total = day->ms[activity] + elapsed_ms;
        day->ms[activity] = total > UINT32_MAX ? UINT32_MAX : (uint32_t)total;
        *since_us += elapsed_ms * 1000;
    } else if (now_us < *since_us) {
        *since_us = now_us;
    }
}
