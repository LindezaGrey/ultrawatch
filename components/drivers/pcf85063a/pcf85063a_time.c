#include "pcf85063a_time.h"

/* Howard Hinnant's civil-calendar<->days-since-epoch algorithm
 * (http://howardhinnant.github.io/date_algorithms.html): proleptic
 * Gregorian, no libc TZ/DST involvement. */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);                    /* [0, 399] */
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; /* [0, 365] */
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;         /* [0, 146096] */
    return era * 146097 + (int64_t)doe - 719468;
}

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d)
{
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);                        /* [0, 146096] */
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; /* [0, 399] */
    int64_t y0 = (int64_t)yoe + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);   /* [0, 365] */
    unsigned mp = (5 * doy + 2) / 153;                        /* [0, 11] */
    *d = doy - (153 * mp + 2) / 5 + 1;                        /* [1, 31] */
    *m = mp + (mp < 10 ? 3 : -9);                             /* [1, 12] */
    *y = (int)(y0 + (*m <= 2));
}

time_t pcf85063a_time_to_epoch(const pcf85063a_time_t *t)
{
    int64_t days = days_from_civil(t->year, t->month, t->day);
    return (time_t)(days * 86400 + t->hour * 3600 + t->min * 60 + t->sec);
}

void pcf85063a_epoch_to_time(time_t epoch, pcf85063a_time_t *out)
{
    int64_t secs = (int64_t)epoch;
    int64_t days = secs / 86400;
    int64_t rem = secs % 86400;
    if (rem < 0) {
        rem += 86400;
        days -= 1;
    }
    int y;
    unsigned m, d;
    civil_from_days(days, &y, &m, &d);
    out->year = (uint16_t)y;
    out->month = (uint8_t)m;
    out->day = (uint8_t)d;
    out->hour = (uint8_t)(rem / 3600);
    out->min = (uint8_t)((rem % 3600) / 60);
    out->sec = (uint8_t)(rem % 60);
    /* 1970-01-01 (day 0) was a Thursday; RTC convention is 1=Sunday. */
    int64_t wd = ((days % 7) + 7 + 4) % 7;   /* 0=Sunday .. 6=Saturday */
    out->weekday = (uint8_t)(wd + 1);
}
