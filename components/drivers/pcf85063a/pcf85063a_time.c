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

bool pcf85063a_decode_bcd_field(uint8_t raw, uint8_t tens_mask,
                                uint8_t max_value, uint8_t *value)
{
    if (!value) {
        return false;
    }
    uint8_t units = raw & 0x0f;
    uint8_t tens = (raw >> 4) & tens_mask;
    if ((raw & (uint8_t)~((tens_mask << 4) | 0x0f)) != 0 ||
        units > 9 || (uint16_t)tens * 10 + units > max_value) {
        return false;
    }
    *value = (uint8_t)(tens * 10 + units);
    return true;
}

static bool is_leap_year(uint16_t year)
{
    return (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
}

static bool calendar_fields_valid(const pcf85063a_time_t *t)
{
    if (!t || t->year < 1 || t->month < 1 || t->month > 12 ||
        t->hour > 23 || t->min > 59 || t->sec > 59 || t->weekday > 6) {
        return false;
    }
    static const uint8_t month_days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    uint8_t max_day = month_days[t->month - 1];
    if (t->month == 2 && is_leap_year(t->year)) {
        max_day = 29;
    }
    return t->day >= 1 && t->day <= max_day;
}

bool pcf85063a_time_is_valid(const pcf85063a_time_t *t)
{
    return t && t->year >= 2000 && t->year <= 2099 && calendar_fields_valid(t);
}

bool pcf85063a_alarm_fields_are_valid(uint8_t sec, uint8_t min, uint8_t hour,
                                      uint8_t day, uint8_t weekday,
                                      bool mask_day, bool mask_weekday)
{
    return sec <= 59 && min <= 59 && hour <= 23 &&
           (mask_weekday || weekday <= 6) &&
           (mask_day || (day >= 1 && day <= 31));
}

time_t pcf85063a_time_to_epoch(const pcf85063a_time_t *t)
{
    if (!calendar_fields_valid(t)) {
        return (time_t)-1;
    }
    int64_t days = days_from_civil(t->year, t->month, t->day);
    return (time_t)(days * 86400 + t->hour * 3600 + t->min * 60 + t->sec);
}

void pcf85063a_epoch_to_time(time_t epoch, pcf85063a_time_t *out)
{
    if (!out) {
        return;
    }
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
    /* 1970-01-01 (day 0) was a Thursday. Register_03 stores 0..6, matching
     * struct tm's tm_wday, so no offset is needed. */
    int64_t wd = ((days % 7) + 7 + 4) % 7;   /* 0=Sunday .. 6=Saturday */
    out->weekday = (uint8_t)wd;
}
