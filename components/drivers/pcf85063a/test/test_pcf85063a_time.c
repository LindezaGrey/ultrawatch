/*
 * test_pcf85063a_time.c - host tests for pcf85063a_time.c's civil<->epoch
 * conversion (no ESP-IDF dependency, no framework: plain asserts).
 *
 * Build/run: see README.md in this directory.
 */
#include "pcf85063a_time.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int g_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        g_failures++; \
    } \
} while (0)

/* The Unix epoch itself: an independent, well-known anchor (not derived from
 * the code under test). 1970-01-01 00:00:00 UTC was a Thursday. */
static void test_epoch_zero_is_1970_01_01_thursday(void)
{
    pcf85063a_time_t t;
    pcf85063a_epoch_to_time(0, &t);
    CHECK(t.year == 1970);
    CHECK(t.month == 1);
    CHECK(t.day == 1);
    CHECK(t.hour == 0);
    CHECK(t.min == 0);
    CHECK(t.sec == 0);
    CHECK(t.weekday == 5);   /* RTC convention: 1=Sunday .. so Thursday=5 */
}

/* Round-trip a real-world value independently verified via `date -u` during
 * this session's GNSS assist test: 2026-08-28 04:21:56 UTC (a Friday). */
static void test_roundtrip_real_world_value(void)
{
    const time_t epoch = 1787890916;
    pcf85063a_time_t t;
    pcf85063a_epoch_to_time(epoch, &t);
    CHECK(t.year == 2026);
    CHECK(t.month == 8);
    CHECK(t.day == 28);
    CHECK(t.hour == 4);
    CHECK(t.min == 21);
    CHECK(t.sec == 56);
    CHECK(t.weekday == 6);   /* Friday */
    CHECK(pcf85063a_time_to_epoch(&t) == epoch);
}

/* Leap day: Feb 29 2024 exists, and the gap to Mar 1 00:00:00 is exactly
 * 12h (43200s) from noon - both values independently confirmed via `date`. */
static void test_leap_day_2024(void)
{
    pcf85063a_time_t noon = { .sec = 0, .min = 0, .hour = 12, .day = 29,
                               .month = 2, .year = 2024 };
    pcf85063a_time_t next = { .sec = 0, .min = 0, .hour = 0, .day = 1,
                               .month = 3, .year = 2024 };
    CHECK(pcf85063a_time_to_epoch(&noon) == 1709208000);
    CHECK(pcf85063a_time_to_epoch(&next) == 1709251200);
    CHECK(pcf85063a_time_to_epoch(&next) - pcf85063a_time_to_epoch(&noon) == 43200);
}

/* Proleptic Gregorian century rule: 1900 is NOT a leap year (div by 100, not
 * 400), 2000 IS (div by 400) - so their Mar 1 epochs differ by 365 vs 366
 * days from Jan 1. Values independently confirmed via `date -u`. */
static void test_century_leap_rule(void)
{
    pcf85063a_time_t mar1_1900 = { .sec = 0, .min = 0, .hour = 0, .day = 1,
                                    .month = 3, .year = 1900 };
    pcf85063a_time_t mar1_2000 = { .sec = 0, .min = 0, .hour = 0, .day = 1,
                                    .month = 3, .year = 2000 };
    CHECK(pcf85063a_time_to_epoch(&mar1_1900) == -2203891200);
    CHECK(pcf85063a_time_to_epoch(&mar1_2000) == 951868800);
}

/* Year rollover: the second before and the instant of a new year, 1s apart -
 * confirms month/day/year all roll together correctly, not just day. */
static void test_year_rollover(void)
{
    pcf85063a_time_t before = { .sec = 59, .min = 59, .hour = 23, .day = 31,
                                 .month = 12, .year = 2025 };
    pcf85063a_time_t after = { .sec = 0, .min = 0, .hour = 0, .day = 1,
                                .month = 1, .year = 2026 };
    CHECK(pcf85063a_time_to_epoch(&before) == 1767225599);
    CHECK(pcf85063a_time_to_epoch(&after) == 1767225600);
}

/* Pre-1970 (negative epoch): the second before the Unix epoch itself. */
static void test_pre_1970_negative_epoch(void)
{
    pcf85063a_time_t t = { .sec = 59, .min = 59, .hour = 23, .day = 31,
                            .month = 12, .year = 1969 };
    CHECK(pcf85063a_time_to_epoch(&t) == -1);
    pcf85063a_time_t back;
    pcf85063a_epoch_to_time(-1, &back);
    CHECK(back.year == 1969 && back.month == 12 && back.day == 31);
    CHECK(back.hour == 23 && back.min == 59 && back.sec == 59);
}

int main(void)
{
    test_epoch_zero_is_1970_01_01_thursday();
    test_roundtrip_real_world_value();
    test_leap_day_2024();
    test_century_leap_rule();
    test_year_rollover();
    test_pre_1970_negative_epoch();

    if (g_failures) {
        printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
