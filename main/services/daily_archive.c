#include "daily_archive.h"
#include <stdio.h>
#include <unistd.h>
int daily_totals_format(char *out, size_t size, const activity_day_t *d, const char *clock)
{
    return snprintf(out, size,
        "%04lu-%02lu-%02lu,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)(d->ymd / 10000), (unsigned long)(d->ymd / 100 % 100),
        (unsigned long)(d->ymd % 100), clock, (unsigned long)d->steps,
        (unsigned long)d->lifetime, (unsigned long)(d->ms[0] / 1000),
        (unsigned long)(d->ms[1] / 1000), (unsigned long)(d->ms[2] / 1000),
        (unsigned long)(d->ms[3] / 1000), (unsigned long)(d->ms[4] / 1000),
        (unsigned long)(d->ms[5] / 1000), (unsigned long)(d->ms[6] / 1000));
}
bool daily_archive_write(const char *directory, const activity_day_t *day)
{
    char path[160], row[256];
    int n = snprintf(path, sizeof(path), "%s/%08lu.csv", directory, (unsigned long)day->ymd);
    if (n < 0 || (size_t)n >= sizeof(path)) return false;
    n = daily_totals_format(row, sizeof(row), day, "");
    if (n < 0 || (size_t)n >= sizeof(row)) return false;
    FILE *file = fopen(path, "w");
    if (!file) return false;
    bool ok = fputs(DAILY_TOTALS_HEADER, file) >= 0 && fputs(row, file) >= 0;
    if (fflush(file) != 0) ok = false;
    if (fsync(fileno(file)) != 0) ok = false;
    if (fclose(file) != 0) ok = false;
    return ok;
}
