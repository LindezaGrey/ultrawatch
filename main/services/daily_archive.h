#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "activity_days.h"
#define DAILY_TOTALS_HEADER "date,time,day_steps,lifetime_steps,still_s,walking_s,running_s,cycling_s,vehicle_s,tilting_s,unknown_s\n"
int daily_totals_format(char *out, size_t size, const activity_day_t *day, const char *clock);
/* Rewriting the same date is intentional: retries never add duplicate days. */
bool daily_archive_write(const char *directory, const activity_day_t *day);
