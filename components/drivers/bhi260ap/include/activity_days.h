#pragma once
#include <stdbool.h>
#include <stdint.h>
#define ACTIVITY_CLASSES 7
#define ACTIVITY_PENDING_DAYS 16
#define ACTIVITY_DAYS_VERSION 1u
typedef struct {
    uint32_t ymd, lifetime, steps;
    uint32_t ms[ACTIVITY_CLASSES];
} activity_day_t;
typedef struct {
    uint32_t version, pending_count;
    activity_day_t current;
    activity_day_t pending[ACTIVITY_PENDING_DAYS];
} activity_days_t;
/* Refuse backward dates and a full outbox; never discard a completed day. */
bool activity_days_advance(activity_days_t *days, uint32_t ymd, int64_t *since_us, int64_t now_us);
void activity_day_steps(activity_day_t *day, uint32_t lifetime);
void activity_days_ack(activity_days_t *days);

void activity_day_credit(activity_day_t *day, unsigned activity, int64_t *since_us, int64_t now_us);
void activity_day_pause(activity_day_t *day, unsigned activity, int64_t *since_us, int64_t now_us);
void activity_day_resume(int64_t *since_us, int64_t now_us);
