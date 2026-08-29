/*
 * cd_timer.c - single countdown timer (see cd_timer.h for the API).
 */
#include "cd_timer.h"
#include "alarm.h"
#include "esp_log.h"

static const char *TAG = "cd_timer";

static bool s_active;
static uint32_t s_remaining_s;

esp_err_t cdtimer_start(uint32_t seconds)
{
    if (seconds == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_remaining_s = seconds;
    s_active = true;
    ESP_LOGI(TAG, "started: %lu s", (unsigned long)seconds);
    return ESP_OK;
}

void cdtimer_cancel(void)
{
    s_active = false;
    s_remaining_s = 0;
}

bool cdtimer_is_active(void)
{
    return s_active;
}

uint32_t cdtimer_remaining_seconds(void)
{
    return s_active ? s_remaining_s : 0;
}

void cdtimer_check(void)
{
    if (!s_active) {
        return;
    }
    if (s_remaining_s > 0) {
        s_remaining_s--;
    }
    if (s_remaining_s == 0) {
        /* Expired: mark inactive *before* ringing, since alarm.c's ring task
         * treats a timer-sourced dismiss as a pure no-op (nothing left to
         * re-arm) - this is the only state that needs clearing. */
        s_active = false;
        ESP_LOGI(TAG, "expired, ringing");
        alarm_ring_now(ALARM_RING_SOURCE_TIMER, ALARM_RING_BOTH);
    }
}
