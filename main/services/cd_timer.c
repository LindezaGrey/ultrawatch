/*
 * services/cd_timer.c - single countdown timer (see cd_timer.h for the API).
 */
#include "cd_timer.h"
#include "alarm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "cd_timer";

/* check() runs from the watch-face UI tick, start/cancel from the UI screen
 * and the dbg console task, so the two scalars need one lock. Everything
 * under it is a scalar access; the ring call stays outside. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_active;
static uint32_t s_remaining_s;

esp_err_t cdtimer_start(uint32_t seconds)
{
    if (seconds == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_mux);
    s_remaining_s = seconds;
    s_active = true;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "started: %lu s", (unsigned long)seconds);
    return ESP_OK;
}

void cdtimer_cancel(void)
{
    portENTER_CRITICAL(&s_mux);
    s_active = false;
    s_remaining_s = 0;
    portEXIT_CRITICAL(&s_mux);
}

bool cdtimer_is_active(void)
{
    portENTER_CRITICAL(&s_mux);
    bool active = s_active;
    portEXIT_CRITICAL(&s_mux);
    return active;
}

uint32_t cdtimer_remaining_seconds(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t remaining = s_active ? s_remaining_s : 0;
    portEXIT_CRITICAL(&s_mux);
    return remaining;
}

void cdtimer_check(void)
{
    bool expired = false;
    portENTER_CRITICAL(&s_mux);
    if (s_active) {
        if (s_remaining_s > 0) {
            s_remaining_s--;
        }
        if (s_remaining_s == 0) {
            /* Expired: mark inactive *before* ringing, since alarm.c's ring task
             * treats a timer-sourced dismiss as a pure no-op (nothing left to
             * re-arm) - this is the only state that needs clearing. */
            s_active = false;
            expired = true;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (expired) {
        ESP_LOGI(TAG, "expired, ringing");
        alarm_ring_now(ALARM_RING_SOURCE_TIMER, ALARM_RING_BOTH);
    }
}
