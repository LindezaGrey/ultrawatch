/*
 * services/housekeeping.c - see housekeeping.h.
 */
#include "housekeeping.h"
#include "syslog_capture.h"
#include "daily_log.h"
#include "gpx_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HOUSEKEEPING_GPX_INTERVAL_MS    30000
#define HOUSEKEEPING_DAILY_INTERVAL_MS  60000
/* Started generous on purpose: this task's real worst-case call depth
 * (a day-roll write plus a GPX trackpoint append plus a syslog flush, all
 * landing on the same tick) hasn't been measured live yet. Only shrink
 * after checking the "stacks" debug command under those conditions
 * actually happening together, with real margin left over - see
 * docs/application.md section 12 for why an idle-only snapshot isn't
 * trustworthy here. */
#define HOUSEKEEPING_TASK_STACK  4096
static const char *TAG = "housekeeping";
static TaskHandle_t s_task;

static void housekeeping_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));   /* let boot settle, matches daily_log's original delay */

    TickType_t last_gpx = xTaskGetTickCount();
    TickType_t last_daily = last_gpx;
    bool sampled = false;

    for (;;) {
        /* Flush syslog on the same one-minute cadence as daily telemetry,
         * or earlier on syslog's own buffer-high-water wake.  The gpx/daily
         * cadence below is tracked by elapsed ticks, never by counting loop
         * iterations, so early syslog wakes cannot postpone either logger. */
        syslog_capture_service(1000);
        daily_log_reconcile();

        TickType_t now = xTaskGetTickCount();
        if ((now - last_gpx) >= pdMS_TO_TICKS(HOUSEKEEPING_GPX_INTERVAL_MS)) {
            gpx_log_tick();
            last_gpx = now;
        }
        uint32_t steps;
        if ((!sampled && daily_log_get_steps(&steps) == ESP_OK) ||
            (now - last_daily) >= pdMS_TO_TICKS(HOUSEKEEPING_DAILY_INTERVAL_MS)) {
            daily_log_tick();
            last_daily = now;
            sampled = true;
        }
    }
}

void housekeeping_init(void)
{
    if (s_task) {
        return;
    }
    daily_log_init();
    if (xTaskCreate(housekeeping_task, "housekeeping", HOUSEKEEPING_TASK_STACK, NULL, 2, &s_task) != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "task allocation failed");
    }
}
