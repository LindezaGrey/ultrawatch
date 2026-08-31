/*
 * housekeeping.c - see housekeeping.h.
 */
#include "housekeeping.h"
#include "syslog_capture.h"
#include "daily_log.h"
#include "gpx_log.h"
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

static void housekeeping_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));   /* let boot settle, matches daily_log's original delay */

    TickType_t last_gpx = xTaskGetTickCount();
    TickType_t last_daily = last_gpx;

    for (;;) {
        /* Blocks up to 10s, or returns early on syslog's own buffer-full
         * wake - so the gpx/daily cadence below is tracked by elapsed
         * ticks, never by counting loop iterations. */
        syslog_capture_service(10000);

        TickType_t now = xTaskGetTickCount();
        if ((now - last_gpx) >= pdMS_TO_TICKS(HOUSEKEEPING_GPX_INTERVAL_MS)) {
            gpx_log_tick();
            last_gpx = now;
        }
        if ((now - last_daily) >= pdMS_TO_TICKS(HOUSEKEEPING_DAILY_INTERVAL_MS)) {
            daily_log_tick();
            last_daily = now;
        }
    }
}

void housekeeping_init(void)
{
    xTaskCreate(housekeeping_task, "housekeeping", HOUSEKEEPING_TASK_STACK, NULL, 2, NULL);
}
