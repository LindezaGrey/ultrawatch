/*
 * debug/debug_bhi.c - BHI260AP sensor-hub debug console commands.
 *
 * Moved out of uwatch_main.c's debug_process_cmd() dispatch (mechanical
 * refactor, no behavior change).
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "bhi260ap.h"
#include "bhy2_defs.h"
#include "lvgl_app.h"
#include "debug_bhi.h"

/* Tally what the WAKE-UP FIFO carried during a trace window, tagging known
 * sensor/event ids. Only used by wudump/wusus/wudis below. */
static void dump_wu_counts(void)
{
    const char *tag;
    for (int i = 1; i < 256; i++) {
        uint32_t c = bhi260ap_wu_trace_get_count((uint8_t)i);
        if (c) {
            tag = NULL;
            switch (i) {
            case BHY2_SENSOR_ID_ACC: tag = "acc"; break;
            case BHY2_SENSOR_ID_ACC_WU: tag = "acc_wu"; break;
            case BHY2_SENSOR_ID_STC: tag = "stc"; break;
            case BHY2_SENSOR_ID_STC_WU: tag = "stc_wu"; break;
            case BHY2_SENSOR_ID_GYRO: tag = "gyro"; break;
            case BHY2_SENSOR_ID_GYRO_WU: tag = "gyro_wu"; break;
            case BHY2_SENSOR_ID_GAMERV: tag = "gamerv"; break;
            case BHY2_SENSOR_ID_GAMERV_WU: tag = "gamerv_wu"; break;
            case BHY2_SENSOR_ID_WRIST_TILT_GESTURE: tag = "wrist_tilt"; break;
            case BHY2_SENSOR_ID_WAKE_GESTURE: tag = "wake_gest"; break;
            case BHY2_SENSOR_ID_GLANCE_GESTURE: tag = "glance"; break;
            case BHY2_SENSOR_ID_PICKUP_GESTURE: tag = "pickup"; break;
            case BHY2_SYS_ID_META_EVENT: tag = "meta"; break;
            case BHY2_SYS_ID_TS_SMALL_DELTA_WU: tag = "ts_small_wu"; break;
            case BHY2_SYS_ID_TS_LARGE_DELTA_WU: tag = "ts_large_wu"; break;
            case BHY2_SYS_ID_TS_FULL_WU: tag = "ts_full_wu"; break;
            case BHY2_SYS_ID_FILLER: tag = "filler"; break;
            default: break;
            }
            printf("  %3d %-12s %lu\n", i, tag ? tag : "?", (unsigned long)c);
        }
    }
}

void debug_bhi_bhi(const char *args)
{
    (void)args;
    /* Dump all BHI260AP sensor values. */
    bool ready = false;
    uint32_t steps = 0;
    bhi260ap_get_status(&ready, &steps);
    printf("bhi: ready=%d steps=%lu\n", (int)ready, (unsigned long)steps);
    int16_t v0, v1, v2;
    if (bhi260ap_get_accel(&v0, &v1, &v2) == ESP_OK) {
        printf("bhi: accel mg: %d %d %d\n", v0, v1, v2);
    }
    if (bhi260ap_get_gyro(&v0, &v1, &v2) == ESP_OK) {
        printf("bhi: gyro dps: %d %d %d\n", v0, v1, v2);
    }
    if (bhi260ap_get_orientation(&v0, &v1, &v2) == ESP_OK) {
        printf("bhi: ori d: %d %d %d\n", v0, v1, v2);
    }
    int16_t qx, qy, qz, qw;
    uint16_t qa;
    if (bhi260ap_get_rotation(&qx, &qy, &qz, &qw, &qa) == ESP_OK) {
        printf("bhi: rv: %d %d %d %d acc=%u\n", qx, qy, qz, qw, (unsigned)qa);
    }
    uint8_t act = BHI260AP_ACTIVITY_UNKNOWN;
    if (bhi260ap_get_activity(&act) == ESP_OK) {
        printf("bhi: activity: %u\n", (unsigned)act);
    }
}

void debug_bhi_suspend(const char *args)
{
    (void)args;
    bhi260ap_ap_suspend();
    printf("suspend: done\n");
}

void debug_bhi_resume(const char *args)
{
    (void)args;
    bhi260ap_ap_resume();
    printf("resume: done\n");
}

void debug_bhi_imon(const char *args)
{
    (void)args;
    /* Watch the BHI INT cmd (GPIO8) for ~30 s, print each change. */
    int last = gpio_get_level(GPIO_NUM_8);
    printf("imon: gpio8 initial=%d\n", last);
    uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start < 30000) {
        int cur = gpio_get_level(GPIO_NUM_8);
        if (cur != last) {
            printf("imon: gpio8 -> %d @ %lu\n", cur,
                   (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));
            last = cur;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("imon: done\n");
}

void debug_bhi_wudump(const char *args)
{
    (void)args;
    /* Tally what the WAKE-UP FIFO actually carries over 5 s, to spot
     * the source of spurious wake interrupts (GPIO8). */
    bhi260ap_wu_trace_start();
    vTaskDelay(pdMS_TO_TICKS(5000));
    bhi260ap_wu_trace_stop();
    printf("wu-fifo ids (5s):\n");
    dump_wu_counts();
    printf("wudump: done\n");
}

void debug_bhi_wusus(const char *args)
{
    (void)args;
    /* Suspend with the AP still awake, so the trace can capture exactly
     * what the chip pushes into the WAKE-UP FIFO when ap_suspend() runs
     * (sensor disables, FIFO flush, meta events). */
    printf("wusus: drain-before -> %d\n", bhi260ap_drain_wakeup_fifo());
    bhi260ap_ap_suspend();
    bhi260ap_wu_trace_start();
    vTaskDelay(pdMS_TO_TICKS(5000));
    bhi260ap_wu_trace_stop();
    printf("wu-fifo ids during 5s suspend:\n");
    dump_wu_counts();
    printf("wusus: gpio8=%d\n", gpio_get_level(GPIO_NUM_8));
    bhi260ap_ap_resume();
    printf("wusus: done\n");
}

void debug_bhi_wudis(const char *args)
{
    /* Bisect: disable one sensor group, then expose the WAKE-UP FIFO
     * traffic + GPIO8 pulsing for 10 s so we can see which enabled sensor
     * (or sensor combination) generates the spurious WU-FIFO events.
     *  1 = wrist-tilt + wake gesture  2 = glance+pickup+tilt
     *  4 = activity recognition       8 = step counter
     *  16 = acc+gyro+gamerv           32 = everything non-gesture */
    unsigned mask = strtoul(args, NULL, 0);
    printf("wudis: disabling group mask 0x%x\n", mask);
    if (mask & 1) {
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_WRIST_TILT_GESTURE, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_WAKE_GESTURE, 0.0f);
    }
    if (mask & 2) {
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_GLANCE_GESTURE, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_PICKUP_GESTURE, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_TILT_DETECTOR, 0.0f);
    }
    if (mask & 4) {
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_AR, 0.0f);
    }
    if (mask & 8) {
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_STC, 0.0f);
    }
    if (mask & 16) {
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_ACC, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_GYRO, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_GAMERV, 0.0f);
    }
    if (mask & 32) {
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_ACC, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_GYRO, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_GAMERV, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_AR, 0.0f);
        bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_STC, 0.0f);
    }
    bhi260ap_wu_trace_start();
    vTaskDelay(pdMS_TO_TICKS(10000));
    bhi260ap_wu_trace_stop();
    printf("wudis: gpio8 level=%d\n", gpio_get_level(GPIO_NUM_8));
    dump_wu_counts();
    printf("wudis: done\n");
}

void debug_bhi_metahist(const char *args)
{
    (void)args;
    bhi260ap_meta_hist_start();
    bhi260ap_meta_print(80);
    vTaskDelay(pdMS_TO_TICKS(5000));
    bhi260ap_meta_hist_stop();
    bhi260ap_meta_print(0);
    uint8_t n = bhi260ap_meta_hist_len_get();
    printf("meta hist (%u keys):\n", n);
    for (uint8_t i = 0; i < n; i++) {
        uint8_t k, c;
        bhi260ap_meta_hist_get(i, &k, &c);
        printf("  key 0x%02x %u\n", k, (unsigned)c);
    }
    printf("metahist: done\n");
}

void debug_bhi_sensorlist(const char *args)
{
    (void)args;
    bhi260ap_dump_sensor_list();
    printf("sensorlist: done\n");
}

void debug_bhi_bhishow(const char *args)
{
    (void)args;
    lvgl_show_bhi_screen();
    printf("bhishow: done\n");
}

void debug_bhi_grate(const char *args)
{
    float hz = strtof(args, NULL);
    esp_err_t e = bhi260ap_set_sensor_rate(BHY2_SENSOR_ID_GAMERV, hz);
    printf("grate: gamerv @ %.1f Hz -> %s\n", (double)hz,
           (e == ESP_OK) ? "accepted" : "rejected");
}
