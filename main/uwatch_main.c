/*
 * uwatch_main.c - UWatch application entry point.
 *
 * FreeRTOS (IDF) app_main: initializes the T-Watch Ultra board package,
 * then hands the display over to LVGL (esp_lvgl_adapter), which runs its
 * own FreeRTOS task. Debug via JTAG/OpenOCD/GDB.
 *
 * A tiny USB-Serial-JTAG command loop handles debug commands such as "shot"
 * (dump the current screen as base64 RGB565). The USB-JTAG port is the same
 * /dev/ttyACM* used for flashing.
 */
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "twatch_board.h"
#include "lvgl_app.h"
#include "sd_log.h"
#include "bhi260ap.h"
#include "bhy2_defs.h"
#include "co5300.h"
#include "m10q.h"
#include "drv2605.h"
#include "xl9555.h"
#include "crash_dump.h"
#include "tracking.h"
#include "sensor_cache.h"
#include "daily_log.h"
#include "max98357a.h"
#include "t3902.h"
#include "axp2101.h"
#include "uwatch_main.h"
#include "ble_debug.h"
#include "alarm.h"
#include "power_mgmt.h"
#include <stdio.h>
#include <dirent.h>

static const char *TAG = "uwatch";

#define DBG_RX_BUF   256
#define DBG_TASK_STACK 4096

/* Audio test: buffer holding the last recording (PSRAM), shared by rec/playrec. */
static int16_t *s_rec_buf;
static size_t s_rec_n;
static SemaphoreHandle_t s_rec_done;   /* signalled when a background rec finishes */

static void debug_process_cmd(const char *cmd);

/* Ensure s_rec_buf can hold n samples; returns true on success. On alloc/realloc
 * failure the previous buffer is left intact (it may still be NULL), so the
 * caller can retry later without having s_rec_n outpace the actual buffer size.
 * When only shrinking, the existing larger buffer is reused as-is. */
static bool rec_ensure_buf(size_t n)
{
    if (!s_rec_buf) {
        s_rec_buf = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    } else if (n > s_rec_n) {
        int16_t *nb = heap_caps_realloc(s_rec_buf, n * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!nb) {
            return false;   /* old (smaller) buffer kept, s_rec_n unchanged */
        }
        s_rec_buf = nb;
    }
    return s_rec_buf != NULL;
}

/* Background recorder: fills s_rec_buf while the speaker plays (the amp and
 * mic are on separate I2S controllers, so TX + RX can run concurrently). */
static void rec_task(void *arg)
{
    (void)arg;
    size_t n = s_rec_n;
    t3902_read(s_rec_buf, n);
    if (s_rec_done) {
        xSemaphoreGive(s_rec_done);
    }
    vTaskDelete(NULL);
}

static void debug_task(void *arg)
{
    (void)arg;
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "USB-JTAG driver install failed: %s; debug console disabled", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    static char line[DBG_RX_BUF];
    size_t len = 0;
    uint32_t last_heap_log = 0;

    for (;;) {
        int n = usb_serial_jtag_read_bytes(line + len, sizeof(line) - len - 1, pdMS_TO_TICKS(50));
        if (n < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (n == 0) {
            /* Periodic heap watchdog to track the DMA-heap leak. */
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now - last_heap_log >= 30000) {
                last_heap_log = now;
                printf("heap: free=%lu min=%lu dma=%lu\n",
                       (unsigned long)esp_get_free_heap_size(),
                       (unsigned long)esp_get_minimum_free_heap_size(),
                       (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
            }
            continue;
        }
        len += (size_t)n;
        /* Process complete lines. */
        char *nl;
        while ((nl = memchr(line, '\n', len)) != NULL) {
            size_t cmd_len = (size_t)(nl - line);
            /* Strip trailing CR. */
            while (cmd_len > 0 && (line[cmd_len - 1] == '\r' || line[cmd_len - 1] == ' ')) {
                cmd_len--;
            }
            line[cmd_len] = '\0';
            debug_process_cmd(line);
            /* Shift remaining bytes. */
            size_t rest = len - (size_t)(nl - line) - 1;
            memmove(line, nl + 1, rest);
            len = rest;
        }
    }
}


/* Execute one console command. Shared by the USB-Serial-JTAG console and
 * the BLE debug bridge so both can drive the watch. */
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

static void debug_process_cmd(const char *cmd)
{
    if (strcmp(cmd, "shot") == 0) {
        lvgl_app_dump_screenshot();
    } else if (strcmp(cmd, "sdin") == 0) {
        /* Read back /sdcard/log/uwatch.log and dump it. */
        FILE *f = fopen("/sdcard/log/uwatch.log", "r");
        if (!f) {
            printf("sdin: no log file\n");
        } else {
            char c;
            while (fread(&c, 1, 1, f) == 1) {
                putchar(c);
            }
            fclose(f);
        }
    } else if (strcmp(cmd, "sdls") == 0) {
        /* List PNG screenshots on the SD card. */
        DIR *d = opendir("/sdcard/shot");
        if (!d) {
            printf("sdls: no shot dir\n");
        } else {
            struct dirent *e;
            while ((e = readdir(d)) != NULL) {
                printf("%s\n", e->d_name);
            }
            closedir(d);
        }
    } else if (strcmp(cmd, "sdclear") == 0) {
        /* Truncate the log and delete screenshots. */
        esp_err_t err = sd_log_clear();
        printf("sdclear: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
    } else if (strcmp(cmd, "heap") == 0) {
        printf("heap: free=%lu min=%lu dma=%lu\n",
               (unsigned long)esp_get_free_heap_size(),
               (unsigned long)esp_get_minimum_free_heap_size(),
               (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    } else if (strcmp(cmd, "bhi") == 0) {
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
    } else if (strcmp(cmd, "suspend") == 0) {
        bhi260ap_ap_suspend();
        printf("suspend: done\n");
    } else if (strcmp(cmd, "resume") == 0) {
        bhi260ap_ap_resume();
        printf("resume: done\n");
    } else if (strcmp(cmd, "imon") == 0) {
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
    } else if (strcmp(cmd, "wudump") == 0) {
        /* Tally what the WAKE-UP FIFO actually carries over 5 s, to spot
         * the source of spurious wake interrupts (GPIO8). */
        bhi260ap_wu_trace_start();
        vTaskDelay(pdMS_TO_TICKS(5000));
        bhi260ap_wu_trace_stop();
        printf("wu-fifo ids (5s):\n");
        dump_wu_counts();
        printf("wudump: done\n");
    } else if (strcmp(cmd, "wusus") == 0) {
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
    } else if (strncmp(cmd, "wudis ", 6) == 0) {
        /* Bisect: disable one sensor group, then expose the WAKE-UP FIFO
         * traffic + GPIO8 pulsing for 10 s so we can see which enabled sensor
         * (or sensor combination) generates the spurious WU-FIFO events.
         *  1 = wrist-tilt + wake gesture  2 = glance+pickup+tilt
         *  4 = activity recognition       8 = step counter
         *  16 = acc+gyro+gamerv           32 = everything non-gesture */
        unsigned mask = strtoul(cmd + 6, NULL, 0);
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
    } else if (strcmp(cmd, "metahist") == 0) {
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
    } else if (strcmp(cmd, "gnsson") == 0) {
        lvgl_gps_set_enabled(true);
        printf("gnss: powered on\n");
    } else if (strcmp(cmd, "gnssoff") == 0) {
        lvgl_gps_set_enabled(false);
        printf("gnss: powered off\n");
    } else if (strcmp(cmd, "gnssver") == 0) {
        /* Dump UBX-MON-VER to identify the module (genuine u-blox vs clone). */
        m10q_power(true);
        vTaskDelay(pdMS_TO_TICKS(500));
        uGnssVersionType_t ver;
        memset(&ver, 0, sizeof(ver));
        if (m10q_get_versions(&ver) == ESP_OK) {
            printf("gnssver: sw=%s\n", ver.ver);
            printf("gnssver: hw=%s\n", ver.hw);
            printf("gnssver: mod=%s\n", ver.mod);
            printf("gnssver: fw=%s\n", ver.fw);
            printf("gnssver: prot=%s\n", ver.prot);
        } else {
            printf("gnssver: failed to read MON-VER\n");
        }
        m10q_power(false);
    } else if (strcmp(cmd, "rtccal") == 0) {
        /* Re-run the PPS-based RTC drift calibration (needs a GNSS fix). */
        m10q_rtc_calibrate();
        printf("rtccal: triggered (PPS window ~120 s, needs fix)\n");
    } else if (strcmp(cmd, "gnss") == 0) {
        /* Dump GNSS state, fix, and satellites. */
        m10q_fix_t fix;
        m10q_get_fix(&fix);
        uint32_t rx = 0, lines = 0;
        m10q_get_dbg(&rx, &lines);
        printf("gnss: state=%d valid=%d rx=%lu lines=%lu gsv=%lu\n",
               (int)m10q_get_state(), (int)fix.valid,
               (unsigned long)rx, (unsigned long)lines,
               (unsigned long)m10q_get_gsv_count());
        if (fix.valid) {
            printf("gnss: pos %.5f %.5f alt %.0fm\n", fix.lat, fix.lon, fix.alt_m);
            printf("gnss: speed %u km/h course %u sats %u hdop %.1f hAcc %um\n",
                   (unsigned)fix.speed_kmh, (unsigned)fix.course_deg,
                   (unsigned)fix.sat_count, fix.hdop / 10.0,
                   (unsigned)fix.hacc_m);
        }
        printf("gnss: in view %u\n", (unsigned)fix.sat_in_view);
        m10q_nav_status_t ns;
        if (m10q_get_nav_status(&ns) == ESP_OK && ns.updated) {
            printf("gnss: nav fix=%u fixOk=%d wknsSet=%d towSet=%d ttff=%lu ms\n",
                   (unsigned)ns.gps_fix, (int)ns.gps_fix_ok, (int)ns.wkns_set,
                   (int)ns.tow_set, (unsigned long)ns.ttff_ms);
        }
        for (int i = 0; i < (int)fix.sat_in_view && i < M10Q_MAX_SATS; i++) {
            printf("gnss: sat %2u el %3d az %3d snr %d used %d\n",
                   (unsigned)fix.sats[i].prn, (int)fix.sats[i].elevation_deg,
                   (int)fix.sats[i].azimuth_deg, (int)fix.sats[i].snr_db,
                   (int)fix.sats[i].used);
        }
        m10q_stats_t st;
        if (m10q_get_stats(&st) == ESP_OK) {
            printf("gnss: fixes=%lu today=%lu ttf_avg=%lu ms best=%lu ms\n",
                   (unsigned long)st.total_fixes, (unsigned long)st.fixes_today,
                   (unsigned long)st.ttf_avg_ms, (unsigned long)st.ttf_best_ms);
        }
        uint16_t agc = 0;
        m10q_poll_agc();
        vTaskDelay(pdMS_TO_TICKS(200));
        if (m10q_get_agc(&agc) == ESP_OK) {
            printf("gnss: agc=%u\n", (unsigned)agc);
        }
    } else if (strcmp(cmd, "gpscheck") == 0) {
        /* Re-run the boot-time GNSS position check (background). */
        printf("gpscheck: queued (GNSS powers on for a 3D fix, then off)\n");
        lvgl_gps_refresh();
    } else if (strncmp(cmd, "gnssseed ", 9) == 0) {
        /* Seed the receiver with an approximate position to speed acquisition.
         * Usage: "gnssseed <lat> <lon>". */
        double lat = atof(cmd + 9);
        char *sp = strchr(cmd + 9, ' ');
        if (sp && lat != 0) {
            double lon = atof(sp + 1);
            m10q_seed_position(lat, lon);
            printf("gnssseed: seeded %.5f, %.5f\n", lat, lon);
        } else {
            printf("gnssseed: usage gnssseed <lat> <lon>\n");
        }
    } else if (strcmp(cmd, "lpk") == 0) {
        double lat = 0, lon = 0;
        m10q_get_last_position(&lat, &lon);
        if (lat == 0 && lon == 0) {
            printf("lpk: none stored yet\n");
        } else {
            printf("lpk: %.5f, %.5f\n", lat, lon);
        }
    } else if (strcmp(cmd, "track") == 0) {
#if TRACKING_ENABLED
        lvgl_tracking_start();
        printf("track: started (GNSS pulses every 50 steps)\n");
#else
        printf("track: disabled (TRACKING_ENABLED=0)\n");
#endif
    } else if (strcmp(cmd, "cachedump") == 0) {
        sensor_cache_t c;
        sensor_cache_get(&c);
        printf("cache: valid=%d rtc=%04u-%02u-%02u %02u:%02u:%02u wd=%u\n",
               (int)c.valid, (unsigned)c.rtc.year, (unsigned)c.rtc.month,
               (unsigned)c.rtc.day, (unsigned)c.rtc.hour, (unsigned)c.rtc.min,
               (unsigned)c.rtc.sec, (unsigned)c.rtc.weekday);
        printf("cache: batt=%u%% %umV chg=%d en=%d ma=%u temp=%d.%dC\n",
               (unsigned)c.batt_pct, (unsigned)c.batt_mv, (int)c.chg_state,
               (int)c.chg_enabled, (unsigned)c.chg_ma,
               c.batt_temp_c10 / 10, abs(c.batt_temp_c10 % 10));
        battery_estimate_t e;
        battery_estimate_get(&e);
        printf("batt est: valid=%d pct_per_hour=%.2f runtime_h=%.1f charge_h=%.1f\n",
               (int)e.estimate_valid, (double)e.pct_per_hour,
               (double)e.runtime_h, (double)e.charge_h);
    } else if (strcmp(cmd, "track stop") == 0) {
        lvgl_tracking_stop();
        printf("track: stopped\n");
    } else if (strcmp(cmd, "trackstat") == 0) {
        tracking_totals_t t;
        tracking_get_totals(&t);
        printf("track: active=%d dist=%.2f km steps=%lu avg=%.2f m session_steps=%lu\n",
               (int)tracking_is_active(),
               t.dist_cm / 100000.0, (unsigned long)t.steps,
               tracking_get_avg_step_cm() / 100.0,
               (unsigned long)tracking_get_session_steps());
    } else if (strcmp(cmd, "gnssraw") == 0) {
        printf("gnssraw: dumping raw UART for 8 s...\n");
        m10q_set_raw_dump(true);
        vTaskDelay(pdMS_TO_TICKS(8000));
        m10q_set_raw_dump(false);
        printf("gnssraw: dump stopped\n");
    } else if (strcmp(cmd, "motor") == 0) {
        /* Verify the haptic motor: enable the DRV2605 rail and fire a
         * short vibration. Usage: "motor" or "motor 47" (waveform id).
         * Default 47 = strong click (library 1). */
        int wave = 47;
        char *sp = strchr(cmd, ' ');
        if (sp) {
            wave = atoi(sp + 1);
        }
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_err_t err = drv2605_play(twatch_haptic_dev, (uint8_t)wave);
        printf("motor: wave=%d %s\n", wave, (err == ESP_OK) ? "ok" : esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(400));
        drv2605_go(twatch_haptic_dev);
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
    } else if (strcmp(cmd, "motor cal") == 0) {
        /* Run the on-chip auto-calibration and save to NVS. The motor
         * vibrates for ~1 s during calibration. */
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_err_t err = drv2605_auto_calibrate(twatch_haptic_dev);
        printf("motor cal: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
    } else if (strcmp(cmd, "motor calrestore") == 0) {
        esp_err_t err = drv2605_calibrate_restore(twatch_haptic_dev);
        printf("motor calrestore: %s\n",
               (err == ESP_OK) ? "ok" : (err == ESP_ERR_NOT_FOUND) ? "none stored" : esp_err_to_name(err));
    } else if (strcmp(cmd, "crashinfo") == 0) {
        crash_dump_print_status();
    } else if (strcmp(cmd, "pm") == 0) {
        /* Dump the power-management settings. */
        printf("pm: night_auto=%d skip_usb=%d night=%d\n",
               power_mgmt_get_night_mode_auto() ? 1 : 0,
               power_mgmt_get_skip_sleep_on_usb() ? 1 : 0,
               power_mgmt_is_night_mode() ? 1 : 0);
    } else if (strncmp(cmd, "bat ", 4) == 0 && strcmp(cmd + 4, "on") == 0) {
        esp_err_t e = axp2101_set_charge_enabled(twatch_pmu_dev, true);
        printf("bat: charge enable -> %s\n", (e == ESP_OK) ? "ok" : esp_err_to_name(e));
    } else if (strncmp(cmd, "bat ", 4) == 0 && strcmp(cmd + 4, "off") == 0) {
        esp_err_t e = axp2101_set_charge_enabled(twatch_pmu_dev, false);
        printf("bat: charge disable -> %s\n", (e == ESP_OK) ? "ok" : esp_err_to_name(e));
    } else if (strcmp(cmd, "bat") == 0) {
        uint16_t mv = 0;
        uint8_t pct = 0;
        bool vbus = false, chg_en = false;
        axp2101_get_battery_mv(twatch_pmu_dev, &mv);
        axp2101_get_battery_pct(twatch_pmu_dev, &pct);
        axp2101_is_vbus_present(twatch_pmu_dev, &vbus);
        axp2101_is_charge_enabled(twatch_pmu_dev, &chg_en);
        axp2101_charge_state_t st;
        axp2101_get_charge_status(twatch_pmu_dev, &st);
        uint16_t ma = 0;
        axp2101_get_charge_current_ma(twatch_pmu_dev, &ma);
        const char *s = st == AXP2101_CHG_TRI ? "trickle" : st == AXP2101_CHG_PRE ? "pre"
                       : st == AXP2101_CHG_CC ? "cc" : st == AXP2101_CHG_CV ? "cv"
                       : st == AXP2101_CHG_DONE ? "done" : "stop";
        printf("bat: %umV %u%% vbus=%d chg_en=%d chg=%s cur=%umA\n",
               (unsigned)mv, (unsigned)pct, vbus ? 1 : 0, chg_en ? 1 : 0, s,
               (unsigned)ma);
    } else if (strcmp(cmd, "dispchk") == 0) {
        uint8_t ldo0 = 0, aldo2_vol = 0;
        axp2101_read_reg(twatch_pmu_dev, 0x90, &ldo0);
        axp2101_read_reg(twatch_pmu_dev, 0x93, &aldo2_vol);
        uint16_t xl = 0;
        xl9555_read_port(twatch_xl9555_dev, &xl);
        printf("disp: LDO_ONOFF0=0x%02x (ALDO2=%d) ALDO2_vol=0x%02x "
               "XL9555=0x%04x (DISP_PWR=%d)\n",
               ldo0, (ldo0 >> 1) & 1, aldo2_vol, xl, (xl >> 7) & 1);
    } else if (strcmp(cmd, "disppwr") == 0) {
        /* Cycle the display power rail off then on to recover a stuck panel.
         * The panel re-inits from its OTP on power-up; re-send wake commands
         * through the existing handle (no re-init, the SPI bus stays owned by
         * the LVGL flush task). */
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_DISP_PWR, false);
        vTaskDelay(pdMS_TO_TICKS(200));
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_DISP_PWR, true);
        vTaskDelay(pdMS_TO_TICKS(200));
        co5300_wake();
        lvgl_force_redraw();
        co5300_display_on();
        co5300_set_brightness(0x80);
        printf("disppwr: display power cycled\n");
    } else if (strcmp(cmd, "dailylog") == 0) {
        uint32_t steps = 0;
        daily_log_get_steps(&steps);
        const uint16_t *min[DAILY_ACT_COUNT];
        daily_log_get_activity_minutes(min);
        static const char *names[DAILY_ACT_COUNT] = {
            "still", "walking", "running", "cycling", "vehicle", "tilting", "unknown" };
        printf("dailylog: day_steps=%lu\n", (unsigned long)steps);
        for (int i = 0; i < DAILY_ACT_COUNT; i++) {
            printf("dailylog: %-8s %u min\n", names[i], (unsigned)*min[i]);
        }
        daily_log_flush();
        printf("dailylog: sd=%d\n", sd_log_available() ? 1 : 0);
    } else if (strcmp(cmd, "gnssprobe") == 0) {
        int8_t r = bhi260ap_gnss_inject_probe();
        printf("gnssprobe: inject_mode_rslt=%d (%s)\n", (int)r,
               (r == 0) ? "supported" : "NOT supported");
    } else if (strncmp(cmd, "pm night ", 9) == 0) {
        power_mgmt_set_night_mode_auto(atoi(cmd + 9) != 0);
        printf("pm: night_auto=%d\n", power_mgmt_get_night_mode_auto() ? 1 : 0);
    } else if (strncmp(cmd, "pm usb ", 7) == 0) {
        power_mgmt_set_skip_sleep_on_usb(atoi(cmd + 7) != 0);
        printf("pm: skip_usb=%d\n", power_mgmt_get_skip_sleep_on_usb() ? 1 : 0);
    } else if (strcmp(cmd, "crashsave") == 0) {
        esp_err_t err = crash_dump_save();
        printf("crashsave: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
    } else if (strcmp(cmd, "crashls") == 0) {
        DIR *d = opendir("/sdcard/log/crash");
        if (!d) {
            printf("crashls: no crash dir\n");
        } else {
            struct dirent *e;
            while ((e = readdir(d)) != NULL) {
                printf("%s\n", e->d_name);
            }
            closedir(d);
        }
    } else if (strncmp(cmd, "crashread ", 10) == 0) {
        size_t name_len = strlen(cmd + 10);
        if (name_len == 0 || name_len >= 64) {
            printf("crashread: invalid filename\n");
        } else {
            char name[64];
            memcpy(name, cmd + 10, name_len);
            name[name_len] = '\0';
            char path[128];
            snprintf(path, sizeof(path), "/sdcard/log/crash/%s", name);
            FILE *f = fopen(path, "r");
            if (!f) {
                printf("crashread: cannot open %s\n", path);
            } else {
                char c;
                while (fread(&c, 1, 1, f) == 1) {
                    putchar(c);
                }
                fclose(f);
            }
        }
    } else if (strncmp(cmd, "tone ", 5) == 0 || strcmp(cmd, "tone") == 0) {
        /* Play a sine tone. Usage: "tone" (440 Hz, 500 ms, near-max vol)
         * or "tone <hz> <ms>" or "tone <hz> <ms> <amp 0-32767>". */
        int hz = 440, ms = 500, amp = 30000;
        char *sp = strchr(cmd, ' ');
        if (sp) {
            hz = atoi(sp + 1);
            sp = strchr(sp + 1, ' ');
            if (sp) {
                ms = atoi(sp + 1);
                sp = strchr(sp + 1, ' ');
                if (sp) {
                    amp = atoi(sp + 1);
                }
            }
        }
        if (hz <= 0) {
            hz = 440;
        }
        if (ms <= 0) {
            ms = 500;
        }
        if (amp <= 0 || amp > 32767) {
            amp = 30000;
        }
        size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);
        int16_t *buf = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!buf) {
            printf("tone: no mem for %u samples\n", (unsigned)n);
        } else {
            for (size_t i = 0; i < n; i++) {
                buf[i] = (int16_t)(sinf(2.0f * 3.14159265f * hz * i / AUDIO_SAMPLE_RATE) * amp);
            }
            printf("tone: %d Hz, %d ms, amp %d (%u samples)\n", hz, ms, amp, (unsigned)n);
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);   /* amp */
            esp_err_t err = max98357a_write(buf, n);
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
            printf("tone: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
            heap_caps_free(buf);
        }
    } else if (strncmp(cmd, "rec ", 4) == 0 || strcmp(cmd, "rec") == 0) {
        /* Record a short mono clip to a PSRAM buffer.
         * Usage: "rec" (2 s) or "rec <ms>". */
        int ms = 2000;
        char *sp = strchr(cmd, ' ');
        if (sp) {
            ms = atoi(sp + 1);
        }
        if (ms <= 0) {
            ms = 2000;
        }
        if (ms > 10000) {
            ms = 10000;
        }
        size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);
        if (!rec_ensure_buf(n)) {
            printf("rec: no mem for %u samples\n", (unsigned)n);
        } else {
            s_rec_n = n;
            printf("rec: recording %d ms (%u samples)...\n", ms, (unsigned)n);
            esp_err_t err = t3902_read(s_rec_buf, n);
            if (err == ESP_OK) {
                /* Report the peak amplitude so we can tell if the mic
                 * captured real audio vs silence. */
                int32_t peak = 0;
                for (size_t i = 0; i < n; i++) {
                    int32_t v = s_rec_buf[i];
                    if (v < 0) v = -v;
                    if (v > peak) peak = v;
                }
                printf("rec: done, peak amp %ld\n", (long)peak);
            } else {
                printf("rec: %s\n", esp_err_to_name(err));
            }
        }
    } else if (strcmp(cmd, "playrec") == 0) {
        /* Play back the last recording (loops 3x so it's audible). */
        if (!s_rec_buf || s_rec_n == 0) {
            printf("playrec: nothing recorded yet (use rec first)\n");
        } else {
            printf("playrec: playing %u samples x3\n", (unsigned)s_rec_n);
            for (int r = 0; r < 3; r++) {
                esp_err_t err = max98357a_write(s_rec_buf, s_rec_n);
                if (err != ESP_OK) {
                    printf("playrec: %s\n", esp_err_to_name(err));
                    break;
                }
            }
            printf("playrec: done\n");
        }
    } else if (strncmp(cmd, "tonerec ", 8) == 0 || strcmp(cmd, "tonerec") == 0) {
        /* Play a single tone and record it with the mic (like sweep,
         * but fixed frequency). Usage: "tonerec" (440 Hz, 3 s) or
         * "tonerec <hz> <ms> <amp>". */
        int hz = 440, ms = 3000, amp = 30000;
        char *sp = strchr(cmd, ' ');
        if (sp) {
            hz = atoi(sp + 1);
            sp = strchr(sp + 1, ' ');
            if (sp) {
                ms = atoi(sp + 1);
                sp = strchr(sp + 1, ' ');
                if (sp) {
                    amp = atoi(sp + 1);
                }
            }
        }
        if (hz < 20) hz = 440;
        if (ms <= 0) ms = 3000;
        if (ms > 10000) ms = 10000;
        if (amp <= 0 || amp > 32767) amp = 30000;
        size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);

        int16_t *tone = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!tone) {
            printf("tonerec: no mem for %u samples\n", (unsigned)n);
        } else {
            for (size_t i = 0; i < n; i++) {
                tone[i] = (int16_t)(sinf(2.0f * 3.14159265f * hz * i / AUDIO_SAMPLE_RATE) * amp);
            }
            if (!s_rec_done) {
                s_rec_done = xSemaphoreCreateBinary();
            }
            xSemaphoreTake(s_rec_done, 0);
            if (!rec_ensure_buf(n)) {
                printf("tonerec: no mem for rec buffer\n");
                heap_caps_free(tone);
            } else {
                s_rec_n = n;
                printf("tonerec: %d Hz, %d ms, amp %d (playing + recording)\n", hz, ms, amp);
                xTaskCreate(rec_task, "tone_rec", 2048, NULL, 5, NULL);
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_err_t err = max98357a_write(tone, n);
                printf("tonerec: playback %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
                if (xSemaphoreTake(s_rec_done, pdMS_TO_TICKS(n * 1000 / AUDIO_SAMPLE_RATE + 5000)) != pdTRUE) {
                    printf("tonerec: rec timed out\n");
                } else {
                    int32_t peak = 0;
                    for (size_t i = 0; i < n; i++) {
                        int32_t v = s_rec_buf[i];
                        if (v < 0) v = -v;
                        if (v > peak) peak = v;
                    }
                    printf("tonerec: recorded, peak amp %ld (play 'playrec' to hear)\n", (long)peak);
                }
                heap_caps_free(tone);
            }
        }
    } else if (strncmp(cmd, "sweep ", 6) == 0 || strcmp(cmd, "sweep") == 0) {
        /* Play a log-frequency sweep and record it with the mic.
         * Usage: "sweep" (200..8000 Hz, 3 s) or "sweep <f0> <f1> <ms>". */
        int f0 = 200, f1 = 8000, ms = 3000, amp = 30000;
        char *sp = strchr(cmd, ' ');
        if (sp) {
            f0 = atoi(sp + 1);
            sp = strchr(sp + 1, ' ');
            if (sp) {
                f1 = atoi(sp + 1);
                sp = strchr(sp + 1, ' ');
                if (sp) {
                    ms = atoi(sp + 1);
                }
            }
        }
        if (f0 < 20) f0 = 20;
        if (f1 <= f0) f1 = f0 + 100;
        if (ms <= 0) ms = 3000;
        if (ms > 10000) ms = 10000;
        size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);
        int f0_start = f0, f1_end = f1;

        /* Generate a logarithmic sine sweep f0 -> f1 over the duration.
         * The running frequency must be a double: incrementing an int by
         * the per-sample factor (~1.00008) truncates and never moves. */
        int16_t *sweep = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!sweep) {
            printf("sweep: no mem for %u samples\n", (unsigned)n);
        } else {
            double k = exp(log((double)f1 / f0) / n);
            double f = f0;
            double ph = 0.0;
            for (size_t i = 0; i < n; i++) {
                sweep[i] = (int16_t)(sinf(ph) * amp);
                ph += 2.0 * 3.14159265358979 * f / AUDIO_SAMPLE_RATE;
                f *= k;
            }
            if (!s_rec_done) {
                s_rec_done = xSemaphoreCreateBinary();
            }
            xSemaphoreTake(s_rec_done, 0);
            /* (Re)size the recording buffer to match. */
            if (!rec_ensure_buf(n)) {
                printf("sweep: no mem for rec buffer\n");
                heap_caps_free(sweep);
            } else {
                s_rec_n = n;
                printf("sweep: %d -> %d Hz, %d ms (playing + recording)\n", f0_start, f1_end, ms);
                /* Start the mic first so it captures the whole sweep. */
                xTaskCreate(rec_task, "sweep_rec", 2048, NULL, 5, NULL);
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_err_t err = max98357a_write(sweep, n);
                printf("sweep: playback %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
                /* Wait for the recording to finish. */
                if (xSemaphoreTake(s_rec_done, pdMS_TO_TICKS(n * 1000 / AUDIO_SAMPLE_RATE + 5000)) != pdTRUE) {
                    printf("sweep: rec timed out\n");
                } else {
                    int32_t peak = 0;
                    for (size_t i = 0; i < n; i++) {
                        int32_t v = s_rec_buf[i];
                        if (v < 0) v = -v;
                        if (v > peak) peak = v;
                    }
                    printf("sweep: recorded, peak amp %ld (play 'playrec' to hear)\n", (long)peak);
                }
                heap_caps_free(sweep);
            }
        }
    } else if (strcmp(cmd, "alarm") == 0) {
        /* Status. */
        alarm_config_t ac;
        alarm_get_config(&ac);
        printf("alarm: %s %02u:%02u mode %u ringing=%d armed=%d\n",
               ac.enabled ? "armed" : "disabled", (unsigned)ac.hour, (unsigned)ac.min,
               (unsigned)ac.ring_mode, (int)alarm_is_ringing(), (int)alarm_is_armed());
    } else if (strncmp(cmd, "alarm ", 6) == 0) {
        /* alarm HH:MM [beep|vib|both]  or  alarm off */
        const char *rest = cmd + 6;
        if (strcmp(rest, "off") == 0) {
            alarm_set(0, 0, false, ALARM_RING_BEEP);
            printf("alarm: disabled\n");
        } else {
            int hh = atoi(rest);
            const char *sp = strchr(rest, ' ');
            if (sp && hh >= 0 && hh <= 23) {
                int mm = atoi(sp + 1);
                uint8_t mode = ALARM_RING_BEEP;
                const char *sp2 = strchr(sp + 1, ' ');
                if (sp2) {
                    if (strncmp(sp2 + 1, "vib", 3) == 0) mode = ALARM_RING_VIB;
                    else if (strncmp(sp2 + 1, "both", 4) == 0) mode = ALARM_RING_BOTH;
                }
                alarm_set((uint8_t)hh, (uint8_t)mm, true, mode);
                printf("alarm: set %02d:%02d mode %u\n", hh, mm, (unsigned)mode);
            } else {
                printf("alarm: usage alarm <hh> <mm> [beep|vib|both] | alarm off\n");
            }
        }
    } else if (strcmp(cmd, "alarmring") == 0) {
        /* Fire the ring now (test). */
        alarm_ring_test();
        printf("alarm: ring test\n");
    } else if (strcmp(cmd, "alarmdismiss") == 0) {
        alarm_dismiss();
        printf("alarm: dismissed\n");
    } else if (strcmp(cmd, "alarmsnooze") == 0) {
        alarm_snooze();
        printf("alarm: snoozed\n");
    } else if (strcmp(cmd, "rtcdump") == 0) {
        pcf85063a_debug_dump(twatch_rtc_dev);
    } else if (strncmp(cmd, "rtctimer ", 9) == 0) {
        /* Test the PCF85063A countdown-timer hardware directly: arm a short
         * timer and watch TF set when it expires (the snooze re-ring path).
         * Expect the ring to start when the timer fires. */
        int secs = atoi(cmd + 9);
        if (secs < 1 || secs > 255) {
            printf("rtctimer: usage: rtctimer <1..255 sec>\n");
        } else {
            esp_err_t e = pcf85063a_set_timer_seconds(twatch_rtc_dev, (uint16_t)secs, true);
            printf("rtctimer: armed %d s countdown: %s\n", secs,
                   e == ESP_OK ? "ok (TIE on)" : esp_err_to_name(e));
        }
    } else if (strcmp(cmd, "rtctimer") == 0) {
        printf("rtctimer: usage: rtctimer <1..255 sec>\n");
    } else if (strcmp(cmd, "ble") == 0) {
        ble_debug_print_status();
    } else if (strcmp(cmd, "bleadv") == 0) {
        ble_debug_set_advertising(true);
        printf("bleadv: advertising enabled\n");
    } else if (strcmp(cmd, "bleadvoff") == 0) {
        ble_debug_set_advertising(false);
        printf("bleadvoff: advertising disabled\n");
    } else if (strcmp(cmd, "panictest") == 0) {
        /* Deliberately crash to exercise the core dump -> SD path. */
        printf("panictest: triggering a null-pointer dereference...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        volatile uint32_t *p = (volatile uint32_t *)0;
        *p = 0xDEADBEEF;
        printf("panictest: should never reach here\n");
    } else if (cmd[0] != '\0') {
        printf("unknown command: %s\n", cmd);
    }
}

void uwatch_debug_process_cmd(const char *cmd)
{
    debug_process_cmd(cmd);
}

void app_main(void)
{
    /* Set the local timezone so UTC<->local conversions (RTC sync from GNSS,
     * MGA-INI aiding) are correct. */
    setenv("TZ", "CET-1CEST-2,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
    }

    err = twatch_board_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "board init reported error 0x%x (%s), continuing", err, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "UWatch boot complete");

    /* SD card logging (best effort; serial-only if no card). Log is reset on
     * each new firmware build (version change). */
    sd_log_set_version(UWATCH_GIT_HASH);
    sd_log_mount();
    sd_log_start();

    /* Per-minute steps + activity logging to the SD card (daily_log.h). */
    daily_log_init();

    /* If the previous boot crashed, decode the flash core dump to the SD card
     * (report + raw ELF) before the UI starts. */
    crash_dump_save();

    err = lvgl_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl start failed: %s", esp_err_to_name(err));
    }

    /* Debug command loop over USB-Serial-JTAG. */
    xTaskCreate(debug_task, "dbg", DBG_TASK_STACK, NULL, 5, NULL);

    /* BLE debug bridge is currently disabled: the BT controller reserves DMA
     * that the 48-row display buffer needs, causing BLE connections to drop.
     * Re-enable when BLE can coexist (e.g. by shrinking the draw buffer). */
    /* ble_debug_init(); */
}
