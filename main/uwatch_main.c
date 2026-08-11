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
#include "m10q.h"
#include "drv2605.h"
#include "xl9555.h"
#include <stdio.h>
#include <dirent.h>

static const char *TAG = "uwatch";

#define DBG_RX_BUF   256
#define DBG_TASK_STACK 4096

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

            if (strcmp(line, "shot") == 0) {
                lvgl_app_dump_screenshot();
            } else if (strcmp(line, "sdin") == 0) {
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
            } else if (strcmp(line, "sdls") == 0) {
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
            } else if (strcmp(line, "sdclear") == 0) {
                /* Truncate the log and delete screenshots. */
                esp_err_t err = sd_log_clear();
                printf("sdclear: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
            } else if (strcmp(line, "heap") == 0) {
                printf("heap: free=%lu min=%lu dma=%lu\n",
                       (unsigned long)esp_get_free_heap_size(),
                       (unsigned long)esp_get_minimum_free_heap_size(),
                       (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
            } else if (strcmp(line, "bhi") == 0) {
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
            } else if (strcmp(line, "suspend") == 0) {
                bhi260ap_ap_suspend();
                printf("suspend: done\n");
            } else if (strcmp(line, "resume") == 0) {
                bhi260ap_ap_resume();
                printf("resume: done\n");
            } else if (strcmp(line, "imon") == 0) {
                /* Watch the BHI INT line (GPIO8) for ~30 s, print each change. */
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
            } else if (strcmp(line, "gnsson") == 0) {
                m10q_power(true);
                printf("gnss: powered on\n");
            } else if (strcmp(line, "gnssoff") == 0) {
                m10q_power(false);
                printf("gnss: powered off\n");
            } else if (strcmp(line, "gnss") == 0) {
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
                if (m10q_get_agc(&agc) == ESP_OK) {
                    printf("gnss: agc=%u\n", (unsigned)agc);
                }
            } else if (strcmp(line, "motor") == 0) {
                /* Verify the haptic motor: enable the DRV2605 rail and fire a
                 * short vibration. Usage: "motor" or "motor 47" (waveform id).
                 * Default 47 = strong click (library 1). */
                int wave = 47;
                char *sp = strchr(line, ' ');
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
            } else if (cmd_len > 0) {
                printf("unknown command: %s\n", line);
            }

            /* Shift remaining bytes. */
            size_t rest = len - (size_t)(nl - line) - 1;
            memmove(line, nl + 1, rest);
            len = rest;
        }
    }
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

    err = lvgl_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl start failed: %s", esp_err_to_name(err));
    }

    /* Debug command loop over USB-Serial-JTAG. */
    xTaskCreate(debug_task, "dbg", DBG_TASK_STACK, NULL, 5, NULL);
}
