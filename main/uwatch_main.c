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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "twatch_board.h"
#include "lvgl_app.h"
#include "sd_log.h"
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

    for (;;) {
        int n = usb_serial_jtag_read_bytes(line + len, sizeof(line) - len - 1, pdMS_TO_TICKS(50));
        if (n < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (n == 0) {
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
    esp_err_t err = twatch_board_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "board init reported error 0x%x (%s), continuing", err, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "UWatch boot complete");

    /* SD card logging (best effort; serial-only if no card). */
    sd_log_mount();
    sd_log_start();

    err = lvgl_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl start failed: %s", esp_err_to_name(err));
    }

    /* Debug command loop over USB-Serial-JTAG. */
    xTaskCreate(debug_task, "dbg", DBG_TASK_STACK, NULL, 5, NULL);
}
