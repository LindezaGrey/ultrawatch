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
#include "twatch_board.h"
#include "lvgl_app.h"
#include "sd_log.h"
#include "crash_dump.h"
#include "daily_log.h"
#include "mesh_log.h"
#include "uwatch_main.h"
#include "debug_audio.h"
#include "debug_bhi.h"
#include "debug_gnss.h"
#include "debug_cmds.h"
#include <stdio.h>

static const char *TAG = "uwatch";

#define DBG_RX_BUF   256
#define DBG_TASK_STACK 4096

static void debug_process_cmd(const char *cmd);

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
        /* Process complete lines. Accept bare \r, bare \n, or \r\n as the
         * line terminator - idf.py monitor's underlying miniterm sends \r
         * only on Enter, not \n, so treating \n as the sole terminator left
         * every typed command sitting unprocessed in the buffer forever
         * (only flushed, garbled, once something else eventually sent a
         * real \n). Stopping at the first \r or \n and skipping the empty
         * line a \r\n pair produces handles all three cases correctly. */
        for (;;) {
            size_t term = len;
            for (size_t i = 0; i < len; i++) {
                if (line[i] == '\r' || line[i] == '\n') {
                    term = i;
                    break;
                }
            }
            if (term == len) {
                break;   /* no terminator yet, wait for more bytes */
            }
            size_t cmd_len = term;
            while (cmd_len > 0 && line[cmd_len - 1] == ' ') {
                cmd_len--;
            }
            line[cmd_len] = '\0';
            if (cmd_len > 0) {
                debug_process_cmd(line);
            }
            /* Shift remaining bytes. */
            size_t rest = len - term - 1;
            memmove(line, line + term + 1, rest);
            len = rest;
        }
    }
}


/* Master dispatch table for console commands. Each entry's handler receives
 * the text after the command's first token (the "args" remainder), with
 * command-specific sub-verb parsing (if any) done inside the handler. Shared
 * by the USB-Serial-JTAG console and the BLE debug bridge so both can drive
 * the watch. */
typedef struct {
    const char *name;
    void (*handler)(const char *args);
} debug_cmd_entry_t;

static const debug_cmd_entry_t s_commands[] = {
    { "shot",           debug_cmd_shot },
    { "sdls",           debug_cmd_sdls },
    { "sdclear",        debug_cmd_sdclear },
    { "heap",           debug_cmd_heap },
    { "bhi",            debug_bhi_bhi },
    { "suspend",        debug_bhi_suspend },
    { "resume",         debug_bhi_resume },
    { "imon",           debug_bhi_imon },
    { "wudump",         debug_bhi_wudump },
    { "wusus",          debug_bhi_wusus },
    { "wudis",          debug_bhi_wudis },
    { "metahist",       debug_bhi_metahist },
    { "gnsson",         debug_gnss_on },
    { "gnssoff",        debug_gnss_off },
    { "gnssver",        debug_gnss_ver },
    { "rtccal",         debug_gnss_rtccal },
    { "gnss",           debug_gnss_status },
    { "gpscheck",       debug_gnss_check },
    { "gnssseed",       debug_gnss_seed },
    { "lpk",            debug_gnss_lpk },
    { "track",          debug_gnss_track },
    { "cachedump",      debug_gnss_cachedump },
    { "trackstat",      debug_gnss_trackstat },
    { "gnssraw",        debug_gnss_raw },
    { "motor",          debug_cmd_motor },
    { "crashinfo",      debug_cmd_crashinfo },
    { "pm",             debug_cmd_pm },
    { "pwroff",         debug_cmd_pwroff },
    { "rails",          debug_cmd_rails },
    { "nfcpoll",        debug_cmd_nfcpoll },
    { "bat",            debug_cmd_bat },
    { "dispchk",        debug_cmd_dispchk },
    { "disppwr",        debug_cmd_disppwr },
    { "dailylog",       debug_cmd_dailylog },
    { "gnssprobe",      debug_gnss_probe },
    { "sensorlist",     debug_bhi_sensorlist },
    { "bhishow",        debug_bhi_bhishow },
    { "grate",          debug_bhi_grate },
    { "crashsave",      debug_cmd_crashsave },
    { "crashls",        debug_cmd_crashls },
    { "crashread",      debug_cmd_crashread },
    { "tone",           debug_audio_tone },
    { "rec",            debug_audio_rec },
    { "playrec",        debug_audio_playrec },
    { "tonerec",        debug_audio_tonerec },
    { "sweep",          debug_audio_sweep },
    { "alarm",          debug_cmd_alarm },
    { "alarmring",      debug_cmd_alarmring },
    { "alarmdismiss",   debug_cmd_alarmdismiss },
    { "alarmsnooze",    debug_cmd_alarmsnooze },
    { "rtcdump",        debug_cmd_rtcdump },
    { "rtctimer",       debug_cmd_rtctimer },
    { "settime",        debug_cmd_settime },
    { "ble",            debug_cmd_ble },
    { "bleadv",         debug_cmd_bleadv },
    { "bleadvoff",      debug_cmd_bleadvoff },
    { "panictest",      debug_cmd_panictest },
};
#define NUM_DEBUG_COMMANDS (sizeof(s_commands) / sizeof(s_commands[0]))

static void debug_process_cmd(const char *cmd)
{
    /* Split on the first space: name = first token, args = remainder (""
     * if there was no space). args points into the caller's cmd buffer past
     * the delimiting space, so handlers see the same text the old strncmp
     * branches computed via cmd + strlen("foo "). */
    const char *sp = strchr(cmd, ' ');
    size_t name_len = sp ? (size_t)(sp - cmd) : strlen(cmd);
    const char *args = sp ? sp + 1 : "";

    char name[32];
    if (name_len >= sizeof(name)) {
        name_len = sizeof(name) - 1;
    }
    memcpy(name, cmd, name_len);
    name[name_len] = '\0';

    for (size_t i = 0; i < NUM_DEBUG_COMMANDS; i++) {
        if (strcmp(name, s_commands[i].name) == 0) {
            s_commands[i].handler(args);
            return;
        }
    }
    if (cmd[0] != '\0') {
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

    /* Mount the SD card (best effort) for crash dumps, daily activity logs,
     * and screenshots - see sd_log.h. */
    sd_log_mount();

    /* Per-minute steps + activity logging to the SD card (daily_log.h). */
    daily_log_init();

    /* Always-on background Meshtastic listener (mesh_log.h). */
    mesh_log_init();

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
