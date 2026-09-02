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
#include "power_mgmt.h"
#include "lvgl_app.h"
#include "crash_dump.h"
#include "housekeeping.h"
#include "mesh_log.h"
#include "haptic.h"
#include "syslog_capture.h"
#include "wifi_scan.h"
#include "ble_scan.h"
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
    { "catbin",         debug_cmd_catbin },
    { "synclog",        debug_cmd_synclog },
    { "sdclear",        debug_cmd_sdclear },
    { "heap",           debug_cmd_heap },
    { "stacks",         debug_cmd_stacks },
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
    { "sparmodus",      debug_cmd_sparmodus },
    { "pwroff",         debug_cmd_pwroff },
    { "rails",          debug_cmd_rails },
    { "nfcpoll",        debug_cmd_nfcpoll },
    { "nfcprobe",       debug_cmd_nfcprobe },
    { "i2cscan",        debug_cmd_i2cscan },
    { "bat",            debug_cmd_bat },
    { "dispchk",        debug_cmd_dispchk },
    { "disppwr",        debug_cmd_disppwr },
    { "dispfix",        debug_cmd_dispfix },
    { "dispreg",        debug_cmd_dispreg },
    { "dispcycle",      debug_cmd_dispcycle },
    { "dispte",         debug_cmd_dispte },
    { "lvglinfo",       debug_cmd_lvglinfo },
    { "disprepaint",    debug_cmd_disprepaint },
    { "dispcmd",        debug_cmd_dispcmd },
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
    { "alarmls",        debug_cmd_alarmls },
    { "alarmadd",       debug_cmd_alarmadd },
    { "alarmrm",        debug_cmd_alarmrm },
    { "alarmen",        debug_cmd_alarmen },
    { "alarmring",      debug_cmd_alarmring },
    { "alarmdismiss",   debug_cmd_alarmdismiss },
    { "alarmsnooze",    debug_cmd_alarmsnooze },
    { "timer",          debug_cmd_timer },
    { "gpxstatus",      debug_cmd_gpxstatus },
    { "gpxcat",         debug_cmd_gpxcat },
    { "meshnodes",      debug_cmd_meshnodes },
    { "meshcat",        debug_cmd_meshcat },
    { "dailycat",       debug_cmd_dailycat },
    { "presetset",      debug_cmd_presetset },
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
    /* Ultra-Sparmodus's silent per-minute wake (docs/application.md
     * section 10.3): re-arm and go straight back to deep sleep, touching
     * nothing else - no NVS, no board bring-up, no display. Must be the
     * very first thing in app_main(), before any other init, since this
     * check itself needs none of it (the RTC_DATA_ATTR flag it reads
     * survives a deep-sleep wake without re-loading anything). Never
     * returns when true. */
    if (power_mgmt_sparmodus_should_resleep_silently()) {
        power_mgmt_sparmodus_resleep();
    }
    /* Snapshot this BEFORE any other init: power_mgmt_init() (inside
     * lvgl_app_start() below) clears the underlying RTC_DATA_ATTR flag as
     * part of the normal "this boot is proceeding past the silent-resleep
     * fork" bookkeeping, so it has to be read now or the information is
     * gone by the time it's needed after lvgl_app_start() returns. */
    bool sparmodus_explicit_wake = power_mgmt_sparmodus_was_deep_sleep_wake();

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

    /* Mirror ESP_LOGx output to the SD card (syslog_capture.h) - installed
     * as early as NVS/the SD card's own rail bookkeeping allow, so a
     * transient glitch (e.g. the white-screen DMA-retry warnings) leaves
     * durable evidence instead of only a live serial console that's easy
     * to miss or lose on reconnect. */
    syslog_capture_init();

    /* SD card is mounted on demand, per write (sd_log_session_begin()/end(),
     * see sd_log.h) - crash dumps, daily activity logs, GPX/mesh logging,
     * and screenshots each bracket their own I/O rather than relying on a
     * persistent boot-time mount, so the card sits unmounted (and safe from
     * an unclean power loss) whenever nothing is actively being written. */

    /* Global vibration pattern selection (Settings > Ton & Vibration),
     * shared by alarm/timer ring and LoRa notification (haptic.h) - must
     * come before either of those can possibly fire. */
    haptic_init();

    /* Shared background task for per-minute steps/activity logging
     * (daily_log.h), 30s GPX trackpoint logging (gpx_log.h), and syslog
     * flushing - must come after syslog_capture_init() above (housekeeping.h). */
    housekeeping_init();

    /* Always-on background Meshtastic listener (mesh_log.h). */
    mesh_log_init();

    /* WiFi scanning, off by default (wifi_scan.h) - started/stopped from
     * the WiFi screen's power switch. */
    wifi_scan_init();

    /* Bluetooth scan, off by default (ble_scan.h) - started/stopped from
     * the Bluetooth screen's power switch. Brings up the NimBLE host
     * stack only on first use, not at boot - see ble_scan.h's header
     * comment on the DMA-conflict risk this carries. */
    ble_scan_init();

    /* If the previous boot crashed, decode the flash core dump to the SD card
     * (report + raw ELF) before the UI starts. */
    crash_dump_save();

    err = lvgl_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl start failed: %s", esp_err_to_name(err));
    }

    /* This is an explicit wake out of an active Ultra-Sparmodus deep-sleep
     * session (not a cold boot) - arms the BOOT-hold-to-exit watcher for
     * the rest of this awake session (docs/application.md section 10.4).
     * Returns immediately; how long the watch stays awake before
     * automatically re-entering deep sleep is the normal auto-sleep idle
     * timeout, not this call - see power_mgmt_sparmodus_handle_wake(). */
    if (err == ESP_OK && sparmodus_explicit_wake) {
        power_mgmt_sparmodus_handle_wake();
    }

    /* Debug command loop over USB-Serial-JTAG. */
    xTaskCreate(debug_task, "dbg", DBG_TASK_STACK, NULL, 5, NULL);

    /* BLE debug bridge is currently disabled: the BT controller reserves DMA
     * that the 48-row display buffer needs, causing BLE connections to drop.
     * Re-enable when BLE can coexist (e.g. by shrinking the draw buffer). */
    /* ble_debug_init(); */
}
