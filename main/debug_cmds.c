/*
 * debug_cmds.c - miscellaneous debug console commands (SD, power, alarm,
 * BLE, motor, RTC, crash dump).
 *
 * Moved out of uwatch_main.c's debug_process_cmd() dispatch (mechanical
 * refactor, no behavior change).
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "twatch_board.h"
#include "lvgl_app.h"
#include "sd_log.h"
#include "crash_dump.h"
#include "drv2605.h"
#include "xl9555.h"
#include "co5300.h"
#include "axp2101.h"
#include "power_mgmt.h"
#include "alarm.h"
#include "ble_debug.h"
#include "daily_log.h"
#include "bhi260ap.h"
#include "pcf85063a.h"
#include "debug_cmds.h"

void debug_cmd_shot(const char *args)
{
    (void)args;
    lvgl_app_dump_screenshot();
}

void debug_cmd_sdin(const char *args)
{
    (void)args;
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
}

void debug_cmd_sdls(const char *args)
{
    (void)args;
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
}

void debug_cmd_sdclear(const char *args)
{
    (void)args;
    /* Truncate the log and delete screenshots. */
    esp_err_t err = sd_log_clear();
    printf("sdclear: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
}

void debug_cmd_heap(const char *args)
{
    (void)args;
    printf("heap: free=%lu min=%lu dma=%lu\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)esp_get_minimum_free_heap_size(),
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
}

void debug_cmd_crashinfo(const char *args)
{
    (void)args;
    crash_dump_print_status();
}

void debug_cmd_crashsave(const char *args)
{
    (void)args;
    esp_err_t err = crash_dump_save();
    printf("crashsave: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
}

void debug_cmd_crashls(const char *args)
{
    (void)args;
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
}

void debug_cmd_crashread(const char *args)
{
    size_t name_len = strlen(args);
    if (name_len == 0 || name_len >= 64) {
        printf("crashread: invalid filename\n");
    } else {
        char name[64];
        memcpy(name, args, name_len);
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
}

void debug_cmd_panictest(const char *args)
{
    (void)args;
    /* Deliberately crash to exercise the core dump -> SD path. */
    printf("panictest: triggering a null-pointer dereference...\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    volatile uint32_t *p = (volatile uint32_t *)0;
    *p = 0xDEADBEEF;
    printf("panictest: should never reach here\n");
}

void debug_cmd_dailylog(const char *args)
{
    (void)args;
    uint32_t steps = 0;
    uint32_t lifetime = 0;
    daily_log_get_steps(&steps);
    esp_err_t lr = bhi260ap_get_step_count(&lifetime);
    const uint32_t *sec[DAILY_ACT_COUNT];
    daily_log_get_activity_seconds(sec);
    static const char *names[DAILY_ACT_COUNT] = {
        "still", "walking", "running", "cycling", "vehicle", "tilting", "unknown" };
    printf("dailylog: day_steps=%lu lifetime=%lu lt_rc=%d\n",
           (unsigned long)steps, (unsigned long)lifetime, (int)lr);
    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        printf("dailylog: %-8s %lu s\n", names[i], (unsigned long)*sec[i]);
    }
    daily_log_flush();
    printf("dailylog: sd=%d\n", sd_log_available() ? 1 : 0);
}

void debug_cmd_motor(const char *args)
{
    if (args[0] == '\0') {
        /* Verify the haptic motor: enable the DRV2605 rail and fire a
         * short vibration. Usage: "motor" or "motor 47" (waveform id).
         * Default 47 = strong click (library 1). */
        int wave = 47;
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_err_t err = drv2605_play(twatch_haptic_dev, (uint8_t)wave);
        printf("motor: wave=%d %s\n", wave, (err == ESP_OK) ? "ok" : esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(400));
        drv2605_go(twatch_haptic_dev);
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
    } else if (strcmp(args, "cal") == 0) {
        /* Run the on-chip auto-calibration and save to NVS. The motor
         * vibrates for ~1 s during calibration. */
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_err_t err = drv2605_auto_calibrate(twatch_haptic_dev);
        printf("motor cal: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
    } else if (strcmp(args, "calrestore") == 0) {
        esp_err_t err = drv2605_calibrate_restore(twatch_haptic_dev);
        printf("motor calrestore: %s\n",
               (err == ESP_OK) ? "ok" : (err == ESP_ERR_NOT_FOUND) ? "none stored" : esp_err_to_name(err));
    } else {
        printf("unknown command: motor %s\n", args);
    }
}

void debug_cmd_pm(const char *args)
{
    if (args[0] == '\0') {
        /* Dump the power-management settings. */
        printf("pm: night_auto=%d skip_usb=%d night=%d\n",
               power_mgmt_get_night_mode_auto() ? 1 : 0,
               power_mgmt_get_skip_sleep_on_usb() ? 1 : 0,
               power_mgmt_is_night_mode() ? 1 : 0);
    } else if (strncmp(args, "night ", 6) == 0) {
        power_mgmt_set_night_mode_auto(atoi(args + 6) != 0);
        printf("pm: night_auto=%d\n", power_mgmt_get_night_mode_auto() ? 1 : 0);
    } else if (strncmp(args, "usb ", 4) == 0) {
        power_mgmt_set_skip_sleep_on_usb(atoi(args + 4) != 0);
        printf("pm: skip_usb=%d\n", power_mgmt_get_skip_sleep_on_usb() ? 1 : 0);
    } else {
        printf("unknown command: pm %s\n", args);
    }
}

void debug_cmd_bat(const char *args)
{
    if (args[0] == '\0') {
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
    } else if (strcmp(args, "on") == 0) {
        esp_err_t e = axp2101_set_charge_enabled(twatch_pmu_dev, true);
        printf("bat: charge enable -> %s\n", (e == ESP_OK) ? "ok" : esp_err_to_name(e));
    } else if (strcmp(args, "off") == 0) {
        esp_err_t e = axp2101_set_charge_enabled(twatch_pmu_dev, false);
        printf("bat: charge disable -> %s\n", (e == ESP_OK) ? "ok" : esp_err_to_name(e));
    } else {
        printf("unknown command: bat %s\n", args);
    }
}

void debug_cmd_dispchk(const char *args)
{
    (void)args;
    uint8_t ldo0 = 0, aldo2_vol = 0;
    axp2101_read_reg(twatch_pmu_dev, 0x90, &ldo0);
    axp2101_read_reg(twatch_pmu_dev, 0x93, &aldo2_vol);
    uint16_t xl = 0;
    xl9555_read_port(twatch_xl9555_dev, &xl);
    printf("disp: LDO_ONOFF0=0x%02x (ALDO2=%d) ALDO2_vol=0x%02x "
           "XL9555=0x%04x (DISP_PWR=%d)\n",
           ldo0, (ldo0 >> 1) & 1, aldo2_vol, xl, (xl >> 7) & 1);
}

void debug_cmd_disppwr(const char *args)
{
    (void)args;
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
}

void debug_cmd_alarm(const char *args)
{
    if (args[0] == '\0') {
        /* Status. */
        alarm_config_t ac;
        alarm_get_config(&ac);
        printf("alarm: %s %02u:%02u mode %u ringing=%d armed=%d\n",
               ac.enabled ? "armed" : "disabled", (unsigned)ac.hour, (unsigned)ac.min,
               (unsigned)ac.ring_mode, (int)alarm_is_ringing(), (int)alarm_is_armed());
        return;
    }
    /* alarm HH:MM [beep|vib|both]  or  alarm off */
    const char *rest = args;
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
}

void debug_cmd_alarmring(const char *args)
{
    (void)args;
    /* Fire the ring now (test). */
    alarm_ring_test();
    printf("alarm: ring test\n");
}

void debug_cmd_alarmdismiss(const char *args)
{
    (void)args;
    alarm_dismiss();
    printf("alarm: dismissed\n");
}

void debug_cmd_alarmsnooze(const char *args)
{
    (void)args;
    alarm_snooze();
    printf("alarm: snoozed\n");
}

void debug_cmd_rtcdump(const char *args)
{
    (void)args;
    pcf85063a_debug_dump(twatch_rtc_dev);
}

void debug_cmd_rtctimer(const char *args)
{
    if (args[0] == '\0') {
        printf("rtctimer: usage: rtctimer <1..255 sec>\n");
        return;
    }
    /* Test the PCF85063A countdown-timer hardware directly: arm a short
     * timer and watch TF set when it expires (the snooze re-ring path).
     * Expect the ring to start when the timer fires. */
    int secs = atoi(args);
    if (secs < 1 || secs > 255) {
        printf("rtctimer: usage: rtctimer <1..255 sec>\n");
    } else {
        esp_err_t e = pcf85063a_set_timer_seconds(twatch_rtc_dev, (uint16_t)secs, true);
        printf("rtctimer: armed %d s countdown: %s\n", secs,
               e == ESP_OK ? "ok (TIE on)" : esp_err_to_name(e));
    }
}

void debug_cmd_settime(const char *args)
{
    /* Manually set the RTC. Args are UTC (the RTC's own convention, see
     * docs/adr/0003-rtc-stores-utc.md) - a fallback for when GNSS hasn't
     * synced it yet (e.g. right after this firmware's RTC-UTC migration, or
     * indoors with no fix). */
    int y, mo, d, h, mi, s;
    if (sscanf(args, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6) {
        printf("settime: usage: settime <YYYY-MM-DD> <HH:MM:SS>  (UTC)\n");
        return;
    }
    if (y < 2000 || y > 2100 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
        h > 23 || mi > 59 || s > 60) {
        printf("settime: value out of range\n");
        return;
    }
    pcf85063a_time_t t;
    t.year = (uint16_t)y;
    t.month = (uint8_t)mo;
    t.day = (uint8_t)d;
    t.hour = (uint8_t)h;
    t.min = (uint8_t)mi;
    t.sec = (uint8_t)s;
    pcf85063a_epoch_to_time(pcf85063a_time_to_epoch(&t), &t);  /* fills weekday */
    esp_err_t e = pcf85063a_set_time(twatch_rtc_dev, &t);
    printf("settime: RTC set to %04u-%02u-%02u %02u:%02u:%02u UTC: %s\n",
           t.year, t.month, t.day, t.hour, t.min, t.sec,
           e == ESP_OK ? "ok" : esp_err_to_name(e));
    twatch_board_sync_system_time();
}

void debug_cmd_ble(const char *args)
{
    (void)args;
    ble_debug_print_status();
}

void debug_cmd_bleadv(const char *args)
{
    (void)args;
    ble_debug_set_advertising(true);
    printf("bleadv: advertising enabled\n");
}

void debug_cmd_bleadvoff(const char *args)
{
    (void)args;
    ble_debug_set_advertising(false);
    printf("bleadvoff: advertising disabled\n");
}
