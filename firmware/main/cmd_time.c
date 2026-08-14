#include "cmd_time.h"
#include "bsp_twatch_ultra.h"
#include "bsp_pcf85063.h"
#include "bsp_display.h"
#include "bsp_sdcard.h"
#include "bsp_axp2101.h"
#include "bsp_i2c.h"
#include "power.h"
#include "ui/ui_clock.h"

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int day_of_week(int y, int m, int d)
{
    /* Sakamoto: returns 0=Sunday..6=Saturday (Gregorian) */
    static const int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    y -= (m < 3);
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

static int cmd_settime(int argc, char **argv)
{
    if (argc != 3) {
        printf("usage: settime YYYY-MM-DD HH:MM:SS\n");
        return 1;
    }

    int y, m, d, hh, mm, ss;
    if (sscanf(argv[1], "%d-%d-%d", &y, &m, &d) != 3 ||
        sscanf(argv[2], "%d:%d:%d", &hh, &mm, &ss) != 3) {
        printf("bad format; usage: settime YYYY-MM-DD HH:MM:SS\n");
        return 1;
    }

    if (y < 2000 || y > 2099 || m < 1 || m > 12 || d < 1 || d > 31 ||
        hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59) {
        printf("value out of range\n");
        return 1;
    }

    struct tm t;
    memset(&t, 0, sizeof(t));
    t.tm_sec = ss;
    t.tm_min = mm;
    t.tm_hour = hh;
    t.tm_mday = d;
    t.tm_mon = m - 1;
    t.tm_year = y - 1900;
    t.tm_wday = day_of_week(y, m, d);
    t.tm_isdst = 0;

    esp_err_t err = bsp_rtc_set_time(&t);
    if (err != ESP_OK) {
        printf("settime failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("rtc set to %04d-%02d-%02d %02d:%02d:%02d\n", y, m, d, hh, mm, ss);
    return 0;
}

static int cmd_time(int argc, char **argv)
{
    struct tm t;
    if (bsp_rtc_get_time(&t) != ESP_OK) {
        printf("rtc: invalid state (oscillator stop flag set)\n");
        return 1;
    }
    static const char *wd[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    printf("rtc: %s %04d-%02d-%02d %02d:%02d:%02d\n",
           wd[(t.tm_wday + 7) % 7], t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
    return 0;
}

static int cmd_shot(int argc, char **argv)
{
    ui_shot_trigger();
    return 0;
}

static int cmd_mode(int argc, char **argv)
{
    if (argc == 2) {
        int idx = atoi(argv[1]);
        if (idx < 0 || idx > 1) {
            printf("usage: mode 0-1 (0=casio day, 1=red low-power)\n");
            return 1;
        }
        ui_mode_select_request(idx);
        return 0;
    }
    ui_mode_cycle_request();
    return 0;
}

static int cmd_brightness(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: brightness 0-100\n");
        return 1;
    }
    int pct = atoi(argv[1]);
    if (pct < 0 || pct > 100) {
        printf("value out of range\n");
        return 1;
    }
    esp_err_t err = bsp_display_set_brightness((uint8_t)pct);
    if (err != ESP_OK) {
        printf("brightness failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    power_set_user_brightness((uint8_t)pct);
    printf("brightness set to %d%%\n", pct);
    return 0;
}

static int cmd_diag(int argc, char **argv)
{
    uint32_t starts, comps, overlaps;
    bsp_display_get_stats(&starts, &comps, &overlaps);
    printf("xfer: starts=%lu completions=%lu overlaps=%lu\n",
           (unsigned long)starts, (unsigned long)comps, (unsigned long)overlaps);

    uint32_t calls, timeouts, max_us, last_us;
    ui_get_te_stats(&calls, &timeouts, &max_us, &last_us);
    printf("te:   calls=%lu timeouts=%lu max_us=%lu last_us=%lu\n",
           (unsigned long)calls, (unsigned long)timeouts,
           (unsigned long)max_us, (unsigned long)last_us);
    return 0;
}

static int cmd_sd(int argc, char **argv)
{
    uint8_t reg = 0;
    if (bsp_i2c_read_reg(AXP2101_I2C_ADDR, 0x90, &reg, 1) == ESP_OK) {
        printf("sd: axp0x90=0x%02X (ALDO1 bit0=%d)\n", reg, reg & 1);
    }
    if (bsp_i2c_read_reg(AXP2101_I2C_ADDR, 0x92, &reg, 1) == ESP_OK) {
        printf("sd: axp0x92=0x%02X (ALDO1 vol=%umV)\n", reg, 500 + (reg & 0x1F) * 100);
    }
    bsp_axp2101_set_aldo1(true);
    if (!bsp_sd_mounted()) {
        esp_err_t err = bsp_sd_init();
        if (err != ESP_OK) {
            printf("sd: mount failed at '%s': %s\n", bsp_sd_last_error(), esp_err_to_name(err));
        }
    }
    printf("sd: detect=%d mounted=%d capacity=%luMB free=%luMB\n",
           bsp_sd_detect(), bsp_sd_mounted(),
           (unsigned long)bsp_sd_capacity_mb(),
           (unsigned long)bsp_sd_free_mb());
    return 0;
}

static int cmd_cat(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: cat <path>  (e.g. /sdcard/power.log)\n");
        return 1;
    }
    if (!bsp_sd_mounted()) {
        printf("cat: SD card not mounted\n");
        return 1;
    }
    FILE *f = fopen(argv[1], "r");
    if (f == NULL) {
        printf("cat: cannot open %s\n", argv[1]);
        return 1;
    }
    char line[160];
    while (fgets(line, sizeof(line), f) != NULL) {
        fputs(line, stdout);
    }
    fclose(f);
    return 0;
}

static int cmd_sleep(int argc, char **argv)
{
    if (argc == 2) {
        if (strcmp(argv[1], "on") == 0) {
            power_set_sleep_enabled(true);
        } else if (strcmp(argv[1], "off") == 0) {
            power_set_sleep_enabled(false);
        } else {
            printf("usage: sleep on|off\n");
            return 1;
        }
    }
    uint32_t sleeps, wake_timer, wake_ext1;
    power_get_stats(&sleeps, &wake_timer, &wake_ext1);
    printf("sleep: enabled=%d sleeps=%lu wake_timer=%lu wake_ext1=%lu\n",
           power_get_sleep_enabled(),
           (unsigned long)sleeps, (unsigned long)wake_timer, (unsigned long)wake_ext1);
    return 0;
}

void cmd_time_register(void)
{
    const esp_console_cmd_t settime_cmd = {
        .command = "settime",
        .help = "set RTC time",
        .hint = "YYYY-MM-DD HH:MM:SS",
        .func = cmd_settime,
    };
    esp_console_cmd_register(&settime_cmd);

    const esp_console_cmd_t time_cmd = {
        .command = "time",
        .help = "print current RTC time",
        .func = cmd_time,
    };
    esp_console_cmd_register(&time_cmd);

    const esp_console_cmd_t shot_cmd = {
        .command = "shot",
        .help = "dump a screenshot over UART",
        .func = cmd_shot,
    };
    esp_console_cmd_register(&shot_cmd);

    const esp_console_cmd_t brightness_cmd = {
        .command = "brightness",
        .help = "set display brightness",
        .hint = "0-100",
        .func = cmd_brightness,
    };
    esp_console_cmd_register(&brightness_cmd);

    const esp_console_cmd_t mode_cmd = {
        .command = "mode",
        .help = "cycle display mode or select one (0-1)",
        .func = cmd_mode,
    };
    esp_console_cmd_register(&mode_cmd);

    const esp_console_cmd_t diag_cmd = {
        .command = "diag",
        .help = "print SPI/TE transfer diagnostics",
        .func = cmd_diag,
    };
    esp_console_cmd_register(&diag_cmd);

    const esp_console_cmd_t sd_cmd = {
        .command = "sd",
        .help = "print SD card state (powers + probes card)",
        .func = cmd_sd,
    };
    esp_console_cmd_register(&sd_cmd);

    const esp_console_cmd_t cat_cmd = {
        .command = "cat",
        .help = "print a file from the SD card",
        .hint = "<path>",
        .func = cmd_cat,
    };
    esp_console_cmd_register(&cat_cmd);

    const esp_console_cmd_t sleep_cmd = {
        .command = "sleep",
        .help = "toggle/print light-sleep engine state",
        .hint = "on|off",
        .func = cmd_sleep,
    };
    esp_console_cmd_register(&sleep_cmd);
}
