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
#include "cd_timer.h"
#include "gpx_log.h"
#include "mesh_log.h"
#include "esp_timer.h"
#include "ble_debug.h"
#include "daily_log.h"
#include "bhi260ap.h"
#include "pcf85063a.h"
#include "st25r3916.h"
#include "spi2_power.h"
#include "ndef.h"
#include "debug_cmds.h"

void debug_cmd_shot(const char *args)
{
    (void)args;
    lvgl_app_dump_screenshot();
}

void debug_cmd_sdls(const char *args)
{
    (void)args;
    /* List PNG screenshots on the SD card. */
    if (sd_log_session_begin() != ESP_OK) {
        printf("sdls: no SD card\n");
        sd_log_session_end();
        return;
    }
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
    sd_log_session_end();
}

void debug_cmd_sdclear(const char *args)
{
    (void)args;
    /* Delete screenshots. */
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
    if (sd_log_session_begin() != ESP_OK) {
        printf("crashls: no SD card\n");
        sd_log_session_end();
        return;
    }
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
    sd_log_session_end();
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
        if (sd_log_session_begin() != ESP_OK) {
            printf("crashread: no SD card\n");
            sd_log_session_end();
            return;
        }
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
        sd_log_session_end();
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

/* Ultra-Sparmodus (docs/application.md section 10), Phase 6 stage 2: only
 * a manual, debug-triggered way in exists so far - `sparmodus on` calls
 * power_mgmt_sparmodus_enter_sleep() directly (peripherals off, wake
 * sources armed, real esp_deep_sleep_start() - does not return). Stage 3
 * wires this into the idle-timeout/auto-entry-threshold path instead. */
void debug_cmd_sparmodus(const char *args)
{
    if (strcmp(args, "on") == 0) {
        printf("sparmodus: entering deep sleep now\n");
        fflush(stdout);
        power_mgmt_sparmodus_enter_sleep();   /* does not return */
    } else if (strcmp(args, "set on") == 0 || strcmp(args, "set off") == 0) {
        /* Sets the persisted flag only - no sleep. Lets the watch-face
         * red/minimal rendering (gated on power_mgmt_get_sparmodus_active())
         * be exercised live without waiting for an idle timeout or a real
         * low-battery/USB condition. */
        bool on = strcmp(args, "set on") == 0;
        power_mgmt_set_sparmodus_active(on);
        printf("sparmodus: persisted flag set to %d\n", on ? 1 : 0);
    } else {
        printf("sparmodus: usage: sparmodus on | sparmodus set on|off (persisted flag=%d)\n",
               power_mgmt_get_sparmodus_active() ? 1 : 0);
    }
}

void debug_cmd_pwroff(const char *args)
{
    (void)args;
    /* Same path a real PWRKEY long-press (>4s) takes: blank the display,
     * unmount the SD card, then hand off to the PMIC. Test hook - there's no
     * way to simulate a 4s physical button hold otherwise. */
    printf("pwroff: shutting down\n");
    power_mgmt_shutdown();
}

void debug_cmd_rails(const char *args)
{
    (void)args;
    /* Named per axp2101_set_default_power()'s power tree (LilyGoWatchUltra::
     * initPMU) - which peripheral each rail actually feeds on this board. */
    static const struct { axp2101_rail_t rail; const char *name; const char *what; } rails[] = {
        { AXP2101_ALDO1, "ALDO1", "SD card" },
        { AXP2101_ALDO2, "ALDO2", "display" },
        { AXP2101_ALDO3, "ALDO3", "LoRa (SX1262)" },
        { AXP2101_ALDO4, "ALDO4", "sensor (BHI260AP)" },
        { AXP2101_BLDO1, "BLDO1", "GNSS (M10Q)" },
        { AXP2101_BLDO2, "BLDO2", "speaker" },
        { AXP2101_DLDO1, "DLDO1", "NFC" },
    };
    for (size_t i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
        bool on = false;
        esp_err_t err = axp2101_is_rail_enabled(twatch_pmu_dev, rails[i].rail, &on);
        if (err != ESP_OK) {
            printf("%-6s %-20s read failed: %s\n", rails[i].name, rails[i].what, esp_err_to_name(err));
        } else {
            printf("%-6s %-20s %s\n", rails[i].name, rails[i].what, on ? "on" : "off");
        }
    }

    bool vbus = false;
    axp2101_is_vbus_present(twatch_pmu_dev, &vbus);
    printf("VBUS (USB power)     %s\n", vbus ? "present" : "absent");
}

void debug_cmd_nfcpoll(const char *args)
{
    /* A single REQA "pings" once, at t=0 of one bring-up - a tag placed a
     * moment later never gets a chance if that's the whole poll. Open the
     * chip ONCE (real bring-up cost, one rail power-on) and retry the cheap
     * REQA/cascade exchange repeatedly within that session instead of
     * power-cycling per attempt - matches LilyGo's own reference firmware,
     * which enables the NFC rail once at boot and never cycles it (see
     * st25r3916_open()'s doc comment). Usage: "nfcpoll" (8s) or
     * "nfcpoll <seconds>". */
    int secs = 8;
    if (args[0] != '\0') {
        secs = atoi(args);
    }
    if (secs <= 0) {
        secs = 8;
    }
    printf("nfcpoll: polling for an ISO14443-A tag (%ds)...\n", secs);
    esp_err_t err = st25r3916_open();
    if (err != ESP_OK) {
        st25r3916_close();
        printf("nfcpoll: %s\n", esp_err_to_name(err));
        return;
    }
    st25r3916_tag_t tag;
    err = ESP_ERR_NOT_FOUND;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(secs * 1000);
    while (xTaskGetTickCount() < deadline) {
        err = st25r3916_try(&tag, 300);
        if (err != ESP_ERR_NOT_FOUND) {
            break;
        }
    }

    /* Attempt an NDEF read while the tag is still selected - i.e. before
     * st25r3916_close() - only when st25r3916_try() actually found one.
     * "not NDEF-formatted" (ESP_ERR_NOT_FOUND from st25r3916_read_type2())
     * is a normal outcome for a blank/non-Type-2 tag, not an error to
     * report as one. */
    uint8_t ndef_buf[256];
    size_t ndef_len = 0;
    ndef_record_t records[4];
    size_t record_count = 0;
    bool have_ndef = false;
    if (err == ESP_OK) {
        if (st25r3916_read_type2(ndef_buf, sizeof(ndef_buf), &ndef_len, 500) == ESP_OK) {
            record_count = ndef_parse(ndef_buf, ndef_len, records, 4);
            have_ndef = true;
        }
    }

    st25r3916_close();
    if (err == ESP_ERR_NOT_FOUND) {
        printf("nfcpoll: no tag\n");
    } else if (err != ESP_OK) {
        printf("nfcpoll: %s\n", esp_err_to_name(err));
    } else {
        printf("nfcpoll: UID ");
        for (uint8_t i = 0; i < tag.uid_len; i++) {
            printf("%02X ", tag.uid[i]);
        }
        printf("(%u bytes)\n", tag.uid_len);

        if (!have_ndef) {
            printf("nfcpoll: not NDEF-formatted (no Type 2 Capability Container)\n");
        } else if (record_count == 0) {
            printf("nfcpoll: NDEF-formatted, but no NDEF message found\n");
        } else {
            for (size_t i = 0; i < record_count; i++) {
                const char *kind = (records[i].kind == NDEF_TEXT) ? "TEXT" :
                                    (records[i].kind == NDEF_URI)  ? "URI"  : "OTHER";
                printf("nfcpoll: NDEF[%u] %s (%s): %s\n", (unsigned)i, kind,
                       records[i].type, records[i].text);
            }
        }
    }
}

void debug_cmd_nfcprobe(const char *args)
{
    /* Sweep the three rails that can plausibly keep the ST25R3916 off the
     * bus, and read its identity register through the driver's own
     * permanent SPI device (st25r3916_probe_identity()) at each combination.
     *
     * The SD card axis is what this was originally built to test: it shares
     * SPI2's MOSI/MISO/SCK, and sd_log_unmount() cuts its rail (ALDO1) while
     * leaving its DAT0 pin tied to the shared MISO net. An unpowered device
     * clamps the net through its ESD protection diodes toward its own 0V
     * rail, which reads back as 0x00 - indistinguishable from "the NFC chip
     * isn't answering". Every earlier probe here ran with the SD card
     * unpowered, so that confound had to be removed before believing any
     * of it (see docs/nfc.md).
     *
     * The LoRa axis (ALDO3) asked the same question for the SX1262: its
     * module has a single VCC pin for the whole package with no separate
     * always-on I/O rail (unlike the ST25R3916, whose host interface runs
     * off the always-on DC3V3 rail independent of DLDO1), so it was assumed
     * to clamp the bus the same way the SD card does once its rail is cut -
     * spi2_power.c's design assumes this worst case defensively. Measured on
     * hardware: it doesn't hold. With ALDO1 on, ic_identity reads correctly
     * regardless of ALDO3's state. See docs/nfc.md for the full sweep this
     * command produced.
     *
     * CAUTION: toggling ALDO3 nominally tears down the SX1262's live RF
     * configuration (TCXO settle, frequency, modulation - none of it is
     * expected to survive a power cycle). Measured on hardware: with this
     * probe's brief per-state toggling (default 150ms settle), mesh_log
     * resumed decoding packets immediately afterward with no reboot needed
     * - the rail was very likely never off long enough to fully discharge
     * the module's supply. That is not a guarantee for every toggle
     * pattern; a real, sustained rail-off (as LoRa's own future on-demand
     * power-down would do) may still require re-running
     * sx1262_configure_lora(). Don't take one clean run here as proof the
     * chip retains its config indefinitely.
     *
     * All three axes bypass spi2_power's own policy on purpose - the whole
     * point is to drive each rail directly and see what the bus does,
     * which spi2_power's SHARED-rail auto-raise would otherwise fight.
     *
     * No SPI mode/clock sweep any more (an earlier version of this command
     * had one): it required a temporary spi_device_handle_t sharing GPIO4
     * with the driver's permanent one, which ESP-IDF warned about
     * ("GPIO 4 is conflict with others and be overwritten") and which is the
     * leading suspect for the permanent device going unresponsive
     * (ic_identity stuck at 0xFF, cleared only by a full reboot) after this
     * command had run earlier in the same boot. See st25r3916.h's doc
     * comment on st25r3916_probe_identity(). That question is already
     * answered anyway (docs/nfc.md's SPI timing section) and wasn't worth
     * re-asking at this risk.
     *
     * "nfcprobe [settle_ms]" - rail settle time, default 150ms. */
    int settle = (args[0] != '\0') ? atoi(args) : 150;
    if (settle < 20) {
        settle = 20;
    }

    /* Unmount cleanly first if the card is up: cutting the rail from under a
     * mounted card is what leaves it unable to remount (sd_log.c). */
    bool was_mounted = sd_log_available();
    if (was_mounted) {
        sd_log_unmount();
    }

    printf("nfcprobe: sweeping SD rail x LoRa rail x NFC rail (settle %dms)\n", settle);
    printf("nfcprobe: CAUTION - this briefly cycles LoRa's rail; a real sustained\n");
    printf("nfcprobe:   power-down (unlike this quick sweep) may still need a reboot to resume RX\n");

    for (int sd_on = 0; sd_on <= 1; sd_on++) {
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, sd_on != 0);
        for (int lora_on = 0; lora_on <= 1; lora_on++) {
            axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO3, lora_on != 0);
            for (int nfc_on = 0; nfc_on <= 1; nfc_on++) {
                axp2101_enable_rail(twatch_pmu_dev, AXP2101_DLDO1, nfc_on != 0);
                vTaskDelay(pdMS_TO_TICKS(settle));

                uint8_t id[2] = { 0 };
                bool ok = st25r3916_probe_identity(id);
                printf("  SD=%-3s LORA=%-3s NFC=%-3s: ic_identity=%02x %02x %s\n",
                       sd_on ? "on" : "off", lora_on ? "on" : "off", nfc_on ? "on" : "off",
                       id[0], id[1], ok ? "<== OK" : "");
            }
        }
    }

    /* Restore: NFC and LoRa rails on (normal operation - LoRa stays
     * permanently on, unchanged in this pass), SD reconciled to its actual
     * steady-state policy (card seated or not) rather than forced off,
     * which is what spi2_power now owns. */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_DLDO1, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO3, true);
    spi2_power_hold(AXP2101_RAIL_MAX);
    spi2_power_release(AXP2101_RAIL_MAX);
    if (was_mounted) {
        sd_log_mount();
    }
    printf("nfcprobe: done\n");
}

void debug_cmd_i2cscan(const char *args)
{
    /* Full 7-bit I2C scan. Chasing the ST25R3916: it selects SPI or I2C from
     * its I2C_EN pin, and in I2C mode it answers at 0x50 - so if SPI is
     * silent, this says whether the chip is alive on the other interface or
     * simply not there. */
    (void)args;
    printf("i2cscan:");
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(twatch_i2c_bus, a, 50) == ESP_OK) {
            printf(" 0x%02x", a);
            found++;
        }
    }
    printf("\ni2cscan: %d device(s)\n", found);
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

/* Prints wmask as e.g. "Daily", "Mon-Fri", or a comma list of 3-letter
 * abbreviations - mirrors the summary the Alarms/Timers list screen shows
 * per row (main/lvgl_app.c). */
static void print_weekday_mask(uint8_t wmask)
{
    static const char *names[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    if (wmask == ALARM_WEEKDAY_ALL) {
        printf("Daily");
        return;
    }
    if (wmask == 0x3E) {   /* Mon..Fri */
        printf("Mon-Fri");
        return;
    }
    bool first = true;
    for (int i = 0; i < 7; i++) {
        if (wmask & (1u << i)) {
            printf("%s%s", first ? "" : ",", names[i]);
            first = false;
        }
    }
}

void debug_cmd_alarm(const char *args)
{
    (void)args;
    printf("alarm: ringing=%d armed=%d\n", (int)alarm_is_ringing(), (int)alarm_is_armed());
    printf("alarm: usage alarmls | alarmadd <hh> <mm> [beep|vib|both] [wmask=0xNN] | alarmrm <idx> | alarmen <idx> <0|1>\n");
}

void debug_cmd_alarmls(const char *args)
{
    (void)args;
    alarm_entry_t list[ALARM_MAX_COUNT];
    size_t n = alarm_get_all(list, ALARM_MAX_COUNT);
    int shown = 0;
    for (size_t i = 0; i < n; i++) {
        if (!list[i].in_use) {
            continue;
        }
        printf("[%u] %02u:%02u mode=%u %s ", (unsigned)i, (unsigned)list[i].hour,
               (unsigned)list[i].min, (unsigned)list[i].ring_mode,
               list[i].enabled ? "enabled " : "disabled");
        print_weekday_mask(list[i].weekday_mask);
        printf("\n");
        shown++;
    }
    if (shown == 0) {
        printf("alarmls: no alarms configured\n");
    }
}

void debug_cmd_alarmadd(const char *args)
{
    /* alarmadd <hh> <mm> [beep|vib|both] [0xNN weekday mask] */
    int hh = -1, mm = -1;
    uint8_t mode = ALARM_RING_BEEP;
    uint8_t wmask = ALARM_WEEKDAY_ALL;
    char mode_str[8] = { 0 };
    int n = sscanf(args, "%d %d %7s %hhx", &hh, &mm, mode_str, &wmask);
    if (n < 2 || hh < 0 || hh > 23 || mm < 0 || mm > 59) {
        printf("alarmadd: usage alarmadd <hh> <mm> [beep|vib|both] [wmask hex]\n");
        return;
    }
    if (n >= 3) {
        if (strcmp(mode_str, "vib") == 0) mode = ALARM_RING_VIB;
        else if (strcmp(mode_str, "both") == 0) mode = ALARM_RING_BOTH;
    }
    int idx = alarm_add((uint8_t)hh, (uint8_t)mm, mode, wmask);
    if (idx < 0) {
        printf("alarmadd: failed (list full?)\n");
    } else {
        printf("alarmadd: [%d] %02d:%02d mode=%u wmask=0x%02x\n", idx, hh, mm,
               (unsigned)mode, (unsigned)wmask);
    }
}

void debug_cmd_alarmrm(const char *args)
{
    int idx = atoi(args);
    esp_err_t e = alarm_remove(idx);
    printf("alarmrm: [%d] %s\n", idx, e == ESP_OK ? "removed" : esp_err_to_name(e));
}

void debug_cmd_alarmen(const char *args)
{
    int idx = -1, en = -1;
    if (sscanf(args, "%d %d", &idx, &en) != 2) {
        printf("alarmen: usage alarmen <idx> <0|1>\n");
        return;
    }
    esp_err_t e = alarm_set_enabled(idx, en != 0);
    printf("alarmen: [%d] %s: %s\n", idx, en ? "enabled" : "disabled",
           e == ESP_OK ? "ok" : esp_err_to_name(e));
}

void debug_cmd_timer(const char *args)
{
    if (args[0] == '\0') {
        if (cdtimer_is_active()) {
            printf("timer: active, %lu s remaining\n", (unsigned long)cdtimer_remaining_seconds());
        } else {
            printf("timer: inactive\n");
        }
        printf("timer: usage timer <seconds> | timer off\n");
        return;
    }
    if (strcmp(args, "off") == 0) {
        cdtimer_cancel();
        printf("timer: cancelled\n");
        return;
    }
    int secs = atoi(args);
    if (secs <= 0) {
        printf("timer: usage timer <seconds> | timer off\n");
        return;
    }
    esp_err_t e = cdtimer_start((uint32_t)secs);
    printf("timer: %s\n", e == ESP_OK ? "started" : esp_err_to_name(e));
}

void debug_cmd_gpxstatus(const char *args)
{
    if (strcmp(args, "on") == 0) {
        esp_err_t e = gpx_log_start();
        printf("gpxstatus: start: %s\n", e == ESP_OK ? "ok" : esp_err_to_name(e));
        return;
    }
    if (strcmp(args, "off") == 0) {
        esp_err_t e = gpx_log_stop();
        printf("gpxstatus: stop: %s\n", e == ESP_OK ? "ok" : esp_err_to_name(e));
        return;
    }
    printf("gpxstatus: active=%d points=%lu path=%s\n", (int)gpx_log_is_active(),
           (unsigned long)gpx_log_point_count(), gpx_log_current_path());
    printf("gpxstatus: usage gpxstatus [on|off]\n");
}

void debug_cmd_gpxcat(const char *args)
{
    (void)args;
    const char *path = gpx_log_current_path();
    if (path[0] == '\0') {
        printf("gpxcat: no session yet this boot\n");
        return;
    }
    FILE *f = fopen(path, "r");
    if (!f) {
        printf("gpxcat: fopen %s failed\n", path);
        return;
    }
    char line[160];
    while (fgets(line, sizeof(line), f)) {
        fputs(line, stdout);
    }
    fclose(f);
}

void debug_cmd_meshnodes(const char *args)
{
    (void)args;
    mesh_node_t nodes[MESH_NODE_TABLE_MAX];
    size_t n = mesh_log_get_nodes(nodes, MESH_NODE_TABLE_MAX);
    if (n == 0) {
        printf("meshnodes: no nodes seen yet\n");
        return;
    }
    int64_t now_us = esp_timer_get_time();
    for (size_t i = 0; i < n; i++) {
        uint32_t age_s = (uint32_t)((now_us - nodes[i].last_seen_us) / 1000000);
        printf("!%08lx  %-20s  %lus ago  %ddBm %+ddB\n", (unsigned long)nodes[i].node_id,
               nodes[i].name[0] ? nodes[i].name : "(unknown)", (unsigned long)age_s,
               (int)nodes[i].last_rssi_dbm, (int)nodes[i].last_snr_db);
    }
}

void debug_cmd_meshcat(const char *args)
{
    (void)args;
    FILE *f = fopen("/sdcard/log/mesh.txt", "r");
    if (!f) {
        printf("meshcat: no log yet (no SD, or no text messages received)\n");
        return;
    }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        fputs(line, stdout);
    }
    fclose(f);
}

void debug_cmd_presetset(const char *args)
{
    int idx = -1;
    int consumed = 0;
    if (sscanf(args, "%d %n", &idx, &consumed) < 1 || idx < 0 || idx >= MESH_PRESET_COUNT) {
        printf("presetset: usage presetset <0-%d> <text>\n", MESH_PRESET_COUNT - 1);
        for (int i = 0; i < MESH_PRESET_COUNT; i++) {
            char cur[MESH_PRESET_MAX_LEN + 1];
            mesh_preset_get(i, cur, sizeof(cur));
            printf("  [%d] %s\n", i, cur);
        }
        return;
    }
    const char *text = args + consumed;
    if (text[0] == '\0') {
        printf("presetset: usage presetset <0-%d> <text>\n", MESH_PRESET_COUNT - 1);
        return;
    }
    mesh_preset_set(idx, text);
    printf("presetset: [%d] = \"%s\"\n", idx, text);
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
