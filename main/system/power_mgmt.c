/*
 * system/power_mgmt.c - watch power management.
 *
 * DFS + automatic light sleep via esp_pm, integrated with the LVGL adapter's
 * auto-sleep (PAUSE mode). Wake sources: touch (GPIO12, handled by the touch
 * driver), power button (AXP2101 IRQ on GPIO7) and boot button (GPIO0).
 *
 * On sleep: panel SLPIN, unused peripheral rails off, GPIO wakeups armed.
 * On wake: rails + panel restored.
 */
#include "power_mgmt.h"
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "esp_timer.h"
#include "display_controller.h"
#include "touch_controller.h"
#include "pcf85063a.h"
#include "bhi260ap.h"
#include "m10q.h"
#include "sensor_cache.h"
#include "lvgl_app.h"
#include "screens/screens.h"
#include "alarm.h"
#include "sd_log.h"
#include "mesh_log.h"

static const char *TAG = "power_mgmt";

#define PM_GPIO_PWRKEY 7    /* AXP2101 IRQ */
#define PM_GPIO_BOOT   0
#define PM_GPIO_IMU    8    /* BHI260AP INT (wake on wrist-raise/gesture) */
#define PM_GPIO_RTC    1    /* PCF85063A INT (alarm / snooze timer) */

/* Wake source bits for s_wake_sources, one per ISR-attributed GPIO above.
 * Touch has no bit: it has no source-specific post-wake behavior. */
#define PM_WAKE_PWRKEY (1u << PM_GPIO_PWRKEY)
#define PM_WAKE_BOOT   (1u << PM_GPIO_BOOT)
#define PM_WAKE_IMU    (1u << PM_GPIO_IMU)
#define PM_WAKE_RTC    (1u << PM_GPIO_RTC)

/* Periodic light-sleep wake for housekeeping (main/housekeeping.c: syslog
 * service, gpx_log_tick(), daily_log_tick() - the per-minute steps/activity/
 * battery CSV rows). That task is an ordinary vTaskDelay/notify loop, so it is
 * frozen for the whole of a light-sleep period: without a timer wake it only
 * ever runs when something else happens to wake the watch. That used to be
 * constant (the watch barely slept), so the minute cadence held by accident;
 * now that sleep periods are long it needs its own wake source, and it will
 * matter more once the BHI's spurious wakes are silenced at the source.
 * Matches HOUSEKEEPING_DAILY_INTERVAL_MS so a sample is ready each time. */
#define PM_HOUSEKEEPING_WAKE_US (60ULL * 1000000ULL)

/* Wake sources as seen by power_mgmt_exit_sleep(), latched separately from
 * s_wake_sources. The two run concurrently and pm_wake_task() clears
 * s_wake_sources under a critical section as soon as it runs, so exit_sleep
 * reading that same variable was a race: when the task won, exit_sleep saw 0,
 * failed to recognise an IMU-only wake and lit the display for what was just
 * BHI scheduler chatter. Observed live (2026-09-01):
 *   wake: sources=0x100  ->  waking: sources=0x0 causes=0x1  ->  panel woken.
 * The ISR sets both; each consumer clears its own copy. */
static volatile uint32_t s_exit_wake_sources;
/* When the current light-sleep period began, used to recognise the
 * housekeeping heartbeat by its duration. */
static int64_t s_sleep_enter_us;
static portMUX_TYPE s_exit_wake_lock = portMUX_INITIALIZER_UNLOCKED;

/* AXP2101 combined 24-bit IRQ status (see axp2101_get_irq_status()):
 * INTSTS2 occupies bits 8-15, so its bit 2 ("long press", per
 * axp2101_configure_pwrkey_shutdown()'s OFFLEVEL threshold) is bit 10 here. */
#define AXP_IRQ_PEK_LONG (1u << 10)
/* INTSTS2 bit 7 ("VBUS insert", datasheet REG49) -> bit 15 here. Only
 * fires once axp2101_enable_vbus_irq() has unmasked it - see Phase 6's
 * Ultra-Sparmodus deep-sleep wake handling. */
#define AXP_IRQ_VBUS_INSERT (1u << 15)

/* Every other named bit in the combined 24-bit IRQ status, from
 * Datasheets/AXP2101_datasheet.pdf REG40/41/42 ("IRQ Enable 0/1/2" - the
 * status registers REG48/49/4A share the same bit layout, INTSTS0 at
 * bits 0-7, INTSTS1 at 8-15, INTSTS2 at 16-23). GPIO7 (PM_GPIO_PWRKEY)
 * is the AXP's single shared IRQ output pin - ANY of these, not just a
 * power-key press, can wake the ESP32 through it. Added after a live
 * "white screen after wake" report turned out to correlate with
 * chgdn_irq (battery charge done), not the power key at all - the old
 * code only ever decoded PEK bits, so every other cause looked
 * unexplained. Decoding all of them now so the next occurrence's real
 * trigger is a log line, not a guess. */
#define AXP_IRQ_BWUT       (1u << 0)    /* battery under-temp, work mode */
#define AXP_IRQ_BWOT       (1u << 1)    /* battery over-temp, work mode */
#define AXP_IRQ_BCUT       (1u << 2)    /* battery under-temp, charge mode */
#define AXP_IRQ_BCOT       (1u << 3)    /* battery over-temp, charge mode */
#define AXP_IRQ_LOWSOC     (1u << 4)    /* gauge: new SOC reading */
#define AXP_IRQ_GWDT       (1u << 5)    /* gauge watchdog timeout */
#define AXP_IRQ_SOCWL1     (1u << 6)    /* SOC dropped to warning level 1 */
#define AXP_IRQ_SOCWL2     (1u << 7)    /* SOC dropped to warning level 2 */
#define AXP_IRQ_PEK_POS    (1u << 8)    /* power key positive edge (press) */
#define AXP_IRQ_PEK_NEG    (1u << 9)    /* power key negative edge (release) */
#define AXP_IRQ_PEK_SHORT  (1u << 11)   /* power key short press */
#define AXP_IRQ_BAT_REMOVE (1u << 12)
#define AXP_IRQ_BAT_INSERT (1u << 13)
#define AXP_IRQ_VBUS_REMOVE (1u << 14)
#define AXP_IRQ_BOVP       (1u << 16)   /* battery over-voltage protection */
#define AXP_IRQ_CHGTE      (1u << 17)   /* charger safety timer expired */
#define AXP_IRQ_DOTL1      (1u << 18)   /* die over-temp level 1 */
#define AXP_IRQ_CHGST      (1u << 19)   /* charge started */
#define AXP_IRQ_CHGDN      (1u << 20)   /* charge done */
#define AXP_IRQ_LDOOC       (1u << 22)  /* LDO over-current */
#define AXP_IRQ_WDEXP      (1u << 23)   /* watchdog expired */

/* Renders every set bit's name into `out` (space-separated), or "none" if
 * irq is 0. Only for the log line above - not performance-sensitive. */
static void axp_irq_describe(uint32_t irq, char *out, size_t out_len)
{
    static const struct { uint32_t bit; const char *name; } bits[] = {
        { AXP_IRQ_BWUT, "bwut" }, { AXP_IRQ_BWOT, "bwot" },
        { AXP_IRQ_BCUT, "bcut" }, { AXP_IRQ_BCOT, "bcot" },
        { AXP_IRQ_LOWSOC, "lowsoc" }, { AXP_IRQ_GWDT, "gwdt" },
        { AXP_IRQ_SOCWL1, "socwl1" }, { AXP_IRQ_SOCWL2, "socwl2" },
        { AXP_IRQ_PEK_POS, "pek_press" }, { AXP_IRQ_PEK_NEG, "pek_release" },
        { AXP_IRQ_PEK_LONG, "pek_long" }, { AXP_IRQ_PEK_SHORT, "pek_short" },
        { AXP_IRQ_BAT_REMOVE, "bat_remove" }, { AXP_IRQ_BAT_INSERT, "bat_insert" },
        { AXP_IRQ_VBUS_REMOVE, "vbus_remove" }, { AXP_IRQ_VBUS_INSERT, "vbus_insert" },
        { AXP_IRQ_BOVP, "bovp" }, { AXP_IRQ_CHGTE, "chgte" },
        { AXP_IRQ_DOTL1, "dotl1" }, { AXP_IRQ_CHGST, "chgst" },
        { AXP_IRQ_CHGDN, "chgdn" }, { AXP_IRQ_LDOOC, "ldooc" },
        { AXP_IRQ_WDEXP, "wdexp" },
    };
    out[0] = '\0';
    if (irq == 0) {
        snprintf(out, out_len, "none");
        return;
    }
    for (size_t i = 0; i < sizeof(bits) / sizeof(bits[0]); i++) {
        if (irq & bits[i].bit) {
            if (out[0] != '\0') {
                strlcat(out, " ", out_len);
            }
            strlcat(out, bits[i].name, out_len);
        }
    }
}

/* Night-mode clock check period while the watch is idle. */
#define PM_NIGHT_CHECK_MS  60000

/* NVS-persisted settings: night-mode auto and "do not sleep while on USB". */
#define PM_NVS_NS         "pm"
#define PM_NVS_KEY_NIGHT  "night_auto"
#define PM_NVS_KEY_USB    "skip_usb"
#define PM_NVS_KEY_DISP_TO "disp_to"
#define PM_NVS_KEY_BRIGHT  "bright"
#define PM_NVS_KEY_SPARMODUS "sparmodus"

static bool s_night_mode_auto = true;    /* default: auto-enter night mode */
static bool s_skip_sleep_on_usb = true;  /* default: never sleep on USB */
static uint32_t s_display_timeout_s = 5; /* default: matches the historical 5000 ms literal */
static uint8_t s_brightness = 0x80;      /* default: matches the historical 0x80 literal */
static bool s_sparmodus_active = false;  /* Ultra-Sparmodus, docs/application.md section 10 */

/* Survives a deep-sleep wake (unlike every other static above, which is
 * re-initialized from scratch since deep sleep wipes normal DRAM/task
 * state - a deep-sleep wake IS a fresh boot). This is what app_main()
 * actually branches on via power_mgmt_sparmodus_should_resleep_silently();
 * s_sparmodus_active above is the separate NVS-persisted "user wants this
 * mode" preference, only read at boot/from Settings. */
RTC_DATA_ATTR static bool s_rtc_sparmodus_active;

/* RTC-capable GPIO wakeup for the touch line is armed here. */
static volatile uint32_t s_wake_sources;   /* bitmask of PM_WAKE_*, ISR-writer/task-reader */
static portMUX_TYPE s_wake_sources_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_wake_task;
static volatile bool s_night_mode;
static volatile bool s_imu_wake_armed;   /* GPIO8 ISR only notifies while asleep */
static power_mgmt_night_mode_cb_t s_night_mode_cb;
static power_mgmt_button_cb_t s_button_cb;

/* Night mode is active between PM_NIGHT_START_HOUR (inclusive) and
 * PM_NIGHT_END_HOUR (exclusive), wrapping midnight. */
static bool pm_is_night_time(void);
static void pm_apply_night_mode(bool night);

static bool pm_is_night_time(void)
{
    /* "Night mode auto" off means night mode never activates (stays off
     * regardless of the RTC time). */
    if (!s_night_mode_auto) {
        return false;
    }
    /* Use the RTC wall clock, not the ESP32 system clock, so night mode never
     * drifts. The RTC is polled by the background telemetry cache task. The
     * RTC itself stores UTC, so convert to local before comparing against
     * the (local) night-mode hour window. */
    pcf85063a_time_t t;
    if (!sensor_cache_get_rtc(&t)) {
        return false;
    }
    time_t epoch = pcf85063a_time_to_epoch(&t);
    struct tm lt;
    localtime_r(&epoch, &lt);
    int h = lt.tm_hour;
    if (PM_NIGHT_START_HOUR <= PM_NIGHT_END_HOUR) {
        return h >= PM_NIGHT_START_HOUR && h < PM_NIGHT_END_HOUR;
    }
    return h >= PM_NIGHT_START_HOUR || h < PM_NIGHT_END_HOUR;
}

bool power_mgmt_is_night_mode(void)
{
    return s_night_mode;
}

void power_mgmt_recheck_night_mode(void)
{
    pm_apply_night_mode(pm_is_night_time());
    /* Touch input stays enabled at night (navigation still works while the
     * watch is awake); only the light-sleep WAKE source is gated, in
     * pm_arm_gpio_wakeup(). */
}

void power_mgmt_register_night_mode_cb(power_mgmt_night_mode_cb_t cb)
{
    s_night_mode_cb = cb;
}

void power_mgmt_register_button_cb(power_mgmt_button_cb_t cb)
{
    s_button_cb = cb;
}

/* ---- Persisted settings (night-mode auto, sleep-on-USB) ---- */

static void pm_config_save(void)
{
    nvs_handle_t h;
    if (nvs_open(PM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, PM_NVS_KEY_NIGHT, s_night_mode_auto ? 1 : 0);
        nvs_set_u8(h, PM_NVS_KEY_USB, s_skip_sleep_on_usb ? 1 : 0);
        nvs_set_u32(h, PM_NVS_KEY_DISP_TO, s_display_timeout_s);
        nvs_set_u8(h, PM_NVS_KEY_BRIGHT, s_brightness);
        nvs_set_u8(h, PM_NVS_KEY_SPARMODUS, s_sparmodus_active ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void pm_config_load(void)
{
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open(PM_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, PM_NVS_KEY_NIGHT, &v) == ESP_OK) {
            s_night_mode_auto = (v != 0);
        }
        v = 0;
        if (nvs_get_u8(h, PM_NVS_KEY_USB, &v) == ESP_OK) {
            s_skip_sleep_on_usb = (v != 0);
        }
        uint32_t to = 0;
        if (nvs_get_u32(h, PM_NVS_KEY_DISP_TO, &to) == ESP_OK && to > 0) {
            s_display_timeout_s = to;
        }
        v = 0;
        if (nvs_get_u8(h, PM_NVS_KEY_BRIGHT, &v) == ESP_OK && v > 0) {
            s_brightness = v;
        }
        v = 0;
        if (nvs_get_u8(h, PM_NVS_KEY_SPARMODUS, &v) == ESP_OK) {
            s_sparmodus_active = (v != 0);
        }
        nvs_close(h);
    }
}

uint32_t power_mgmt_get_display_timeout_s(void)
{
    return s_display_timeout_s;
}

void power_mgmt_set_display_timeout_s(uint32_t seconds)
{
    if (seconds == 0 || seconds == s_display_timeout_s) {
        return;
    }
    s_display_timeout_s = seconds;
    pm_config_save();
}

uint8_t power_mgmt_get_brightness(void)
{
    return s_brightness;
}

void power_mgmt_set_brightness(uint8_t level)
{
    if (level == 0 || level == s_brightness) {
        return;
    }
    s_brightness = level;
    pm_config_save();
    if (!s_night_mode) {
        display_controller_set_brightness(s_brightness);
    }
}

bool power_mgmt_get_sparmodus_active(void)
{
    return s_sparmodus_active;
}

/* Persists the NVS preference only - does NOT itself trigger deep sleep.
 * Stage 3 of Phase 6 wires this into the idle-timeout path (auto-entry);
 * for now the Settings toggle just remembers the choice, and
 * power_mgmt_sparmodus_enter_sleep() below is reachable only via the
 * `sparmodus on` debug command while that wiring is being built/verified. */
void power_mgmt_set_sparmodus_active(bool on)
{
    if (on == s_sparmodus_active) {
        return;
    }
    s_sparmodus_active = on;
    pm_config_save();
}

bool power_mgmt_sparmodus_should_resleep_silently(void)
{
    /* esp_sleep_get_wakeup_cause() (singular) is deprecated in this
     * ESP-IDF version - use the bitmap form instead. */
    uint32_t causes = esp_sleep_get_wakeup_causes();
    return s_rtc_sparmodus_active && causes == (1u << ESP_SLEEP_WAKEUP_TIMER);
}

bool power_mgmt_sparmodus_was_deep_sleep_wake(void)
{
    /* Only meaningful once should_resleep_silently() has already been
     * checked and returned false (a pure timer wake never reaches this) -
     * at that point s_rtc_sparmodus_active being true means this boot is a
     * real explicit wake (touch/PWRKEY/BOOT/RTC-alarm) out of an active
     * deep-sleep session, not a cold boot/reset. Must be called before
     * power_mgmt_init() (inside lvgl_app_start()) clears the flag - see
     * main/uwatch_main.c. */
    return s_rtc_sparmodus_active;
}

/* 60s "invisible" internal wake (docs/application.md section 10.3) - not
 * every minute on the wall clock, just every 60s of sleep duration; close
 * enough for "roughly once a minute" and avoids needing RTC-alarm-based
 * scheduling for something this loose. */
#define PM_SPARMODUS_TICK_US (60ULL * 1000000ULL)
/* Battery percent below which Ultra-Sparmodus auto-activates, and at or
 * below which a BOOT-long-press exit request is denied (docs/application.md
 * section 10.2/10.4). */
#define PM_SPARMODUS_BATT_PCT 10

/* 4 EXT1 pins = every existing light-sleep wake source (touch/PWRKEY/BOOT)
 * plus the RTC alarm line (so a scheduled alarm still reaches the user -
 * see the Phase 6 plan's alarm-during-Sparmodus decision), all active-low
 * (ESP_EXT1_WAKEUP_ANY_LOW), matching the LOW_LEVEL trigger these same
 * GPIOs already use for light-sleep wake in pm_arm_gpio_wakeup() below.
 * Touch is deliberately NOT armed here (per explicit feedback: it read as
 * too sensitive, e.g. an accidental wrist-brush waking the display) -
 * PWRKEY/BOOT/RTC-alarm only. */
static void pm_sparmodus_arm_wake(void)
{
    esp_sleep_enable_timer_wakeup(PM_SPARMODUS_TICK_US);
    /* GPIO7 is the PMIC's shared, active-low IRQ output.  Do not arm it
     * while a status bit is holding it low: EXT1 ANY_LOW would then wake
     * immediately and Ultra-Sparmodus would never actually remain asleep.
     * power_mgmt_sparmodus_enter_sleep() clears the PMIC latch before its
     * first entry; on a timer-only re-sleep this line was already clean. */
    uint64_t pins = (1ULL << PM_GPIO_BOOT);
    if (gpio_get_level(PM_GPIO_PWRKEY) == 1) {
        pins |= (1ULL << PM_GPIO_PWRKEY);
    } else {
        ESP_LOGI(TAG, "Ultra-Sparmodus: PWRKEY IRQ low at entry; not arming it");
    }
    /* Keep RTC armed even on a timer-only re-sleep before normal RAM state
     * is restored: a pending RTC alarm deliberately wakes immediately. */
    pins |= (1ULL << PM_GPIO_RTC);
    esp_sleep_enable_ext1_wakeup_io(pins, ESP_EXT1_WAKEUP_ANY_LOW);
}

/* Real deep-sleep entry: shuts every peripheral down (docs/application.md
 * section 10.3's "alle Peripherie wird aktiv abgeschaltet"), arms wake
 * sources, and calls esp_deep_sleep_start() - does not return. Currently
 * reachable only via the `sparmodus on` debug command (Stage 2 of Phase 6 -
 * auto-entry-on-idle-timeout wiring is Stage 3). */
void power_mgmt_sparmodus_enter_sleep(void)
{
    ESP_LOGI(TAG, "Ultra-Sparmodus: entering deep sleep");
    /* Panel must go dark here - deep sleep means the ESP32 stops driving it
     * entirely, and an AMOLED panel otherwise just keeps showing its last
     * latched frame indefinitely (its own GRAM holds it with no refresh
     * needed). Without this the watch looked "frozen"/unresponsive on a
     * real explicit wake - reported live and mistaken for a crash - since
     * the screen stayed lit on stale content while the chip was actually
     * correctly asleep underneath it. Use the same serialized lifecycle as
     * normal light sleep before cutting every peripheral rail. */
    esp_err_t display_err = display_controller_sleep(pdMS_TO_TICKS(1000));
    if (display_err != ESP_OK) {
        ESP_LOGW(TAG, "Ultra-Sparmodus display sleep: %s", esp_err_to_name(display_err));
    }
    lvgl_gps_set_enabled(false);   /* GNSS (BLDO1): clean stop through the driver's own path */

    /* LoRa (ALDO3): unlike the normal light-sleep path (which keeps this
     * rail on so mesh_log_task can keep listening across sleep), Sparmodus
     * cuts it - mesh listening is one of the things "ultra" power saving
     * gives up. mesh_log_task's RX loop runs continuously with no locking
     * of its own around the sx1262 driver's static state, so the rail
     * can't just be yanked out from under it (same bug class as the SD
     * card crash this session already hit) - mesh_log_stop_radio_for_sleep()
     * asks the task itself to stop cleanly and waits for confirmation
     * before it's safe to cut power. A timeout is logged and treated as
     * "proceed anyway" rather than blocking sleep entry indefinitely on a
     * stuck task. */
    esp_err_t mesh_stop_err = mesh_log_stop_radio_for_sleep();
    if (mesh_stop_err != ESP_OK) {
        ESP_LOGW(TAG, "Ultra-Sparmodus: mesh_log radio stop: %s (cutting rail anyway)",
                 esp_err_to_name(mesh_stop_err));
    }
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO3, false);

    /* BHI260AP motion sensor (ALDO4): the normal light-sleep path only
     * suspends it (bhi260ap_ap_suspend(), keeping ~0.1-0.3 mA alive) to
     * avoid a firmware re-upload on the next wake, since that path resumes
     * the SAME running app. Sparmodus's wake sources don't include IMU
     * gestures at all (touch/PWRKEY/BOOT/RTC-alarm only, see
     * pm_sparmodus_arm_wake()), and any wake here is a full reboot that
     * already re-initializes every driver from scratch regardless of
     * whether this rail was cut - so there's no re-upload cost being
     * avoided by leaving it on, only wasted current. bhi260ap_ap_suspend()
     * first for a clean stop of bhi260_task's I2C polling (same rationale
     * as the LoRa handshake above; that task already skips polling once
     * suspended) before cutting the rail a moment later. */
    bhi260ap_ap_suspend();
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO4, false);

    /* NFC (DLDO1): deliberately left untouched, matching the normal
     * light-sleep path's own reasoning (twatch_board.c/power_mgmt.c) - the
     * ST25R3916 "has not proven able to come back reliably after a rail
     * power-cycle", a real malfunction risk, not just an extra bring-up
     * cost. Not worth trading that for Sparmodus's incremental savings on
     * an already low-priority peripheral. */

    sd_log_unmount();
    /* VBUS-insert IRQ is a one-time PMIC register write (persists on its
     * own power, independent of the ESP32's sleep/reset cycle) - only
     * needs enabling once, here, not on every re-sleep. */
    axp2101_enable_vbus_irq(twatch_pmu_dev);
    /* Rail shutdown and the VBUS-IRQ setup can latch PMIC status bits.
     * GPIO7 must be de-asserted before EXT1 ANY_LOW is armed, exactly as in
     * the normal light-sleep path. */
    axp2101_clear_irq(twatch_pmu_dev);
    s_rtc_sparmodus_active = true;
    pm_sparmodus_arm_wake();
    esp_deep_sleep_start();
}

/* The silent per-minute wake's entire job: re-arm the same wake sources
 * and go straight back to sleep, touching nothing else (no display, no
 * peripheral I/O) - see the Phase 6 plan's "per-minute wake does no work
 * at all" decision. Peripherals are already off from enter_sleep() above;
 * nothing to redo. */
void power_mgmt_sparmodus_resleep(void)
{
    pm_sparmodus_arm_wake();
    esp_deep_sleep_start();
}

/* How long BOOT must be held continuously to exit Ultra-Sparmodus. */
#define PM_SPARMODUS_EXIT_HOLD_MS   3000

/* Watches BOOT for the rest of an explicit-wake session and exits Ultra-
 * Sparmodus on a 3s continuous hold (battery permitting) - see
 * power_mgmt_sparmodus_handle_wake(), which spawns this. Deliberately NOT
 * a one-shot check at boot: the user must be able to press BOOT at any
 * point while the watch face is up, not just in a narrow window right
 * after wake, and "how long the watch face stays up at all" is governed
 * entirely by the normal auto-sleep idle timeout (adapter_cfg_mut.auto_sleep
 * in lvgl_app.c, already wired to branch into power_mgmt_sparmodus_enter_sleep()
 * when the flag is still set) - not by anything in this file.
 *
 * "Halts the timer" while held (per explicit feedback): esp_lv_adapter_report_activity()
 * is called every 50ms of the hold, which resets that same idle timeout,
 * so a hold in progress can never be interrupted by an auto-resleep -
 * the display timeout and the exit-hold share one clock, not two competing
 * ones. */
static void sparmodus_boot_watch_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!power_mgmt_get_sparmodus_active()) {
            break;   /* exited some other way (VBUS insert, debug console) */
        }
        if (gpio_get_level(PM_GPIO_BOOT) == 0) {
            uint32_t held_ms = 0;
            while (gpio_get_level(PM_GPIO_BOOT) == 0) {
                esp_lv_adapter_report_activity();
                vTaskDelay(pdMS_TO_TICKS(50));
                held_ms += 50;
                if (held_ms >= PM_SPARMODUS_EXIT_HOLD_MS) {
                    break;
                }
            }
            if (held_ms >= PM_SPARMODUS_EXIT_HOLD_MS) {
                /* Read the PMIC directly rather than sensor_cache_get():
                 * no dependency on whether that background task has
                 * produced its first sample yet, and this isn't a hot
                 * path - one extra I2C round trip here is free. */
                uint8_t batt_pct = 0;
                bool batt_ok = axp2101_get_battery_pct(twatch_pmu_dev, &batt_pct) == ESP_OK;
                if (batt_ok && batt_pct > PM_SPARMODUS_BATT_PCT) {
                    ESP_LOGI(TAG, "Ultra-Sparmodus: BOOT held %u ms, battery %u%% - exiting",
                             (unsigned)PM_SPARMODUS_EXIT_HOLD_MS, (unsigned)batt_pct);
                    power_mgmt_set_sparmodus_active(false);
                    break;   /* nothing left to watch for */
                }
                ESP_LOGI(TAG, "Ultra-Sparmodus: BOOT held but battery too low (%u%%) - denied",
                         batt_ok ? (unsigned)batt_pct : 0);
                /* Denied, not exited - keep watching in case of a later,
                 * separate hold (e.g. after plugging in to charge). */
            }
            /* Released before 3s (or the hold was denied): fall back to
             * the outer loop and keep watching for the next press. */
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

/* Call once, after a full boot completes (lvgl_app_start() returned OK),
 * ONLY when power_mgmt_sparmodus_was_deep_sleep_wake() returned true
 * earlier in app_main() - i.e. this boot is an explicit wake out of an
 * active Sparmodus deep-sleep session, not a cold boot. Returns
 * immediately (never blocks the caller) - see main/uwatch_main.c. */
void power_mgmt_sparmodus_handle_wake(void)
{
    /* An alarm/timer firing is also one of the EXT1 wake sources (PM_GPIO_RTC)
     * - let that ring normally, with no BOOT-hold watcher competing for the
     * button; alarm_ring_cb() owns the screen/audio for as long as it's
     * ringing, and its own dismiss button already exists separately. */
    uint64_t ext1_status = esp_sleep_get_ext1_wakeup_status();
    if (ext1_status & (1ULL << PM_GPIO_RTC)) {
        ESP_LOGI(TAG, "Ultra-Sparmodus: woke for an alarm, skipping the BOOT-exit watcher");
        return;
    }
    xTaskCreate(sparmodus_boot_watch_task, "sparmodus_boot", 3072, NULL,
                ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY, NULL);
}

bool power_mgmt_get_night_mode_auto(void)
{
    return s_night_mode_auto;
}

void power_mgmt_set_night_mode_auto(bool on)
{
    if (on == s_night_mode_auto) {
        return;
    }
    s_night_mode_auto = on;
    /* Re-apply from the current time: turning auto off forces night mode off
     * immediately; turning it back on re-enters night mode right away. */
    pm_apply_night_mode(pm_is_night_time());
    pm_config_save();
}

bool power_mgmt_get_skip_sleep_on_usb(void)
{
    return s_skip_sleep_on_usb;
}

void power_mgmt_set_skip_sleep_on_usb(bool yes)
{
    if (yes == s_skip_sleep_on_usb) {
        return;
    }
    s_skip_sleep_on_usb = yes;
    pm_config_save();
}

static void pm_apply_night_mode(bool night)
{
    if (night == s_night_mode) {
        return;
    }
    s_night_mode = night;
    ESP_LOGI(TAG, "night mode %s", night ? "on" : "off");

    /* Dim to ~10% (night) or restore normal brightness. */
    display_controller_set_brightness(night ? PM_NIGHT_BRIGHTNESS : s_brightness);

    /* Touch input remains enabled at night so navigation works while the
     * watch is awake; only the light-sleep wake source is gated in
     * pm_arm_gpio_wakeup() (no touch-wake at night). */

    /* Force the UI to redraw everything so the red-only transform (or its
     * removal) is applied to every pixel, not just newly invalidated areas. */
    if (s_night_mode_cb) {
        s_night_mode_cb(night);
    }
}

/* Wakes the LVGL adapter from a task context. Calling the adapter's
 * *_from_isr() wake API from inside the shared GPIO ISR service crashed
 * (spinlock_acquire on a NULL mux), so button wakes are routed through this
 * task instead.
 *
 * The AXP IRQ line is latched LOW until its status registers are cleared over
 * I2C. pm_arm_gpio_wakeup() arms it as LOW_LEVEL for light-sleep wakeup, which
 * permanently switches the pin to level-triggering. A level-triggered interrupt
 * on a line held low re-fires forever (interrupt WDT timeout / reboot), so the
 * ISR disables the pins immediately and this task clears the AXP IRQ, restores
 * edge triggering and only then re-arms them. */
static void IRAM_ATTR button_isr(void *arg)
{
    uint32_t gpio = (uint32_t)arg;
    /* The IMU INT pulses on every gesture/step while awake; only act on it
     * when the host is actually asleep (light-sleep wake). While awake, keep
     * the interrupt DISABLED so a busy INT line cannot storm the CPU; it is
     * re-enabled in pm_arm_gpio_wakeup() when sleep is entered. Note: the
     * PWRKEY/BOOT disables come AFTER this early-return, so an awake IMU
     * pulse must not disable the button edge ISRs. */
    if (gpio == PM_GPIO_IMU && !s_imu_wake_armed) {
        gpio_intr_disable(PM_GPIO_IMU);
        return;
    }
    /* OR the source in rather than overwrite: two sources (e.g. alarm +
     * gesture) can fire close together on either core, and both must survive
     * to get their own handling in pm_wake_task instead of one silently
     * losing to the other. */
    portENTER_CRITICAL_ISR(&s_wake_sources_lock);
    s_wake_sources |= (1u << gpio);
    portEXIT_CRITICAL_ISR(&s_wake_sources_lock);
    portENTER_CRITICAL_ISR(&s_exit_wake_lock);
    s_exit_wake_sources |= (1u << gpio);
    portEXIT_CRITICAL_ISR(&s_exit_wake_lock);
    gpio_intr_disable(PM_GPIO_PWRKEY);
    gpio_intr_disable(PM_GPIO_BOOT);
    gpio_intr_disable(PM_GPIO_IMU);
    gpio_intr_disable(PM_GPIO_RTC);
    if (s_wake_task) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_wake_task, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

/* Distinguish a REAL wrist-raise/wake gesture from the BHI260AP's routine
 * WAKE-UP-FIFO traffic. The chip's scheduler continuously emits
 * SAMPLE_RATE_CHANGED / POWER_MODE_CHANGED meta events for the gesture sensors
 * (~1-3/s, also during AP-suspend), and with the WU-FIFO watermark at 1 every
 * one asserts the INT line and wakes the sleeping host. Draining the FIFO and
 * checking for an actual gesture flag keeps those spurious wakes invisible
 * (screen stays off, CPU returns to light sleep) while a real gesture still
 * turns the display on. Returns true if a gesture flag was latched. */
static bool pm_imu_wake_is_real(void)
{
    bhi260ap_drain_wakeup_fifo();   /* read the WU FIFO, de-assert the INT line */
    bool wrist = false, wake = false, glance = false, pickup = false, tilt = false;
    bhi260ap_consume_gestures(&wrist, &wake, &glance, &pickup, &tilt);
    bool real = wrist || wake || glance || pickup || tilt;
    if (!real) {
        ESP_LOGI(TAG, "IMU wake: no gesture flag (scheduler meta), staying asleep");
    }
    return real;
}

/* PWRKEY held past OFFLEVEL (4s - see axp2101_configure_pwrkey_shutdown()).
 * The PMIC's own instant hardware cutoff for this is disabled specifically so
 * this can run first: blank the display for a clean visual finish, unmount
 * the SD card (flushes + powers ALDO1 off - see sd_log_unmount()) so a card
 * pull mid-write can't corrupt it, then hand off to the PMIC's own controlled
 * power-off sequence. Does not return in the normal case - the watch loses
 * power once the PMIC acknowledges the command. */
void power_mgmt_shutdown(void)
{
    ESP_LOGI(TAG, "PWRKEY long-press: shutting down");
    /* The PMIC immediately removes panel power, so omit SLPIN deliberately.
     * This is still serialized with the LVGL flush path by the controller. */
    esp_err_t display_err = display_controller_power_off(pdMS_TO_TICKS(1000));
    if (display_err != ESP_OK) {
        ESP_LOGW(TAG, "shutdown display off: %s", esp_err_to_name(display_err));
    }
    sd_log_unmount();
    axp2101_soft_poweroff(twatch_pmu_dev);
}

/* Ultra-Sparmodus auto-entry/exit check period (docs/application.md
 * section 10.2/10.4) - independent of PM_NIGHT_CHECK_MS's cadence, a
 * battery-threshold/USB-plug transition deserves noticing sooner than
 * night mode's clock check. */
#define PM_SPARMODUS_CHECK_MS 30000

static void pm_wake_task(void *arg)
{
    (void)arg;
    uint32_t since_night_check = 0;
    uint32_t since_sparmodus_check = 0;
    /* Tracks the VBUS level as of the last check, to detect a present ->
     * absent edge (the "just unplugged" moment section 10.4 cares about,
     * as opposed to "still unplugged from before"). Seeded from the real
     * level on the first check below rather than assumed false, so a
     * device that boots already unplugged doesn't read as a false
     * unplug-edge on its very first tick. */
    bool sparmodus_vbus_seeded = false;
    bool sparmodus_last_vbus = false;
    for (;;) {
        /* Wake every 5 s: pump USB activity (so the adapter never hits its idle
         * timeout while plugged in -> no sleep attempt / no error spam), and
         * run the night-mode clock check every PM_NIGHT_CHECK_MS. A shorter
         * period would cap every light-sleep segment and force needless I2C
         * reads on battery. */
        uint32_t t = pdMS_TO_TICKS(5000);
        if (ulTaskNotifyTake(pdTRUE, t) == 0) {
            since_night_check += 5000;
            /* Pump USB activity so the adapter never hits its idle timeout
             * while plugged in (no sleep attempt / no error spam). Only when
             * "do not sleep on USB" is enabled - otherwise let it sleep even
             * on USB. */
            bool vbus = false;
            if (s_skip_sleep_on_usb &&
                    axp2101_is_vbus_present(twatch_pmu_dev, &vbus) == ESP_OK && vbus) {
                esp_lv_adapter_report_activity();
            }
            /* Safety net for snooze-timer wakes: RTC INT (GPIO1) is active-low.
             * If the line is held LOW on a timeout tick, the edge-ISR wake
             * attribution may have been missed (e.g. a LOW_LEVEL light-sleep wake
             * whose edge fired while LVGL was still paused, or the pin glitched
             * back out before the ISR ran). Re-arm the edge ISR and let the alarm
             * module check the flags, so a 10-min snooze re-ring is never silently
             * dropped just because no wake notification arrived. */
            if (gpio_get_level(PM_GPIO_RTC) == 0) {
                gpio_set_intr_type(PM_GPIO_RTC, GPIO_INTR_NEGEDGE);
                gpio_intr_enable(PM_GPIO_RTC);
                alarm_handle_wake();
                esp_lv_adapter_request_wake();
            }
            /* Same safety net for the BHI260AP gesture wake: while asleep its
             * INT line (GPIO8) is held LOW until the WU FIFO is drained, so if
             * the edge ISR attribution missed the wake (level wake without a
             * clean edge on the tick), a wrist-tilt/wake gesture would never
             * turn the display on. Re-arm the edge ISR and request a wake.
             * gated on the same conditions used to arm the gesture wake
             * (not night mode, IMU wake actually armed). */
            if (!s_night_mode && s_imu_wake_armed && gpio_get_level(PM_GPIO_IMU) == 0) {
                gpio_set_intr_type(PM_GPIO_IMU, GPIO_INTR_NEGEDGE);
                gpio_intr_enable(PM_GPIO_IMU);
                if (pm_imu_wake_is_real()) {
                    esp_lv_adapter_request_wake();
                }
            }
            if (since_night_check >= PM_NIGHT_CHECK_MS) {
                since_night_check = 0;
                pm_apply_night_mode(pm_is_night_time());
            }
            since_sparmodus_check += 5000;
            if (!sparmodus_vbus_seeded ||
                    since_sparmodus_check >= PM_SPARMODUS_CHECK_MS) {
                since_sparmodus_check = 0;
                bool vbus_now = false;
                axp2101_is_vbus_present(twatch_pmu_dev, &vbus_now);
                if (!sparmodus_vbus_seeded) {
                    /* First tick: just record the level, no edge to react
                     * to yet (nothing to compare against). */
                    sparmodus_vbus_seeded = true;
                } else if (vbus_now && s_sparmodus_active) {
                    /* USB inserted while Sparmodus was active (either it
                     * was auto/manually entered before this device woke,
                     * or the user plugged in during an explicit-wake
                     * session) - full exit, no special "restore" needed
                     * (the next real sleep/wake cycle is just normal). */
                    ESP_LOGI(TAG, "Ultra-Sparmodus: USB inserted, exiting");
                    power_mgmt_set_sparmodus_active(false);
                } else if (!vbus_now) {
                    sensor_cache_t cache;
                    sensor_cache_get(&cache);
                    bool low_batt = cache.valid && cache.batt_pct < PM_SPARMODUS_BATT_PCT;
                    if (sparmodus_last_vbus && low_batt) {
                        /* Just unplugged, still low - re-enter immediately
                         * (section 10.4's "sofort wieder aktiviert"),
                         * rather than waiting for the next idle timeout. */
                        ESP_LOGI(TAG, "Ultra-Sparmodus: USB removed, battery still low, re-entering");
                        power_mgmt_set_sparmodus_active(true);
                        power_mgmt_sparmodus_enter_sleep();   /* does not return */
                    } else if (!s_sparmodus_active && low_batt) {
                        /* Plain auto-entry (section 10.2): just set the
                         * flag: power_mgmt_enter_sleep() picks it up on
                         * the next idle timeout, same as a manual Settings
                         * toggle, rather than cutting the display mid-
                         * interaction. */
                        ESP_LOGI(TAG, "Ultra-Sparmodus: battery below %u%%, auto-activating",
                                 PM_SPARMODUS_BATT_PCT);
                        power_mgmt_set_sparmodus_active(true);
                    }
                }
                sparmodus_last_vbus = vbus_now;
            }
            continue;
        }

        since_night_check = 0;
        portENTER_CRITICAL(&s_wake_sources_lock);
        uint32_t sources = s_wake_sources;
        s_wake_sources = 0;
        portEXIT_CRITICAL(&s_wake_sources_lock);
        ESP_LOGI(TAG, "wake: sources=0x%x", (unsigned)sources);

        /* De-assert the latched AXP IRQ line. */
        if (sources & PM_WAKE_PWRKEY) {
            uint32_t irq = 0;
            axp2101_get_irq_status(twatch_pmu_dev, &irq);
            axp2101_clear_irq(twatch_pmu_dev);
            char irq_desc[160];
            axp_irq_describe(irq, irq_desc, sizeof(irq_desc));
            ESP_LOGI(TAG, "AXP IRQ wake: irq=0x%06lx (%s)", (unsigned long)irq, irq_desc);
            if (irq & AXP_IRQ_PEK_LONG) {
                power_mgmt_shutdown();
                /* Falls through (no continue/return) rather than skip the
                 * rest of this wake cycle: if we're still running, the PMIC
                 * didn't actually cut power (e.g. on USB, where poweroff
                 * behavior can differ) - let the normal re-arm-buttons +
                 * wake-display flow below run so the watch stays usable
                 * instead of getting stuck mid-shutdown until a real reset. */
            } else if (s_button_cb) {
                /* Short press (bit8/bit11, i.e. not PEK_LONG) - e.g. dismiss
                 * a ringing alarm. Not gated on which IRQ bit specifically,
                 * same as BOOT below: the callback itself decides whether
                 * there's anything to do right now. */
                s_button_cb();
            }
        } else if ((sources & PM_WAKE_BOOT) && s_button_cb) {
            s_button_cb();
        }

        /* Restore edge triggering (gpio_wakeup_enable() left these level) and
         * re-arm the button + IMU pins. */
        gpio_set_intr_type(PM_GPIO_PWRKEY, GPIO_INTR_NEGEDGE);
        gpio_set_intr_type(PM_GPIO_BOOT, GPIO_INTR_NEGEDGE);
        gpio_intr_enable(PM_GPIO_PWRKEY);
        gpio_intr_enable(PM_GPIO_BOOT);
        /* Only re-arm the IMU edge ISR if the wake actually came from it; the
         * ISR self-disables on every awake pulse, and re-enabling it here on
         * every wake-task run would leave a live edge on a busy line. */
        if ((sources & PM_WAKE_IMU) && s_imu_wake_armed) {
            gpio_set_intr_type(PM_GPIO_IMU, GPIO_INTR_NEGEDGE);
            gpio_intr_enable(PM_GPIO_IMU);
        }

        /* A BHI260AP INT is not proof of a gesture: scheduler meta events in
         * the WU FIFO assert the same line constantly. Only wake the display
         * for it when a real gesture flag was latched; a non-IMU source
         * always wakes the display regardless. pm_imu_wake_is_real() also
         * drains the WU FIFO (de-asserts the INT line), so it must run
         * whenever the IMU bit is set even if another source already
         * justifies waking - it's not just a boolean check. */
        bool wake_user = (sources & ~(uint32_t)PM_WAKE_IMU) != 0;
        if ((sources & PM_WAKE_IMU) && s_imu_wake_armed) {
            bool imu_real = pm_imu_wake_is_real();
            wake_user = wake_user || imu_real;
        }

/* RTC INT (GPIO1): the PCF85063A pulls the line LOW when the alarm
 * (AF) or snooze timer (TF) fires. Restore edge triggering (the
 * LOW_LEVEL sleep wake left it level) and re-arm the ISR, then let
 * the alarm module check the flags and start the ring. The ring task
 * clears the pending flag(s), which de-asserts the line. */
if (sources & PM_WAKE_RTC) {
    gpio_set_intr_type(PM_GPIO_RTC, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(PM_GPIO_RTC);
    alarm_handle_wake();
}

        if (wake_user) {
            esp_lv_adapter_request_wake();
        } else {
            /* Spurious: the panel was never woken, so the screen stays off and
             * this costs only the idle timeout the adapter restarts, not a lit
             * display for the whole of it. */
            ESP_LOGI(TAG, "IMU wake was spurious, staying in light sleep");
        }
    }
}

static void pm_arm_gpio_wakeup(void)
{
    s_wake_sources = 0;
    /* Clear any previously-armed RTC level wakeups. gpio_wakeup_enable()
     * config persists across sleep cycles until explicitly disabled, so a
     * stale LOW_LEVEL from an earlier cycle could instantly re-wake the watch
     * on a line that is low at entry. Re-arm from a clean state every cycle. */
    touch_controller_set_wake_enabled(false);
    gpio_wakeup_disable(PM_GPIO_IMU);
    gpio_wakeup_disable(PM_GPIO_PWRKEY);
    gpio_wakeup_disable(PM_GPIO_BOOT);
    gpio_wakeup_disable(PM_GPIO_RTC);
    s_imu_wake_armed = false;
    gpio_intr_disable(PM_GPIO_IMU);

    /* The rail shutdowns in power_mgmt_enter_sleep() (GNSS SPI/I2C BLDO1,
     * etc.) latch AXP IRQ status bits and hold the GPIO7 line LOW; if PWRKEY
     * is then armed as a LOW_LEVEL wake the ESP32 re-wakes the instant light
     * sleep starts. Clear it here so the line de-asserts before arming. */
    axp2101_clear_irq(twatch_pmu_dev);

    /* Touch and IMU-gesture are not wake sources in night mode (avoid
     * accidental screen activation); PWR/BOOT buttons always wake. The
     * BHI260AP INT line (GPIO8) is active-LOW (idles high, pulses low on a
     * wake-up gesture). During light sleep the FIFO is not drained, so the
     * line holds low -> LOW_LEVEL wakes the watch. */
    if (!s_night_mode) {
        touch_controller_set_wake_enabled(true);
        /* The IMU edge ISR is disabled while awake (any pulse disables it);
         * arm it again so a gesture during light sleep wakes the adapter.
         * Only enable the level-wake if the line is de-asserted (high): if a
         * spurious gesture already asserted it low, a LOW_LEVEL wake would fire
         * instantly and the watch would never sleep. bhi260ap_ap_suspend()
         * already drained any leftover WU-FIFO event, but a real gesture can
         * still land during the power-down window; drain again on each retry so
         * a transient low doesn't permanently disable gesture wake (which would
         * eat the very gesture that's supposed to wake the watch). */
        for (int try = 0; try < 5; try++) {
            bhi260ap_drain_wakeup_fifo();
            if (gpio_get_level(PM_GPIO_IMU) == 1) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        if (gpio_get_level(PM_GPIO_IMU) == 1) {
            gpio_wakeup_enable(PM_GPIO_IMU, GPIO_INTR_LOW_LEVEL);
            s_imu_wake_armed = true;
            gpio_intr_enable(PM_GPIO_IMU);
        } else {
            ESP_LOGI(TAG, "IMU INT low at sleep, gesture wake disabled this cycle");
        }
    }
    /* PWRKEY (GPIO7, AXP IRQ) and BOOT wake always. Arm as LOW_LEVEL only if
     * the line is de-asserted (high): the AXP IRQ line is latched LOW until
     * its status is cleared, so a pending PWRKEY/rail event would otherwise
     * fire a LOW_LEVEL wake the instant light sleep starts (the watch would
     * never actually sleep). Blocking on a low output covers presses logged
     * during the sleep-entry window itself, which the edge ISR already saw. */
    gpio_wakeup_enable(PM_GPIO_BOOT, GPIO_INTR_LOW_LEVEL);
    if (gpio_get_level(PM_GPIO_PWRKEY) == 1) {
        gpio_wakeup_enable(PM_GPIO_PWRKEY, GPIO_INTR_LOW_LEVEL);
    } else {
        ESP_LOGI(TAG, "PWRKEY low at sleep, power-key wake disabled this cycle");
    }

    /* RTC INT (GPIO1): the PCF85063A pulls it LOW when the alarm (AF) or the
     * snooze timer (TF) fires, and it stays low until the flag is cleared.
     * Arm it as a LOW_LEVEL wake only while an alarm is armed; the ring task
     * re-arms the daily alarm after dismiss and the snooze timer after snooze,
     * both of which clear the pending flag and de-assert the line. */
    if (alarm_is_armed()) {
        gpio_wakeup_enable(PM_GPIO_RTC, GPIO_INTR_LOW_LEVEL);
    }
    esp_sleep_enable_gpio_wakeup();
    /* ...plus the housekeeping heartbeat. Deliberately unconditional: logging
     * should keep sampling in night mode and while the screen is off, which is
     * exactly when a drain is worth recording. power_mgmt_exit_sleep() keeps
     * the display dark for these (see its wake classification), so the cost is
     * one brief CPU wake per minute, not a lit panel. */
    esp_sleep_enable_timer_wakeup(PM_HOUSEKEEPING_WAKE_US);
}

esp_err_t power_mgmt_enter_sleep(void *ctx)
{
    (void)ctx;

    bool vbus = false;
    axp2101_is_vbus_present(twatch_pmu_dev, &vbus);

    /* Ultra-Sparmodus takes over the idle-timeout moment entirely (deep
     * sleep instead of the normal light-sleep cycle below) - unless VBUS
     * is present, in which case being plugged in already means "not low-
     * battery survival mode" even if the periodic exit check in
     * pm_wake_task() hasn't run yet, so just fall through to the normal
     * light-sleep path instead. Never returns when it does fire. */
    if (s_sparmodus_active && !vbus) {
        power_mgmt_sparmodus_enter_sleep();
    }

    /* Skip auto-sleep while on USB power (charging / development) unless the
     * user disabled the "do not sleep on USB" setting. */
    if (s_skip_sleep_on_usb && vbus) {
        ESP_LOGD(TAG, "on USB power, skipping auto sleep");
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_sleep_enter_us = esp_timer_get_time();
    ESP_LOGI(TAG, "entering sleep: panel DISPOFF+blank+SLPIN, IMU AP-suspend, rails off");
    esp_err_t display_err = display_controller_sleep(pdMS_TO_TICKS(1000));
    if (display_err != ESP_OK) {
        ESP_LOGW(TAG, "display sleep: %s", esp_err_to_name(display_err));
        return display_err;
    }
    bhi260ap_ap_suspend();

    /* Disable unused peripheral rails. ALDO4 (sensor) is kept ON: the
     * BHI260AP runs its wake-up sensors in AP-suspend mode (wrist-raise wake)
     * at ~0.1-0.3 mA, avoiding the firmware re-upload + data freeze after a
     * rail power-cycle. Keep ALDO2 (display/touch) for touch wake.
     * ALDO3 (LoRa) is KEPT ON, unconditionally: mesh_log.c's background task
     * keeps the SX1262 in RX Continuous mode for always-on Meshtastic
     * listening, and a rail power-cycle would mean re-running the whole
     * TCXO/RF-switch/frequency bring-up sequence on every wake instead of
     * just continuing to listen. */
    /* GNSS (BLDO1): cut the rail only when GNSS is neither deliberately
     * enabled NOR actively mid-acquisition. With the GPS-screen switch on
     * (lvgl_gps_enabled()), the receiver is kept alive across the whole
     * sleep session so wake-ups don't pay the ~4 s cold re-power + warm
     * start each time. gps_screen_is_powered() covers the other real case:
     * a one-shot boot-time/console "gpscheck" refresh (GPS_CTRL_REFRESH)
     * powers the module on without setting the persisted switch flag - if
     * light-sleep cut the rail mid-search there, a walk with enough wrist
     * motion to repeatedly trigger wrist-raise wake (see button_isr's own
     * "storm the CPU" comment on why that source exists) would restart
     * acquisition from a cold power-on every single wake, and a fix could
     * take many minutes to never actually land - confirmed as the likely
     * cause of a live "walked outside for several minutes, no fix" report.
     * When neither is true the rail is cut as usual and stays off. */
    if (lvgl_gps_enabled() || gps_screen_is_powered()) {
        ESP_LOGI(TAG, "GNSS enabled/active: keeping BLDO1 rail on across sleep");
    } else {
        m10q_power(false);                                       /* GNSS (tells the driver) */
    }
    /* BLDO2 (speaker) isn't touched here - it's left off at boot (see
     * axp2101_set_default_power()) and only powered around actual playback
     * (alarm.c, debug_audio.c). DLDO1 (NFC) is left on: it's registered
     * with spi2_power as an SPI2_POWER_OWNED rail (twatch_board.c), which
     * nothing ever auto-lowers - the chip has not proven able to come back
     * reliably after a rail power-cycle once bring-up has completed, and
     * st25r3916_close() already stops the RF field, which is the part that
     * actually costs anything. */

    /* SD card (ALDO1): unmount cleanly and last, after everything above has
     * had its chance to log - sd_log_unmount() flushes the RAM ring to disk
     * before it unmounts, so anything logged during this function (including
     * the "entering sleep" line above) is captured, not lost.
     *
     * Whether the rail is then cut is spi2_power's call, not ours: ALDO1 is
     * registered as an SPI2_POWER_SHARED rail (twatch_board.c), because an
     * unpowered card clamps the shared MISO net and the SX1262 still services
     * LoRa RX from its DIO1 interrupt during light sleep. It's cut only when
     * the socket is empty - see spi2_power.h and sd_log.c's
     * sd_rail_reconcile(). */
    sd_log_unmount();

    pm_arm_gpio_wakeup();
    return ESP_OK;
}

/* The controller owns panel recovery and first-frame gating on every normal
 * light-sleep exit. Wake-source attribution remains here for diagnostics and
 * peripheral handling only; it no longer controls panel visibility. */
esp_err_t power_mgmt_exit_sleep(void *ctx)
{
    (void)ctx;
    s_imu_wake_armed = false;
    uint32_t causes = esp_sleep_get_wakeup_causes();   /* bitmap, not a single enum */

    portENTER_CRITICAL(&s_exit_wake_lock);
    uint32_t src = s_exit_wake_sources;
    s_exit_wake_sources = 0;
    portEXIT_CRITICAL(&s_exit_wake_lock);
    int64_t slept_us = (s_sleep_enter_us > 0) ? (esp_timer_get_time() - s_sleep_enter_us) : 0;
    ESP_LOGI(TAG, "waking: src=0x%x causes=0x%x slept=%dms panel=wake",
             (unsigned)src, (unsigned)causes, (int)(slept_us / 1000));

    /* Restore rails. ALDO3 (LoRa) stayed on across sleep, untouched; the
     * rest are rearmed. */
    /* GNSS (BLDO1): mirror enter_sleep's condition. Restoring this
     * unconditionally desyncs m10q's s_powered from the physical rail (the
     * driver never learns it came back on), which makes every later
     * m10q_power(false) in enter_sleep a silent no-op and leaves BLDO1
     * permanently on (~25-30 mA) once GPS is switched off mid-session. */
    if (lvgl_gps_enabled() || gps_screen_is_powered()) {
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO1, true);
    }
    /* BLDO2 (speaker) isn't restored here - it stays off across sleep and
     * wake alike; see enter_sleep's comment. */

    /* SD card (ALDO1): deliberately NOT remounted here - it now mounts only
     * on demand around an actual write (sd_log_session_begin()/end(), see
     * sd_log.h), not for the whole awake session, so there's nothing to
     * restore just because the watch woke up. */

    /* Wake the panel FIRST, before touching the IMU. Used to be the other
     * way around (IMU resume, then co5300_wake()) - a live incident showed
     * the display stuck white after a GPIO wake, system otherwise still
     * responsive (console/scheduler fine), and even a full panel-level
     * recovery (power-cycle the display rail, resend wake commands from a
     * completely separate code path) couldn't bring it back. Root cause
     * wasn't conclusively pinned down before the device had to be rebooted
     * to restore it, but bhi260ap_ap_resume() sitting ahead of the panel
     * wake meant anything that made IT slow or stuck would delay the most
     * user-visible part of waking - the screen - for no good reason: the
     * IMU and the display don't depend on each other. Panel first now, so
     * a slow/stuck sensor resume can never hold the screen hostage; see
     * bhi260ap_ap_resume()'s own header comment for the matching bounded-
     * wait change on the BHI260AP side.
     *
     * Every co5300 call below used to have its esp_err_t discarded - a
     * transient SPI hiccup on SLPOUT or DISPON (co5300_send_cmd() itself
     * never logs a failure, see co5300.c) would leave the panel in an
     * inconsistent state with zero trace anywhere, while the rest of this
     * function proceeded as if it had worked. That's a strong match for a
     * live report of white screens with no corresponding log line at all,
     * correlating with wrist-motion-triggered wake-from-sleep during
     * physical activity (many wake cycles = many chances to hit this) -
     * the same failure class as night_mode_draw_bitmap()'s DMA-retry fix,
     * just at the panel-command level instead of the pixel-data level.
     * Retry + log now, so if this recurs there's real evidence in
     * synclog next time instead of nothing. */
    /* Full re-initialization on wake, not a bare co5300_wake().
     *
     * co5300_wake() only sends SLPOUT + brightness. Established by live
     * reproduction (2026-08-31): once the panel is in the "white screen"
     * state it still scans at 60 Hz and still accepts commands (DISPOFF
     * blanks it), but pixel writes stop landing - and re-sending the whole
     * init command list plus a forced full repaint does NOT recover it.
     * Only a hardware RST pulse followed by re-init does, reproducibly.
     * So the old wake path could never heal a panel that went wedged while
     * the host was asleep; it would come back white and stay white.
     *
     * Doing it here is close to free: the screen is off at this point and a
     * full repaint follows anyway, so the reset is invisible. It costs the
     * panel's SLPOUT/DISPON settling (~250 ms) on each wake, which is the
     * price of a display that always comes back. The LVGL lock is held so
     * the flush task cannot push pixels at a panel that is mid-reset.
     *
     * NOTE: the underlying trigger is still not root-caused - it is NOT
     * simply mechanical shock (the panel has gone white with the watch
     * unplugged and undisturbed), and synthetic SLPIN/SLPOUT cycling does
     * not reproduce it. This makes the failure self-healing rather than
     * preventing it. See the `dispfix` debug command for manual recovery. */
    display_controller_request_visible(DISPLAY_REASON_WAKE);

    /* Resume the IMU wake-up streams now that the panel is up - see the
     * comment above co5300_wake() for why this moved to run after, not
     * before, the panel wake. */
    bhi260ap_ap_resume();

    /* Safety net: clear any pending AXP IRQ (de-asserts the GPIO7 line).
     * Detailed power-key reporting happens in pm_wake_task. */
    axp2101_clear_irq(twatch_pmu_dev);
    return ESP_OK;
}

void power_mgmt_load_config(void)
{
    pm_config_load();
}

void power_mgmt_init(void)
{
    power_mgmt_load_config();
    /* This boot is proceeding normally past app_main()'s early fork (see
     * power_mgmt_sparmodus_should_resleep_silently()) - whether that's a
     * cold boot or an explicit Sparmodus wake, the RTC flag's only job was
     * telling that fork apart from a silent timer wake, and it's now
     * spent either way (power_mgmt_sparmodus_enter_sleep() sets it again
     * if/when this boot goes back to deep sleep). */
    s_rtc_sparmodus_active = false;
    /* Apply the persisted night-mode setting on boot. */
    pm_apply_night_mode(pm_is_night_time());
    /* Ultra-Sparmodus's reduced brightness (docs/application.md section
     * 10.3) - kept independent of night mode (which also disables touch
     * input; Sparmodus must NOT do that, since the user needs touch/swipe
     * to reach Settings and turn it off during an explicit wake). Reuses
     * the existing dim level rather than introducing a second one the
     * spec doesn't actually distinguish. */
    if (s_sparmodus_active) {
        display_controller_set_brightness(PM_NIGHT_BRIGHTNESS);
    }
    /* DFS + automatic light sleep (tickless). */
    esp_pm_config_t pm = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = true,
    };
    esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "esp_pm: DFS 240/80 MHz + auto light sleep");
    }

    /* Enable the AXP2101 PEK (power key) interrupt -> GPIO7. */
    axp2101_enable_pek_irq(twatch_pmu_dev);
    /* Mask the gauge's periodic new-SOC interrupt. It is enabled by the AXP's
     * power-on defaults and asserts the same IRQ line the power key uses, so
     * it showed up here as a PM_WAKE_PWRKEY wake roughly every 20 s - ending
     * every light-sleep period and putting the watch back into a full awake
     * cycle with the display on. See axp2101_disable_gauge_soc_irq(). */
    axp2101_disable_gauge_soc_irq(twatch_pmu_dev);
    axp2101_clear_irq(twatch_pmu_dev);

    /* PWRKEY long-press (>4s) shuts the watch down, but in software: disable
     * the PMIC's own instant hardware cutoff on a long press and instead
     * unmount the SD card cleanly before calling axp2101_soft_poweroff() -
     * see pm_wake_task()'s PWRKEY handling below. */
    esp_err_t pwrkey_err = axp2101_configure_pwrkey_shutdown(twatch_pmu_dev);
    if (pwrkey_err != ESP_OK) {
        ESP_LOGW(TAG, "axp2101_configure_pwrkey_shutdown: %s", esp_err_to_name(pwrkey_err));
    }

    /* Shared GPIO ISR service; the touch driver installs it first, so
     * "already installed" is fine. */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
    }
    /* Wake handler task (priority above the LVGL adapter task). Created before
     * the button ISRs so button_isr always has a task to re-arm the pins. */
    xTaskCreate(pm_wake_task, "pm_wake", 4096, NULL,
                ESP_LV_ADAPTER_DEFAULT_TASK_PRIORITY + 1, &s_wake_task);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PM_GPIO_PWRKEY) | (1ULL << PM_GPIO_BOOT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);
    gpio_isr_handler_add(PM_GPIO_PWRKEY, button_isr, (void *)(uintptr_t)PM_GPIO_PWRKEY);
    gpio_isr_handler_add(PM_GPIO_BOOT, button_isr, (void *)(uintptr_t)PM_GPIO_BOOT);

    /* BHI260AP INT (GPIO8) as input. The INT is active-low (idles high, pulses
     * low on a wake-up gesture). Its falling edge is used as an awake-time
     * wake request (mirrors the button path); during light sleep it is armed
     * as a LOW_LEVEL wake source in pm_arm_gpio_wakeup(). */
    gpio_config_t imu_io = {
        .pin_bit_mask = (1ULL << PM_GPIO_IMU),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&imu_io);
    gpio_isr_handler_add(PM_GPIO_IMU, button_isr, (void *)(uintptr_t)PM_GPIO_IMU);
    /* Start with the IMU edge ISR disabled; it is armed only when sleep is
     * entered (pm_arm_gpio_wakeup) and self-disables on any pulse while awake,
     * so a busy INT line can never storm the CPU. */
    gpio_intr_disable(PM_GPIO_IMU);

    /* PCF85063A INT (GPIO1) as input. It idles high and pulses/stays LOW while
     * the alarm (AF) or snooze timer (TF) flag is set. Falling edge wakes the
     * power task to start the ring; during light sleep it is additionally
     * armed as a LOW_LEVEL wake source in pm_arm_gpio_wakeup(). */
    gpio_config_t rtc_io = {
        .pin_bit_mask = (1ULL << PM_GPIO_RTC),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,   /* INT is open-drain, active-low */
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&rtc_io);
    gpio_isr_handler_add(PM_GPIO_RTC, button_isr, (void *)(uintptr_t)PM_GPIO_RTC);

    /* Apply the initial night-mode state (and touch-ISR state). */
    pm_apply_night_mode(pm_is_night_time());

    /* Silence the adapter's periodic auto-sleep INFO chatter (it retries every
     * 5 s while on USB); errors still print. */
    esp_log_level_set("esp_lvgl:adapter", ESP_LOG_WARN);
}
