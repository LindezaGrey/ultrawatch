/*
 * power_mgmt.c - watch power management.
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
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "co5300.h"
#include "pcf85063a.h"
#include "bhi260ap.h"
#include "m10q.h"
#include "sensor_cache.h"
#include "lvgl_app.h"
#include "alarm.h"

static const char *TAG = "power_mgmt";

#define PM_GPIO_TOUCH  12
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

/* Night-mode clock check period while the watch is idle. */
#define PM_NIGHT_CHECK_MS  60000

/* NVS-persisted settings: night-mode auto and "do not sleep while on USB". */
#define PM_NVS_NS         "pm"
#define PM_NVS_KEY_NIGHT  "night_auto"
#define PM_NVS_KEY_USB    "skip_usb"

static bool s_night_mode_auto = true;    /* default: auto-enter night mode */
static bool s_skip_sleep_on_usb = true;  /* default: never sleep on USB */

/* RTC-capable GPIO wakeup for the touch line is armed here. */
static volatile uint32_t s_wake_sources;   /* bitmask of PM_WAKE_*, ISR-writer/task-reader */
static portMUX_TYPE s_wake_sources_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_wake_task;
static volatile bool s_night_mode;
static volatile bool s_imu_wake_armed;   /* GPIO8 ISR only notifies while asleep */
static power_mgmt_night_mode_cb_t s_night_mode_cb;

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

/* ---- Persisted settings (night-mode auto, sleep-on-USB) ---- */

static void pm_config_save(void)
{
    nvs_handle_t h;
    if (nvs_open(PM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, PM_NVS_KEY_NIGHT, s_night_mode_auto ? 1 : 0);
        nvs_set_u8(h, PM_NVS_KEY_USB, s_skip_sleep_on_usb ? 1 : 0);
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
        nvs_close(h);
    }
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
    co5300_set_brightness(night ? PM_NIGHT_BRIGHTNESS : 0x80);

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

static void pm_wake_task(void *arg)
{
    (void)arg;
    uint32_t since_night_check = 0;
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
            ESP_LOGI(TAG, "power key: irq=0x%06lx (bit8 press, bit10 long, bit11 short)",
                     (unsigned long)irq);
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
    gpio_wakeup_disable(PM_GPIO_TOUCH);
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
        gpio_wakeup_enable(PM_GPIO_TOUCH, GPIO_INTR_LOW_LEVEL);
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
}

esp_err_t power_mgmt_enter_sleep(void *ctx)
{
    (void)ctx;

    /* Skip auto-sleep while on USB power (charging / development) unless the
     * user disabled the "do not sleep on USB" setting. */
    bool vbus = false;
    if (s_skip_sleep_on_usb &&
            axp2101_is_vbus_present(twatch_pmu_dev, &vbus) == ESP_OK && vbus) {
        ESP_LOGD(TAG, "on USB power, skipping auto sleep");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "entering sleep: panel DISPOFF+blank+SLPIN, IMU AP-suspend, rails off");
    co5300_display_off();
    co5300_blank();
    co5300_sleep();
    bhi260ap_ap_suspend();

    /* Disable unused peripheral rails. ALDO4 (sensor) is kept ON: the
     * BHI260AP runs its wake-up sensors in AP-suspend mode (wrist-raise wake)
     * at ~0.1-0.3 mA, avoiding the firmware re-upload + data freeze after a
     * rail power-cycle. Keep ALDO2 (display/touch) for touch wake.
     * ALDO1 (SD) is KEPT ON: the FAT is left mounted across sleep, and
     * power-cycling the rail without unmount leaves the card's SD registers
     * undefined, so the log flush after wake fails with resp/CRC errors and
     * the battery log is lost. The card draws little at idle.
     * ALDO3 (LoRa) is also KEPT ON, unconditionally, same reasoning as
     * ALDO4/sensor: mesh_log.c's background task keeps the SX1262 in RX
     * Continuous mode for always-on Meshtastic listening, and a rail
     * power-cycle would mean re-running the whole TCXO/RF-switch/frequency
     * bring-up sequence on every wake instead of just continuing to listen. */
    /* GNSS (BLDO1): cut the rail only when GNSS is not deliberately enabled.
     * With the GPS-screen switch on, the receiver is kept alive across the
     * whole sleep session so wake-ups don't pay the ~4 s cold re-power + warm
     * start each time (one clean power-on per session). When the switch is
     * off the rail is cut as usual and stays off. */
    if (lvgl_gps_enabled()) {
        ESP_LOGI(TAG, "GNSS enabled: keeping BLDO1 rail on across sleep");
    } else {
        m10q_power(false);                                       /* GNSS (tells the driver) */
    }
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);  /* speaker */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_DLDO1, false);  /* NFC */

    pm_arm_gpio_wakeup();
    return ESP_OK;
}

esp_err_t power_mgmt_exit_sleep(void *ctx)
{
    (void)ctx;
    s_imu_wake_armed = false;
    esp_sleep_wakeup_cause_t cause = (esp_sleep_wakeup_cause_t)esp_sleep_get_wakeup_causes();
    ESP_LOGI(TAG, "waking: sources=0x%x cause=0x%x", (unsigned)s_wake_sources, (unsigned)cause);

    /* Restore rails. ALDO1 (SD) and ALDO3 (LoRa) stay on across sleep; the
     * rest are rearmed. */
    /* GNSS (BLDO1): mirror enter_sleep's condition. Restoring this
     * unconditionally desyncs m10q's s_powered from the physical rail (the
     * driver never learns it came back on), which makes every later
     * m10q_power(false) in enter_sleep a silent no-op and leaves BLDO1
     * permanently on (~25-30 mA) once GPS is switched off mid-session. */
    if (lvgl_gps_enabled()) {
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO1, true);
    }
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_DLDO1, true);

    /* Resume the IMU wake-up streams before waking the panel. */
    bhi260ap_ap_resume();

    co5300_wake();
    co5300_set_brightness(s_night_mode ? PM_NIGHT_BRIGHTNESS : 0x80);

    /* GRAM was blanked before sleep; force a full repaint so the screen shows
     * the current UI instead of staying black (SPI path doesn't auto-refresh).
     * DISPON is sent only after the repaint so no stale frame flashes. */
    lvgl_force_redraw();
    co5300_display_on();

    /* Safety net: clear any pending AXP IRQ (de-asserts the GPIO7 line).
     * Detailed power-key reporting happens in pm_wake_task. */
    axp2101_clear_irq(twatch_pmu_dev);
    return ESP_OK;
}

void power_mgmt_init(void)
{
    pm_config_load();
    /* Apply the persisted night-mode setting on boot. */
    pm_apply_night_mode(pm_is_night_time());
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
    axp2101_clear_irq(twatch_pmu_dev);

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
