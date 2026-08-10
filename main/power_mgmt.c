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
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "co5300.h"
#include <time.h>

static const char *TAG = "power_mgmt";

#define PM_GPIO_TOUCH  12
#define PM_GPIO_PWRKEY 7    /* AXP2101 IRQ */
#define PM_GPIO_BOOT   0

/* Night-mode clock check period while the watch is idle. */
#define PM_NIGHT_CHECK_MS  60000

/* RTC-capable GPIO wakeup for the touch line is armed here. */
static volatile uint32_t s_wake_gpio;
static TaskHandle_t s_wake_task;
static volatile bool s_night_mode;
static power_mgmt_night_mode_cb_t s_night_mode_cb;

/* Night mode is active between PM_NIGHT_START_HOUR (inclusive) and
 * PM_NIGHT_END_HOUR (exclusive), wrapping midnight. */
static bool pm_is_night_time(void);
static void pm_apply_night_mode(bool night);

static bool pm_is_night_time(void)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    int h = tm.tm_hour;
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
    /* Force the touch-ISR disable/enable even if the state didn't change
     * (e.g. a driver like esp_lcd_touch re-enabled the GPIO interrupt after
     * night mode disabled it). */
    if (s_night_mode) {
        gpio_intr_disable(PM_GPIO_TOUCH);
    } else {
        gpio_intr_enable(PM_GPIO_TOUCH);
    }
}

void power_mgmt_register_night_mode_cb(power_mgmt_night_mode_cb_t cb)
{
    s_night_mode_cb = cb;
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

    /* Disable touch as an input/wake source at night (avoid accidental
     * screen activation); re-enable it in the morning. */
    if (night) {
        gpio_intr_disable(PM_GPIO_TOUCH);
    } else {
        gpio_intr_enable(PM_GPIO_TOUCH);
    }

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
    s_wake_gpio = (uint32_t)arg;
    gpio_intr_disable(PM_GPIO_PWRKEY);
    gpio_intr_disable(PM_GPIO_BOOT);
    if (s_wake_task) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_wake_task, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void pm_wake_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* Wait for a button wake, or the periodic night-mode clock check. */
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PM_NIGHT_CHECK_MS)) == 0) {
            pm_apply_night_mode(pm_is_night_time());
            continue;
        }

        uint32_t gpio = s_wake_gpio;
        ESP_LOGI(TAG, "wake: gpio=%u", (unsigned)gpio);

        /* De-assert the latched AXP IRQ line. */
        if (gpio == PM_GPIO_PWRKEY) {
            uint32_t irq = 0;
            axp2101_get_irq_status(twatch_pmu_dev, &irq);
            axp2101_clear_irq(twatch_pmu_dev);
            ESP_LOGI(TAG, "power key: irq=0x%06lx (bit8 press, bit10 long, bit11 short)",
                     (unsigned long)irq);
        }

        /* Restore edge triggering (gpio_wakeup_enable() left these level) and
         * re-arm both button pins. */
        gpio_set_intr_type(PM_GPIO_PWRKEY, GPIO_INTR_NEGEDGE);
        gpio_set_intr_type(PM_GPIO_BOOT, GPIO_INTR_NEGEDGE);
        gpio_intr_enable(PM_GPIO_PWRKEY);
        gpio_intr_enable(PM_GPIO_BOOT);

        esp_lv_adapter_request_wake();
    }
}

static void pm_arm_gpio_wakeup(void)
{
    s_wake_gpio = 0;
    /* Touch is not a wake source in night mode (avoid accidental screen
     * activation); PWR/BOOT buttons always wake. */
    if (!s_night_mode) {
        gpio_wakeup_enable(PM_GPIO_TOUCH, GPIO_INTR_LOW_LEVEL);
    }
    gpio_wakeup_enable(PM_GPIO_PWRKEY, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable(PM_GPIO_BOOT, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}

esp_err_t power_mgmt_enter_sleep(void *ctx)
{
    (void)ctx;

    /* Skip auto-sleep while on USB power (development / charging). */
    bool vbus = false;
    if (axp2101_is_vbus_present(twatch_pmu_dev, &vbus) == ESP_OK && vbus) {
        ESP_LOGI(TAG, "on USB power, skipping auto sleep");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "entering sleep: panel SLPIN, rails off");
    co5300_sleep();

    /* Disable unused peripheral rails; keep ALDO2 (display/touch) for touch wake. */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, false);  /* SD */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO3, false);  /* LoRa */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO4, false);  /* sensor */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO1, false);  /* GNSS */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);  /* speaker */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_DLDO1, false);  /* NFC */

    pm_arm_gpio_wakeup();
    return ESP_OK;
}

esp_err_t power_mgmt_exit_sleep(void *ctx)
{
    (void)ctx;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "waking: gpio=%u cause=%d", (unsigned)s_wake_gpio, (int)cause);

    /* Restore rails. */
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO1, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO3, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_ALDO4, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO1, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);
    axp2101_enable_rail(twatch_pmu_dev, AXP2101_DLDO1, true);

    co5300_wake();
    co5300_set_brightness(s_night_mode ? PM_NIGHT_BRIGHTNESS : 0x80);

    /* Safety net: clear any pending AXP IRQ (de-asserts the GPIO7 line).
     * Detailed power-key reporting happens in pm_wake_task. */
    axp2101_clear_irq(twatch_pmu_dev);
    return ESP_OK;
}

void power_mgmt_init(void)
{
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
    xTaskCreate(pm_wake_task, "pm_wake", 2048, NULL,
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

    /* Apply the initial night-mode state (and touch-ISR state). */
    pm_apply_night_mode(pm_is_night_time());
}
