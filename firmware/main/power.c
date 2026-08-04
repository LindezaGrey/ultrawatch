#include "power.h"

#include "bsp_twatch_ultra.h"
#include "bsp_display.h"
#include "bsp_pcf85063.h"
#include "ui/ui_clock.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"

#include "lvgl.h"

static const char *TAG = "power";

/* Stay awake this long after the last touch so interaction works */
#define POWER_AWAKE_MS          10000
/* Panel brightness during sleep (clamped to the user brightness) */
#define POWER_SLEEP_BRIGHTNESS  20
/* Min timer wake so we never re-sleep-thrash right at a minute boundary */
#define POWER_MIN_WAKE_MS       1000

static bool s_enabled = true;
static bool s_interactive;
static uint32_t s_last_interact_ms;
static uint8_t s_user_brightness = UI_DAY_MODE_BRIGHTNESS_PCT;

static uint32_t s_sleeps;
static uint32_t s_wake_timer;
static uint32_t s_wake_ext1;

esp_err_t power_sleep_init(void)
{
    ESP_LOGI(TAG, "power engine ready (sleep enabled)");
    return ESP_OK;
}

void power_set_sleep_enabled(bool enable)
{
    s_enabled = enable;
    if (enable) {
        s_interactive = true;
        s_last_interact_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    }
    ESP_LOGI(TAG, "sleep %s", enable ? "on" : "off");
}

bool power_get_sleep_enabled(void)
{
    return s_enabled;
}

void power_set_interactive(void)
{
    s_interactive = true;
    s_last_interact_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void power_set_user_brightness(uint8_t percent)
{
    s_user_brightness = percent > 100 ? 100 : percent;
}

void power_get_stats(uint32_t *sleeps, uint32_t *wake_timer, uint32_t *wake_ext1)
{
    if (sleeps) {
        *sleeps = s_sleeps;
    }
    if (wake_timer) {
        *wake_timer = s_wake_timer;
    }
    if (wake_ext1) {
        *wake_ext1 = s_wake_ext1;
    }
}

bool power_sleep_tick(void)
{
    if (!s_enabled) {
        return false;
    }
    /* While a USB host is attached (debugging / charging) stay fully awake so
     * the console keeps working. The USB-Serial-JTAG VFS is also the reason we
     * avoid sleeping: it would re-enumerate on every wake. */
    if (usb_serial_jtag_is_connected()) {
        return false;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (s_interactive) {
        if (now_ms - s_last_interact_ms < POWER_AWAKE_MS) {
            return false;
        }
        s_interactive = false;
    }

    /* Dim the panel for the sleep window (AMOLED power ~ brightness) */
    uint8_t dim = s_user_brightness < POWER_SLEEP_BRIGHTNESS ? s_user_brightness
                                                             : POWER_SLEEP_BRIGHTNESS;
    if (dim < s_user_brightness) {
        bsp_display_set_brightness(dim);
    }

    uint32_t ms_to_next = 60 * 1000;
    struct tm tm;
    if (bsp_rtc_get_time(&tm) == ESP_OK) {
        ms_to_next = (uint32_t)(60 - tm.tm_sec) * 1000;
        if (ms_to_next < POWER_MIN_WAKE_MS) {
            ms_to_next = POWER_MIN_WAKE_MS;
        }
    }
    esp_sleep_enable_timer_wakeup((uint64_t)ms_to_next * 1000ULL);
    esp_sleep_enable_ext1_wakeup_io(1ULL << BSP_TOUCH_INT_PIN, ESP_EXT1_WAKEUP_ANY_LOW);

    esp_light_sleep_start();
    s_sleeps++;

    uint32_t causes = esp_sleep_get_wakeup_causes();
    if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        s_wake_timer++;
    } else if (causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) {
        s_wake_ext1++;
        power_set_interactive();
    } else if (!(causes & BIT(ESP_SLEEP_WAKEUP_UNDEFINED))) {
        ESP_LOGI(TAG, "wake causes 0x%lx", (unsigned long)causes);
    }

    /* Restore user brightness and force the LVGL tick to re-render */
    if (dim < s_user_brightness) {
        bsp_display_set_brightness(s_user_brightness);
    }
    lv_obj_invalidate(lv_screen_active());
    return true;
}
