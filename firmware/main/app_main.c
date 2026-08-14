#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "esp_console.h"

#include "bsp/bsp_i2c.h"
#include "bsp/bsp_axp2101.h"
#include "bsp/bsp_xl9555.h"
#include "bsp/bsp_display.h"
#include "bsp/bsp_cst9217.h"
#include "bsp/bsp_pcf85063.h"
#include "bsp/bsp_sdcard.h"
#include "ui/ui_clock.h"
#include "cmd_time.h"
#include "power.h"
#include "powerlog.h"

static const char *TAG = "app_main";

static void step_init(const char *name, esp_err_t err, bool fatal)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s: OK", name);
        return;
    }
    ESP_LOGE(TAG, "%s: FAILED (%s)", name, esp_err_to_name(err));
    if (fatal) {
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot start");

    step_init("nvs", nvs_flash_init(), true);

    step_init("i2c", bsp_i2c_init(), true);
    step_init("axp2101", bsp_axp2101_init(), true);
    step_init("xl9555", bsp_xl9555_init(), true);
    step_init("display", bsp_display_init(), true);

    esp_err_t err = bsp_display_set_brightness(UI_DAY_MODE_BRIGHTNESS_PCT);
    ESP_LOGI(TAG, "brightness: %s", err == ESP_OK ? "OK" : esp_err_to_name(err));

    /* Quick pattern to verify orientation/rotation during bring-up */
    bsp_display_show_test_pattern();
    ESP_LOGI(TAG, "test pattern shown");
    vTaskDelay(pdMS_TO_TICKS(1500));

    step_init("touch", bsp_touch_init(), false);
    step_init("rtc", bsp_rtc_init(), false);

    struct tm tm;
    if (bsp_rtc_get_time(&tm) != ESP_OK) {
        ESP_LOGW(TAG, "RTC lost time, setting a default (2026-01-01 12:00)");
        struct tm def = {
            .tm_sec = 0, .tm_min = 0, .tm_hour = 12,
            .tm_mday = 1, .tm_mon = 0, .tm_year = 126, .tm_wday = 4,
        };
        bsp_rtc_set_time(&def);
    }

    step_init("ui", ui_clock_init(), false);

    /* SD card (auto-mount; fails silently when no card is inserted) */
    step_init("sd", bsp_sd_init(), false);
    step_init("powerlog", powerlog_init(), false);
    step_init("power", power_sleep_init(), false);

    /* Interactive console over USB-Serial-JTAG: `settime` / `time` */
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.max_cmdline_length = 128;
    esp_console_dev_usb_serial_jtag_config_t hw_cfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_console_repl_t *repl = NULL;
    if (esp_console_new_repl_usb_serial_jtag(&hw_cfg, &repl_cfg, &repl) == ESP_OK) {
        cmd_time_register();
        esp_console_register_help_command();
        ESP_LOGI(TAG, "console repl started (settime/time)");
        esp_console_start_repl(repl);
        /* NOTE: do NOT set stdout/stderr to O_NONBLOCK via fcntl. The
         * USB-Serial-JTAG VFS shares ONE non_blocking flag across stdin,
         * stdout and stderr, so doing so silently makes stdin non-blocking
         * too. linenoise then gets EWOULDBLOCK on every read, returns NULL
         * and the REPL spins printing "esp> " at ~100 lines/s. In blocking
         * mode the driver write path blocks at most ~50 ms per byte (then
         * drops), so a full TX buffer (host not reading) can never stall
         * the app tasks permanently. */
    } else {
        ESP_LOGW(TAG, "console repl failed to start");
    }

    uint32_t ticks = 0;
    while (1) {
        uint32_t sleep_ms = lv_timer_handler();

        /* Power engine may enter light sleep here and sleep until the next
         * minute boundary or a touch. When it just woke up, re-run the LVGL
         * tick immediately so the display reflects the new time. */
        if (power_sleep_tick()) {
            continue;
        }

        if (sleep_ms == LV_NO_TIMER_READY) {
            sleep_ms = 1000;
        }
        vTaskDelay(pdMS_TO_TICKS(sleep_ms > 0 ? sleep_ms : 5));

        if ((++ticks % 50) == 0) {
            ESP_LOGI(TAG, "heartbeat: bat=%d%% %dmV", bsp_axp2101_battery_percent(),
                     bsp_axp2101_battery_voltage_mv());
        }
    }
}
