#include <inttypes.h>
#include <stdlib.h>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "twatch_board.h"
#include "twatch_display.h"
#include "esp_timer.h"
#include "battery_observation.h"
#include "lvgl_app.h"
#include "twatch_brookesia_bridge.h"
#include "bhi260ap_core.h"

static const char *TAG = "phase1";
enum { EXPECTED_FLASH_BYTES = 16 * 1024 * 1024, EXPECTED_PSRAM_BYTES = 8 * 1024 * 1024 };
#define DISPLAY_GEOMETRY_TEST 0
#define DISPLAY_INVERTED 1

static void require_integrity(void)
{
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "FAIL heap integrity");
        abort();
    }
}

void app_main(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_reset_reason_t boot_reason = esp_reset_reason();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "BOOT board=T-Watch-Ultra-V1.0 radio_variant=SX1262 app=%s idf=%s",
             app->version, esp_get_idf_version());
    ESP_LOGI(TAG, "reset_reason=%d chip_model=%d revision=%u cores=%u",
             (int)boot_reason, (int)chip.model, chip.revision, chip.cores);

    uint32_t flash_bytes = 0;
    ESP_ERROR_CHECK(esp_flash_get_physical_size(NULL, &flash_bytes));
    const size_t psram_bytes = esp_psram_is_initialized() ? esp_psram_get_size() : 0;
    ESP_LOGI(TAG, "flash_bytes=%" PRIu32 " psram_bytes=%zu", flash_bytes, psram_bytes);
    if (chip.model != CHIP_ESP32S3 || flash_bytes != EXPECTED_FLASH_BYTES ||
        psram_bytes != EXPECTED_PSRAM_BYTES) {
        ESP_LOGE(TAG, "FAIL expected ESP32-S3, 16 MB flash, 8 MB PSRAM");
        abort();
    }
    require_integrity();
    ESP_ERROR_CHECK(twatch_board_init());
    ESP_ERROR_CHECK(twatch_board_prepare_bhi());
    ESP_ERROR_CHECK(twatch_board_prepare_display_touch());
    bool bhi_core_product_ok = false;
    uint8_t bhi_core_product_id = 0;
    ESP_ERROR_CHECK(bhi260ap_core_probe(&bhi_core_product_ok, &bhi_core_product_id));
    if (!bhi_core_product_ok) {
        ESP_LOGE(TAG, "BHI260AP Bosch core product ID mismatch: 0x%02x", bhi_core_product_id);
        abort();
    }
    uint8_t touch_address = 0;
    bool touch_present = false;
    ESP_ERROR_CHECK(twatch_board_probe_touch(&touch_address, &touch_present));
    if (touch_present) {
        esp_err_t touch_info_err = twatch_board_read_touch_info();
        if (touch_info_err != ESP_OK) {
            ESP_LOGW(TAG, "Touch identity read did not validate: %s", esp_err_to_name(touch_info_err));
        }
    }
    bool rtc_present = false;
    ESP_ERROR_CHECK(twatch_board_probe_rtc(&rtc_present));
    if (rtc_present) {
        twatch_rtc_snapshot_t rtc_snapshot;
        ESP_ERROR_CHECK(twatch_board_read_rtc(&rtc_snapshot));
        vTaskDelay(pdMS_TO_TICKS(2000));
        twatch_rtc_snapshot_t rtc_followup;
        ESP_ERROR_CHECK(twatch_board_read_rtc(&rtc_followup));
        ESP_LOGI(TAG, "RTC_TICK_CHECK first=%02x/%02x/%02x followup=%02x/%02x/%02x",
                 rtc_snapshot.hours, rtc_snapshot.minutes, rtc_snapshot.seconds,
                 rtc_followup.hours, rtc_followup.minutes, rtc_followup.seconds);
    }
    ESP_ERROR_CHECK(twatch_display_init());
#if DISPLAY_INVERTED
    ESP_ERROR_CHECK(twatch_display_set_inverted(true));
#endif
#if DISPLAY_GEOMETRY_TEST
    ESP_ERROR_CHECK(twatch_display_geometry_test());
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    for (;;) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
    ESP_ERROR_CHECK(twatch_brookesia_bridge_start() ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(lvgl_app_init());
    // IDF initializes TWDT and subscribes both idle tasks through sdkconfig.
    // The existing main task owns these checks; no extra task/ISR is needed.
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    ESP_LOGI(TAG, "READY heap_integrity=ok main_wdt=subscribed timeout_s=%d",
             CONFIG_ESP_TASK_WDT_TIMEOUT_S);
    battery_observation_t battery_run = {0};
    uint64_t next_telemetry_ms = 0;
    for (uint32_t sample = 0;; ++sample) {
        require_integrity();
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        lvgl_app_tick();
        const uint64_t now_ms = (uint64_t)esp_timer_get_time() / 1000;
        if (now_ms < next_telemetry_ms) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        next_telemetry_ms = now_ms + 5000;
        twatch_power_snapshot_t power;
        ESP_ERROR_CHECK(twatch_board_read_power(&power));
        twatch_rail_snapshot_t rails;
        ESP_ERROR_CHECK(twatch_board_read_rails(&rails));
        ESP_LOGI("phase2", "RAILS dcdc=%02x dc1_raw=%02x ldo=%02x/%02x aldo2_raw=%02x "
                 "dc1_on=%u dc1_valid=%u dc1_config_mv=%u aldo2_on=%u aldo2_valid=%u aldo2_config_mv=%u",
                 rails.dcdc_control, rails.dc1_voltage, rails.ldo_control[0], rails.ldo_control[1],
                 rails.aldo2_voltage, rails.dc1_enabled, rails.dc1_setting_valid, rails.dc1_configured_mv,
                 rails.aldo2_enabled, rails.aldo2_setting_valid, rails.aldo2_configured_mv);
        ESP_LOGI("phase2", "XL_CANDIDATE addr=20 out=%02x/%02x polarity=%02x/%02x cfg=%02x/%02x "
                 "P07_output=%u P07_latch=%u P10_output=%u P10_latch=%u",
                 rails.expander_output[0], rails.expander_output[1],
                 rails.expander_polarity[0], rails.expander_polarity[1],
                 rails.expander_config[0], rails.expander_config[1],
                 rails.display_enable_output, rails.display_enable_latch_high,
                 rails.touch_reset_output, rails.touch_reset_latch_high);
        battery_observe(&battery_run, (uint64_t)esp_timer_get_time() / 1000,
                        power.usb_present, power.battery_present, power.charging,
                        power.battery_voltage_available, power.battery_mv);
        ESP_LOGI("phase2", "BATTERY_RUN samples=%" PRIu32 " span_ms=%" PRIu64
                 " first_ms=%" PRIu64 " last_ms=%" PRIu64 " voltage_samples=%" PRIu32
                 " min_mv=%u max_mv=%u charging_samples=%" PRIu32 " active=%u",
                 battery_run.samples, battery_run.last_ms - battery_run.first_ms,
                 battery_run.first_ms, battery_run.last_ms, battery_run.voltage_samples,
                 battery_run.min_mv, battery_run.max_mv, battery_run.charging_samples, battery_run.active);
        ESP_LOGI("phase2", "PMU status=%02x/%02x adc=%02x usb=%u battery_present=%u "
                 "charging=%u charger_state=%u irq=%02x/%02x/%02x off_en=%02x",
                 power.status1, power.status2, power.adc_control, power.usb_present,
                 power.battery_present, power.charging, power.charger_state,
                 power.irq_status[0], power.irq_status[1], power.irq_status[2], power.poweroff_enable);
        if (power.battery_voltage_available) {
            ESP_LOGI("phase2", "battery_mv=%u", power.battery_mv);
        } else {
            ESP_LOGI("phase2", "battery_mv=unavailable (battery absent or ADC disabled)");
        }
        // USB may enumerate after startup logs have passed. Repeat only facts
        // established on this boot; this line is reached after all fatal checks.
        ESP_LOGI(TAG, "BOOT_EVIDENCE app=%s idf=%s reset_reason=%d "
                 "flash_bytes=%" PRIu32 " psram_bytes=%zu psram_memtest_config=%d "
                 "startup_checks=ok main_wdt=subscribed",
                 app->version, esp_get_idf_version(), (int)boot_reason,
                 flash_bytes, psram_bytes, CONFIG_SPIRAM_MEMTEST);
        ESP_LOGI(TAG, "HEALTH sample=%" PRIu32 " internal_free=%zu internal_min=%zu "
                 "psram_free=%zu stack_free_bytes=%u heap_integrity=ok",
                 sample, heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
