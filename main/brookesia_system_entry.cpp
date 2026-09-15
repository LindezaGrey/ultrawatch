#include "brookesia_system_entry.h"

#include <memory>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "brookesia/gui_lvgl.hpp"
#include "brookesia/service_manager/service/manager.hpp"
#include "brookesia/system_super.hpp"

namespace {

constexpr const char *TAG = "brookesia_system";
std::unique_ptr<esp_brookesia::system::super::System> system_instance;
bool started;
RTC_NOINIT_ATTR uint32_t last_stage;

void cleanup_runtime_services()
{
    // DisplaySource owns the adapter started by System Super. Stop it before
    // releasing the service graph so the native fallback can become the sole
    // LVGL owner after a failed handoff.
    esp_brookesia::gui::lvgl::DisplaySource::get_instance().stop();
    const esp_err_t adapter_result = esp_lv_adapter_deinit();
    if (adapter_result != ESP_OK && adapter_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "LVGL adapter cleanup returned %s", esp_err_to_name(adapter_result));
    }
    auto &service_manager = esp_brookesia::service::ServiceManager::get_instance();
    service_manager.stop();
    service_manager.deinit();
}

} // namespace

extern "C" bool twatch_brookesia_system_start(void)
{
    if (started) {
        return true;
    }

    last_stage = 1;
    auto &service_manager = esp_brookesia::service::ServiceManager::get_instance();
    if (!service_manager.init()) {
        ESP_LOGE(TAG, "Service manager init failed");
        return false;
    }
    last_stage = 2;
    if (!service_manager.start()) {
        ESP_LOGE(TAG, "Service manager start failed");
        service_manager.deinit();
        return false;
    }

    last_stage = 3;
    esp_brookesia::gui::lvgl::DisplaySourceConfig display_config{};
    display_config.output_name.clear();
    if (!esp_brookesia::gui::lvgl::DisplaySource::get_instance().start(display_config)) {
        ESP_LOGE(TAG, "LVGL display source start failed");
        cleanup_runtime_services();
        return false;
    }

    system_instance = std::make_unique<esp_brookesia::system::super::System>();
    esp_brookesia::system::super::System::Config config;
    config.core_config.gui_backend = std::make_unique<esp_brookesia::gui::lvgl::Backend>();
    config.core_config.environment = {
        .width_px = 410,
        .height_px = 502,
        .density = 1.0F,
        .font_scale = 1.0F,
    };
    last_stage = 4;
    const auto init_result = system_instance->init(std::move(config));
    if (!init_result) {
        ESP_LOGE(TAG, "System init failed: %s", init_result.error().c_str());
        system_instance.reset();
        cleanup_runtime_services();
        return false;
    }
    last_stage = 5;
    const auto start_result = system_instance->start();
    if (!start_result) {
        ESP_LOGE(TAG, "System start failed: %s", start_result.error().c_str());
        system_instance.reset();
        cleanup_runtime_services();
        return false;
    }

    started = true;
    last_stage = 0;
    ESP_LOGI(TAG, "System Super started on T-Watch Ultra 410x502");
    return true;
}

extern "C" uint32_t twatch_brookesia_system_last_stage(void)
{
    return last_stage;
}

extern "C" bool twatch_brookesia_services_probe(void)
{
    // Run before native LVGL starts reading touch; release Display and its
    // Storage dependency and stop scheduler workers before handing ownership back.
    auto &manager = esp_brookesia::service::ServiceManager::get_instance();
    bool all_bound = true;
    for (unsigned cycle = 1; cycle <= 3; ++cycle) {
        const size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const unsigned tasks_before = uxTaskGetNumberOfTasks();
        bool bound = false;
        if (manager.init() && manager.start()) {
            auto display = manager.bind("Display");
            bound = display.is_valid();
            display.release();
        }
        manager.stop();
        manager.deinit();
        const size_t immediate = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const unsigned tasks_immediate = uxTaskGetNumberOfTasks();
        // Diagnostic sampling interval only: allow idle-task reclamation to run.
        // This is not a lifecycle synchronization mechanism or a proposed fix.
        vTaskDelay(pdMS_TO_TICKS(100));
        const bool healthy = heap_caps_check_integrity_all(true);
        ESP_LOGI(TAG, "SERVICE_CYCLE cycle=%u bind=%s before=%u immediate=%u settled=%u "
                      "tasks=%u/%u/%u heap_ok=%u",
                 cycle, bound ? "ok" : "failed", static_cast<unsigned>(heap_before),
                 static_cast<unsigned>(immediate),
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                 tasks_before, tasks_immediate, static_cast<unsigned>(uxTaskGetNumberOfTasks()), healthy);
        all_bound = all_bound && bound && healthy;
        if (!all_bound) break;
    }
    return all_bound;
}
