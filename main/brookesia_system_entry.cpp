#include "brookesia_system_entry.h"

#include <memory>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_heap_trace.h"
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
        return false;
    }

    last_stage = 3;
    esp_brookesia::gui::lvgl::DisplaySourceConfig display_config{};
    display_config.output_name.clear();
    if (!esp_brookesia::gui::lvgl::DisplaySource::get_instance().start(display_config)) {
        ESP_LOGE(TAG, "LVGL display source start failed");
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
        return false;
    }
    last_stage = 5;
    const auto start_result = system_instance->start();
    if (!start_result) {
        ESP_LOGE(TAG, "System start failed: %s", start_result.error().c_str());
        system_instance.reset();
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
    // Temporary diagnostic buffer in PSRAM to preserve internal heap headroom.
    // ESP-IDF excludes ISR allocations when trace records live in external RAM.
    constexpr size_t record_count = 2048;
    auto *records = static_cast<heap_trace_record_t *>(heap_caps_calloc(
        record_count, sizeof(heap_trace_record_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    bool trace_ready = records && heap_trace_init_standalone(records, record_count) == ESP_OK;
    if (!trace_ready) ESP_LOGW(TAG, "SERVICE_TRACE unavailable");
    bool all_bound = true;
    for (unsigned cycle = 1; cycle <= 3; ++cycle) {
        const size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const unsigned tasks_before = uxTaskGetNumberOfTasks();
        const bool tracing = trace_ready && cycle == 1 && heap_trace_start(HEAP_TRACE_LEAKS) == ESP_OK;
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
        if (tracing) {
            ESP_ERROR_CHECK(heap_trace_stop());
            heap_trace_summary_t summary{};
            ESP_ERROR_CHECK(heap_trace_summary(&summary));
            ESP_LOGI(TAG, "SERVICE_TRACE count=%u capacity=%u peak=%u overflow=%u isr_excluded=1",
                     static_cast<unsigned>(summary.count), static_cast<unsigned>(summary.capacity),
                     static_cast<unsigned>(summary.high_water_mark),
                     static_cast<unsigned>(summary.has_overflowed));
            heap_trace_dump_caps(MALLOC_CAP_INTERNAL);
        }
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
    if (trace_ready) ESP_ERROR_CHECK(heap_trace_init_standalone(nullptr, 0));
    heap_caps_free(records);
    return all_bound;
}
