#include "brookesia_system_entry.h"

#include <memory>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
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
    const size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    bool bound = false;
    if (manager.init() && manager.start()) {
        auto display = manager.bind("Display");
        bound = display.is_valid();
        ESP_LOGI(TAG, "SERVICE_PROBE display_bind=%s", bound ? "ok" : "failed");
        display.release();
    } else {
        ESP_LOGE(TAG, "SERVICE_PROBE manager startup failed");
    }
    manager.stop();
    manager.deinit();
    ESP_LOGI(TAG, "SERVICE_PROBE cleanup=done internal_before=%u internal_after=%u heap_ok=%u",
             static_cast<unsigned>(heap_before),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             heap_caps_check_integrity_all(true));
    return bound;
}
