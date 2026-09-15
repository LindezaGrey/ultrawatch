#include "brookesia_system_entry.h"

#include <memory>

#include "esp_log.h"
#include "brookesia/gui_lvgl.hpp"
#include "brookesia/service_manager/service/manager.hpp"
#include "brookesia/system_super.hpp"

namespace {

constexpr const char *TAG = "brookesia_system";
std::unique_ptr<esp_brookesia::system::super::System> system_instance;
bool started;

} // namespace

extern "C" bool twatch_brookesia_system_start(void)
{
    if (started) {
        return true;
    }

    auto &service_manager = esp_brookesia::service::ServiceManager::get_instance();
    if (!service_manager.init()) {
        ESP_LOGE(TAG, "Service manager init failed");
        return false;
    }
    if (!service_manager.start()) {
        ESP_LOGE(TAG, "Service manager start failed");
        return false;
    }

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
    const auto init_result = system_instance->init(std::move(config));
    if (!init_result) {
        ESP_LOGE(TAG, "System init failed: %s", init_result.error().c_str());
        system_instance.reset();
        return false;
    }
    const auto start_result = system_instance->start();
    if (!start_result) {
        ESP_LOGE(TAG, "System start failed: %s", start_result.error().c_str());
        system_instance.reset();
        return false;
    }

    started = true;
    ESP_LOGI(TAG, "System Super started on T-Watch Ultra 410x502");
    return true;
}
