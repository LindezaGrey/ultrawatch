#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "brookesia/hal_interface/device.hpp"
#include "brookesia/hal_interface/interface.hpp"
#include "brookesia/hal_interface/interfaces/display/panel.hpp"
#include "brookesia/hal_interface/interfaces/display/touch.hpp"
#include "brookesia/hal_interface/interfaces/storage/file_system.hpp"
#include "twatch_board.h"
#include "key_value_nvs_impl.hpp"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "twatch_display.h"
#include "twatch_brookesia_bridge.h"

namespace {

using esp_brookesia::hal::Device;
using esp_brookesia::hal::InterfaceSpec;
using esp_brookesia::hal::display::PanelIface;
using esp_brookesia::hal::display::TouchIface;
using esp_brookesia::hal::storage::FileSystemIface;

constexpr const char *DEVICE_NAME = "TWatchUltra";
constexpr const char *PANEL_NAME = "TWatchUltra:CO5300";
constexpr const char *TOUCH_NAME = "TWatchUltra:CST9220";
constexpr const char *GROUP_ID = "t_watch_ultra_display";
constexpr const char *FILESYSTEM_NAME = "TWatchUltra:SPIFFS";
constexpr const char *FILESYSTEM_LABEL = "storage";
// Keep the upstream Brookesia runtime namespace while using ESP-IDF SPIFFS.
constexpr const char *FILESYSTEM_BASE = "/littlefs";

esp_brookesia::hal::InterfaceHandle<PanelIface> panel_handle;
esp_brookesia::hal::InterfaceHandle<TouchIface> touch_handle;

class Spiffs final : public FileSystemIface {
public:
    Spiffs()
    {
        const esp_vfs_spiffs_conf_t config = {
            .base_path = FILESYSTEM_BASE,
            .partition_label = FILESYSTEM_LABEL,
            .max_files = 8,
            // The new labeled partition is blank on first deployment; once
            // valid, subsequent mounts preserve its contents.
            .format_if_mount_failed = true,
        };
        const esp_err_t err = esp_vfs_spiffs_register(&config);
        if (err != ESP_OK) {
            ESP_LOGE("TWatchStorage", "SPIFFS mount failed: %s", esp_err_to_name(err));
            return;
        }
        info_list_.push_back(Info{
            .fs_type = FileSystemType::SPIFFS,
            .medium_type = MediumType::Flash,
            .mount_point = FILESYSTEM_BASE,
            .root_path = FILESYSTEM_BASE,
            .supports_directories = false,
        });
        ESP_LOGI("TWatchStorage", "SPIFFS mount=ok base=%s label=%s",
                 FILESYSTEM_BASE, FILESYSTEM_LABEL);
    }

    ~Spiffs() override
    {
        if (!info_list_.empty()) {
            esp_vfs_spiffs_unregister(FILESYSTEM_LABEL);
        }
    }

    bool get_capacity(const char *mount_point, Capacity &capacity) override
    {
        if (mount_point == nullptr || std::string_view(mount_point) != FILESYSTEM_BASE ||
            info_list_.empty()) {
            return false;
        }
        size_t total = 0;
        size_t used = 0;
        if (esp_spiffs_info(FILESYSTEM_LABEL, &total, &used) != ESP_OK) {
            return false;
        }
        capacity.total_bytes = total;
        capacity.used_bytes = used;
        capacity.free_bytes = total - used;
        return true;
    }
};

class Panel final : public PanelIface {
public:
    Panel()
        : PanelIface(Info{
              .h_res = 410,
              .v_res = 502,
              .pixel_format = PixelFormat::RGB565,
              .group_id = GROUP_ID,
          })
    {
    }

    bool draw_bitmap(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2,
                     const uint8_t *data) override
    {
        if (data == nullptr || x2 <= x1 || y2 <= y1 || x2 > 410 || y2 > 502) {
            return false;
        }
        return twatch_display_flush(static_cast<uint16_t>(x1), static_cast<uint16_t>(y1),
                                    static_cast<uint16_t>(x2 - x1), static_cast<uint16_t>(y2 - y1),
                                    data) == ESP_OK;
    }

    bool get_driver_specific(DriverSpecific &specific) override
    {
        /* Borrowed ESP LCD wrappers share the validated synchronous QSPI path.
         * The application adapter supplies task-context completion and the
         * watch rounder. DisplaySource event bridging is not enabled yet. */
        specific = DriverSpecific{
            .io_handle = twatch_display_get_panel_io(),
            .panel_handle = twatch_display_get_panel(),
            .bus_type = BusType::Generic,
            .frame_buffer_count = 1,
            .draw_x_align_bytes = 1,
            .draw_y_align_bytes = 1,
            .event_dispatcher = {},
        };
        return true;
    }
};

class Touch final : public TouchIface {
public:
    Touch()
        : TouchIface(Info{
              .x_max = 410,
              .y_max = 502,
              .max_points = 1,
              .operation_mode = OperationMode::Polling,
              .group_id = GROUP_ID,
          })
    {
    }

    bool read_points(std::vector<Point> &points) override
    {
        twatch_touch_sample_t sample{};
        const esp_err_t result = twatch_board_read_touch(&sample);
        if (result == ESP_ERR_INVALID_RESPONSE) {
            // CST92xx emits a non-touch idle/status packet while no finger is
            // present. Treat it as an empty sample; returning false makes the
            // upstream polling task allocate a formatted warning on every
            // poll and can exhaust the service task heap.
            points.clear();
            return true;
        }
        if (result != ESP_OK) {
            return false;
        }
        points.clear();
        if (sample.pressed) {
            points.push_back(Point{
                .x = static_cast<int16_t>(sample.x),
                .y = static_cast<int16_t>(sample.y),
                .pressure = 1,
                .track_id = 0,
            });
        }
        return true;
    }

    bool register_interrupt_handler(InterruptHandler handler, void *ctx) override
    {
        (void)ctx;
        /* The verified board path is polling; no interrupt line is claimed here. */
        return handler == nullptr;
    }
};

class DeviceImpl final : public Device {
public:
    DeviceImpl()
        : Device(std::string(DEVICE_NAME))
    {
    }

private:
    bool probe() override { return true; }

    std::vector<InterfaceSpec> get_interface_specs() const override
    {
        return {
            {PanelIface::NAME, PANEL_NAME},
            {TouchIface::NAME, TOUCH_NAME},
            {esp_brookesia::hal::storage::KeyValueIface::NAME, "TWatchUltra:NVS"},
            {FileSystemIface::NAME, FILESYSTEM_NAME},
        };
    }

    bool on_init() override
    {
        interfaces_.emplace(PANEL_NAME, std::make_shared<Panel>());
        interfaces_.emplace(TOUCH_NAME, std::make_shared<Touch>());
        interfaces_.emplace("TWatchUltra:NVS",
                            std::make_shared<esp_brookesia::hal::StorageKeyValueNvsImpl>());
        interfaces_.emplace(FILESYSTEM_NAME, std::make_shared<Spiffs>());
        return true;
    }

    void on_deinit() override { interfaces_.clear(); }
};

} // namespace

BROOKESIA_PLUGIN_REGISTER(esp_brookesia::hal::Device, DeviceImpl, std::string(DEVICE_NAME));

extern "C" bool twatch_brookesia_bridge_ready(void)
{
    return twatch_brookesia_bridge_start();
}

extern "C" bool twatch_brookesia_bridge_start(void)
{
    if (!panel_handle) panel_handle = esp_brookesia::hal::acquire_interface<PanelIface>(PANEL_NAME);
    if (!touch_handle) touch_handle = esp_brookesia::hal::acquire_interface<TouchIface>(TOUCH_NAME);
    static bool storage_probed;
    if (!storage_probed) {
        storage_probed = true;
        auto kv = esp_brookesia::hal::acquire_interface<
            esp_brookesia::hal::storage::KeyValueIface>("TWatchUltra:NVS");
        if (!kv || !kv->init()) {
            ESP_LOGE("TWatchStorage", "STORAGE_PROBE init failed: %s",
                     kv ? kv->get_last_error().c_str() : "interface unavailable");
        } else {
            std::vector<esp_brookesia::hal::storage::KeyValueIface::EntryInfo> entries;
            const bool listed = kv->list("uw_probe", entries);
            ESP_LOGI("TWatchStorage", "STORAGE_PROBE init=ok list=%s entries=%u kv_writes=0",
                     listed ? "ok" : "failed", static_cast<unsigned>(entries.size()));
            if (!listed) ESP_LOGE("TWatchStorage", "%s", kv->get_last_error().c_str());
            kv->deinit();
        }
    }
    return static_cast<bool>(panel_handle) && static_cast<bool>(touch_handle);
}

extern "C" bool twatch_brookesia_bridge_draw(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2,
                                               const uint8_t *data)
{
    return panel_handle && panel_handle->draw_bitmap(x1, y1, x2, y2, data);
}

extern "C" bool twatch_brookesia_bridge_read_touch(twatch_touch_sample_t *sample)
{
    if (!touch_handle || sample == nullptr) return false;
    std::vector<TouchIface::Point> points;
    if (!touch_handle->read_points(points)) return false;
    sample->pressed = !points.empty();
    if (sample->pressed) {
        sample->x = static_cast<uint16_t>(points.front().x);
        sample->y = static_cast<uint16_t>(points.front().y);
    }
    return true;
}
