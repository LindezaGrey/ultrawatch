#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "brookesia/hal_interface/device.hpp"
#include "brookesia/hal_interface/interface.hpp"
#include "brookesia/hal_interface/interfaces/display/panel.hpp"
#include "brookesia/hal_interface/interfaces/display/touch.hpp"
#include "twatch_board.h"
#include "twatch_display.h"
#include "twatch_brookesia_bridge.h"

namespace {

using esp_brookesia::hal::Device;
using esp_brookesia::hal::InterfaceSpec;
using esp_brookesia::hal::display::PanelIface;
using esp_brookesia::hal::display::TouchIface;

constexpr const char *DEVICE_NAME = "TWatchUltra";
constexpr const char *PANEL_NAME = "TWatchUltra:CO5300";
constexpr const char *TOUCH_NAME = "TWatchUltra:CST9220";
constexpr const char *GROUP_ID = "t_watch_ultra_display";

esp_brookesia::hal::InterfaceHandle<PanelIface> panel_handle;
esp_brookesia::hal::InterfaceHandle<TouchIface> touch_handle;

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
        /* The panel uses the validated custom QSPI transaction path. Brookesia's
         * LVGL adapter still needs the bus classification and alignment metadata;
         * its flush callback is replaced below with draw_bitmap(). */
        specific = DriverSpecific{
            .io_handle = nullptr,
            .panel_handle = nullptr,
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
        if (twatch_board_read_touch(&sample) != ESP_OK) {
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
        };
    }

    bool on_init() override
    {
        interfaces_.emplace(PANEL_NAME, std::make_shared<Panel>());
        interfaces_.emplace(TOUCH_NAME, std::make_shared<Touch>());
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
