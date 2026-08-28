#include "i2c_bus.h"
#include "i2c_bus_frame.h"
#include <stdbool.h>
#include "esp_heap_caps.h"

/* Stack buffer big enough for every non-BHI260AP driver's writes today
 * (largest is pcf85063a's 7-byte time write, 8 bytes leaves headroom). */
#define I2C_BUS_SMALL_WRITE_MAX 8

esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev, uint8_t reg,
                        uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, timeout_ms);
}

esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev, uint8_t reg,
                         const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    uint8_t small[1 + I2C_BUS_SMALL_WRITE_MAX];
    uint8_t *data = small;
    bool heap = false;
    if (len > I2C_BUS_SMALL_WRITE_MAX) {
        data = heap_caps_malloc(len + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!data) {
            return ESP_ERR_NO_MEM;
        }
        heap = true;
    }

    if (!i2c_bus_build_frame(reg, buf, len, data)) {
        if (heap) {
            heap_caps_free(data);
        }
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_master_transmit(dev, data, len + 1, timeout_ms);

    if (heap) {
        heap_caps_free(data);
    }
    return err;
}
