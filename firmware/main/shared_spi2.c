#include "shared_spi2.h"

#include "board.h"

#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "shared_spi2";
static SemaphoreHandle_t bus_lock;
static unsigned client_count;
static bool bus_ready;

static esp_err_t i2c_read(uint8_t address, uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(BOARD_I2C_PORT, address, &reg, 1,
                                        value, 1, portMAX_DELAY);
}

static esp_err_t i2c_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(BOARD_I2C_PORT, address, data,
                                      sizeof(data), portMAX_DELAY);
}

static esp_err_t set_shared_rails(bool enabled)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(i2c_read(BOARD_AXP2101_ADDR,
                                 BOARD_AXP2101_LDO_ENABLE, &value),
                        TAG, "rail state read failed");
    if (enabled) {
        uint8_t voltage;
        ESP_RETURN_ON_ERROR(i2c_read(BOARD_AXP2101_ADDR,
                                     BOARD_AXP2101_ALDO1_VOLTAGE, &voltage),
                            TAG, "ALDO1 voltage read failed");
        voltage = (voltage & 0xe0) | 28; /* 3.3 V */
        ESP_RETURN_ON_ERROR(i2c_write(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_ALDO1_VOLTAGE, voltage),
                            TAG, "ALDO1 voltage write failed");
        ESP_RETURN_ON_ERROR(i2c_read(BOARD_AXP2101_ADDR,
                                     BOARD_AXP2101_ALDO3_VOLTAGE, &voltage),
                            TAG, "ALDO3 voltage read failed");
        voltage = (voltage & 0xe0) | 28; /* 3.3 V */
        ESP_RETURN_ON_ERROR(i2c_write(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_ALDO3_VOLTAGE, voltage),
                            TAG, "ALDO3 voltage write failed");
        value |= (1U << BOARD_AXP2101_ALDO1_BIT) |
                 (1U << BOARD_AXP2101_ALDO3_BIT);
    } else {
        value &= ~((1U << BOARD_AXP2101_ALDO1_BIT) |
                   (1U << BOARD_AXP2101_ALDO3_BIT));
    }
    return i2c_write(BOARD_AXP2101_ADDR, BOARD_AXP2101_LDO_ENABLE, value);
}

esp_err_t shared_spi2_acquire(void)
{
    if (bus_lock == NULL) {
        bus_lock = xSemaphoreCreateMutex();
        if (bus_lock == NULL) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(bus_lock, portMAX_DELAY);
    if (client_count > 0) {
        client_count++;
        xSemaphoreGive(bus_lock);
        return ESP_OK;
    }

    esp_err_t result = set_shared_rails(true);
    if (result == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const spi_bus_config_t bus = {
            .mosi_io_num = BOARD_SD_MOSI,
            .miso_io_num = BOARD_SD_MISO,
            .sclk_io_num = BOARD_SD_SCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096,
        };
        result = spi_bus_initialize(BOARD_SD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    }
    if (result == ESP_OK) {
        bus_ready = true;
        client_count = 1;
    } else {
        ESP_ERROR_CHECK_WITHOUT_ABORT(set_shared_rails(false));
    }
    xSemaphoreGive(bus_lock);
    return result;
}

void shared_spi2_release(void)
{
    if (bus_lock == NULL) return;
    xSemaphoreTake(bus_lock, portMAX_DELAY);
    if (client_count > 0) client_count--;
    if (client_count == 0) {
        if (bus_ready) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_free(BOARD_SD_SPI_HOST));
        }
        bus_ready = false;
        ESP_ERROR_CHECK_WITHOUT_ABORT(set_shared_rails(false));
    }
    xSemaphoreGive(bus_lock);
}
