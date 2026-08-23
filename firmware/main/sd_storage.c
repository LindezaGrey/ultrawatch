#include "sd_storage.h"

#include "board.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "sd_storage";
static SemaphoreHandle_t storage_lock;
static sdmmc_card_t *mounted_card;
static unsigned client_count;
static bool bus_ready;
static bool mounted;

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

static esp_err_t set_power(bool enabled)
{
    uint8_t enable;
    ESP_RETURN_ON_ERROR(i2c_read(BOARD_AXP2101_ADDR,
                                  BOARD_AXP2101_LDO_ENABLE, &enable),
                        TAG, "ALDO1 state read failed");
    if (enabled) {
        uint8_t voltage;
        ESP_RETURN_ON_ERROR(i2c_read(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_ALDO1_VOLTAGE, &voltage),
                            TAG, "ALDO1 voltage read failed");
        voltage = (voltage & 0xe0) | 28;
        ESP_RETURN_ON_ERROR(i2c_write(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_ALDO1_VOLTAGE, voltage),
                            TAG, "ALDO1 voltage write failed");
        enable |= 1U << BOARD_AXP2101_ALDO1_BIT;
    } else {
        enable &= ~(1U << BOARD_AXP2101_ALDO1_BIT);
    }
    return i2c_write(BOARD_AXP2101_ADDR, BOARD_AXP2101_LDO_ENABLE, enable);
}

bool sd_storage_card_present(void)
{
    uint8_t config;
    uint8_t input;
    if (i2c_read(BOARD_XL9555_ADDR, BOARD_XL9555_CONFIG1, &config) != ESP_OK) {
        return false;
    }
    config |= 1U << BOARD_XL9555_SD_DETECT_BIT;
    if (i2c_write(BOARD_XL9555_ADDR, BOARD_XL9555_CONFIG1, config) != ESP_OK ||
        i2c_read(BOARD_XL9555_ADDR, BOARD_XL9555_INPUT1, &input) != ESP_OK) {
        return false;
    }
    return (input & (1U << BOARD_XL9555_SD_DETECT_BIT)) == 0;
}

static void close_storage(void)
{
    if (mounted) {
        esp_err_t result = esp_vfs_fat_sdcard_unmount("/sdcard", mounted_card);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "SD unmount failed: %s", esp_err_to_name(result));
        }
    }
    mounted = false;
    mounted_card = NULL;
    if (bus_ready) {
        esp_err_t result = spi_bus_free(BOARD_SD_SPI_HOST);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "SD SPI release failed: %s", esp_err_to_name(result));
        }
    }
    bus_ready = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(set_power(false));
}

esp_err_t sd_storage_acquire(void)
{
    if (storage_lock == NULL) {
        storage_lock = xSemaphoreCreateMutex();
        if (storage_lock == NULL) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(storage_lock, portMAX_DELAY);
    if (mounted) {
        client_count++;
        xSemaphoreGive(storage_lock);
        return ESP_OK;
    }

    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << BOARD_NFC_CS) | (1ULL << BOARD_LORA_CS) |
                        (1ULL << BOARD_LORA_RESET),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t result = gpio_config(&outputs);
    if (result == ESP_OK) result = gpio_set_level(BOARD_NFC_CS, 1);
    if (result == ESP_OK) result = gpio_set_level(BOARD_LORA_CS, 1);
    if (result == ESP_OK) result = gpio_set_level(BOARD_LORA_RESET, 1);
    if (result != ESP_OK) goto fail;

    ESP_ERROR_CHECK_WITHOUT_ABORT(set_power(false));
    vTaskDelay(pdMS_TO_TICKS(250));
    result = set_power(true);
    if (result != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(250));
    if (!sd_storage_card_present()) {
        result = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    const spi_bus_config_t bus = {
        .mosi_io_num = BOARD_SD_MOSI,
        .miso_io_num = BOARD_SD_MISO,
        .sclk_io_num = BOARD_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    result = spi_bus_initialize(BOARD_SD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) goto fail;
    bus_ready = true;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = BOARD_SD_SPI_HOST;
    host.max_freq_khz = BOARD_SD_SPI_HZ / 1000;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.host_id = BOARD_SD_SPI_HOST;
    slot.gpio_cs = BOARD_SD_CS;
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 0,
    };
    result = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot,
                                     &mount_config, &mounted_card);
    if (result != ESP_OK) goto fail;
    mounted = true;
    client_count = 1;
    ESP_LOGI(TAG, "SD mounted for first client");
    xSemaphoreGive(storage_lock);
    return ESP_OK;

fail:
    close_storage();
    client_count = 0;
    xSemaphoreGive(storage_lock);
    return result;
}

void sd_storage_release(void)
{
    if (storage_lock == NULL) return;
    xSemaphoreTake(storage_lock, portMAX_DELAY);
    if (client_count == 0) {
        xSemaphoreGive(storage_lock);
        return;
    }
    client_count--;
    if (client_count == 0) {
        close_storage();
        ESP_LOGI(TAG, "SD unmounted after last client");
    }
    xSemaphoreGive(storage_lock);
}

