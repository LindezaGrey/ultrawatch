#include <inttypes.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bhi260ap_core.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "twatch_board.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "bhi260ap_core";
static struct bhy2_dev device;
static bool initialized;
static TaskHandle_t irq_task_handle;
static uint8_t fifo_buffer[2048];
extern const uint8_t _binary_BHI260AP_fw_start[];
extern const uint8_t _binary_BHI260AP_fw_end[];

static int8_t read_cb(uint8_t reg, uint8_t *data, uint32_t len, void *ctx)
{
    (void)ctx;
    return twatch_board_bhi_read(reg, data, len) == ESP_OK ? BHY2_INTF_RET_SUCCESS : -1;
}

static int8_t write_cb(uint8_t reg, const uint8_t *data, uint32_t len, void *ctx)
{
    (void)ctx;
    return twatch_board_bhi_write(reg, data, len) == ESP_OK ? BHY2_INTF_RET_SUCCESS : -1;
}

static void delay_cb(uint32_t period_us, void *ctx)
{
    (void)ctx;
    esp_rom_delay_us(period_us);
}

esp_err_t bhi260ap_core_probe(bool *product_id_ok, uint8_t *product_id)
{
    if (!product_id_ok || !product_id) return ESP_ERR_INVALID_ARG;
    int8_t result = bhy2_init(BHY2_I2C_INTERFACE, read_cb, write_cb, delay_cb, 32, NULL, &device);
    if (result == BHY2_OK) initialized = true;
    if (result == BHY2_OK) result = bhy2_soft_reset(&device);
    if (result == BHY2_OK) result = bhy2_get_product_id(product_id, &device);
    *product_id_ok = result == BHY2_OK && *product_id == BHY2_PRODUCT_ID;
    ESP_LOGI(TAG, "core_probe result=%d product=0x%02x expected=0x%02x valid=%u",
             result, *product_id, BHY2_PRODUCT_ID, *product_id_ok);
    return result == BHY2_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t bhi260ap_core_load_firmware(uint8_t *boot_status)
{
    if (!initialized || !boot_status) return ESP_ERR_INVALID_STATE;
    const uint32_t firmware_size = (uint32_t)(_binary_BHI260AP_fw_end - _binary_BHI260AP_fw_start);
    int8_t result = bhy2_upload_firmware_to_ram(_binary_BHI260AP_fw_start, firmware_size, &device);
    if (result == BHY2_OK) result = bhy2_boot_from_ram(&device);
    if (result == BHY2_OK) result = bhy2_get_boot_status(boot_status, &device);
    ESP_LOGI(TAG, "firmware_load bytes=%" PRIu32 " result=%d boot_status=0x%02x",
             firmware_size, result, *boot_status);
    if (result != BHY2_OK) return ESP_FAIL;
    if (!(*boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) ||
        (*boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR)) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t bhi260ap_core_list_virtual_sensors(uint8_t *present, size_t present_size, size_t *count)
{
    if (!initialized || !present || !count || present_size < sizeof(device.present_buff)) {
        return ESP_ERR_INVALID_ARG;
    }
    int8_t result = bhy2_get_virt_sensor_list(&device);
    if (result != BHY2_OK) {
        ESP_LOGE(TAG, "virtual sensor list failed result=%d", result);
        return ESP_FAIL;
    }
    memcpy(present, device.present_buff, sizeof(device.present_buff));
    *count = 0;
    for (size_t sensor_id = 0; sensor_id < sizeof(device.present_buff) * 8U; ++sensor_id) {
        if (present[sensor_id / 8U] & (uint8_t)(1U << (sensor_id % 8U))) {
            ++*count;
        }
    }
    ESP_LOGI(TAG, "virtual sensor list retrieved count=%u", (unsigned)*count);
    return ESP_OK;
}

static void IRAM_ATTR bhi_irq_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &higher_priority_task_woken);
    if (higher_priority_task_woken) portYIELD_FROM_ISR();
}

static void bhi_irq_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t notifications = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (notifications == 0) continue;
        int8_t result = bhy2_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &device);
        ESP_LOGI(TAG, "irq event count=%" PRIu32 " fifo_result=%d", notifications, result);
    }
}

static void accel_callback(const struct bhy2_fifo_parse_data_info *info, void *private_data)
{
    (void)private_data;
    if (!info || info->data_size < 6) return;
    struct bhy2_data_xyz xyz;
    bhy2_parse_xyz(info->data_ptr, &xyz);
    ESP_LOGI(TAG, "accelerometer id=%u x=%d y=%d z=%d", info->sensor_id, xyz.x, xyz.y, xyz.z);
}

esp_err_t bhi260ap_core_start_irq_probe(void)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    if (irq_task_handle) return ESP_OK;
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << 8,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_HIGH_LEVEL,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    esp_err_t service = gpio_install_isr_service(0);
    if (service != ESP_OK && service != ESP_ERR_INVALID_STATE) return service;
    BaseType_t created = xTaskCreate(bhi_irq_task, "bhi_irq", 3072, NULL, 5, &irq_task_handle);
    if (created != pdPASS) return ESP_ERR_NO_MEM;
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_8, bhi_irq_isr, irq_task_handle));
    int8_t result = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, accel_callback, NULL, &device);
    if (result != BHY2_OK) return ESP_FAIL;
    result = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, 1.0f, 0, &device);
    if (result != BHY2_OK) return ESP_FAIL;
    result = bhy2_set_host_interrupt_ctrl(0, &device);
    if (result != BHY2_OK) return ESP_FAIL;
    result = bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &device);
    if (result != BHY2_OK) return ESP_FAIL;
    ESP_LOGI(TAG, "accelerometer stream id=%u rate=1Hz configured", BHY2_SENSOR_ID_ACC);
    ESP_LOGI(TAG, "irq probe GPIO8 configured idle=%d", gpio_get_level(GPIO_NUM_8));
    return ESP_OK;
}
