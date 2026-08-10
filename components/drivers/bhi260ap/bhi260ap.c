/*
 * bhi260ap.c - Bosch BHI260AP smart sensor / IMU (I2C, Bosch FSC protocol).
 *
 * Uses the Bosch BHI2xy Sensor API (BSD-3, vendored in this component) to
 * bring the chip up: upload the RAM firmware over I2C, boot it, and stream
 * sensor data from the FIFO.
 *
 * The firmware blob is loaded from the SPIFFS "assets" partition
 * (/assets/bhi260/BHI260AP.fw) at init; call bhi260ap_init() only after the
 * assets partition is mounted. On failure the driver logs and continues
 * (the watch must keep booting even if the IMU is absent/faulty).
 */
#include "bhi260ap.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "bhi260ap";

#define BHI260_I2C_TIMEOUT_MS   1000
#define BHY2_RD_WR_LEN          256
#define BHY2_FIFO_BUFFER_SIZE   512

#define FIRMWARE_PATH           "/assets/bhi260/BHI260AP.fw"

static struct bhy2_dev s_bhy2;
static bool s_initialized;
static uint32_t s_step_count;

/* ---- I2C glue for the Bosch API ---- */

static int8_t bhi260ap_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg_addr, 1, reg_data, length, BHI260_I2C_TIMEOUT_MS);
    return (ret == ESP_OK) ? BHY2_OK : BHY2_E_IO;
}

static int8_t bhi260ap_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    uint8_t *buf = heap_caps_malloc(length + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) {
        return BHY2_E_IO;
    }
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, length);
    esp_err_t ret = i2c_master_transmit(dev, buf, length + 1, BHI260_I2C_TIMEOUT_MS);
    free(buf);
    return (ret == ESP_OK) ? BHY2_OK : BHY2_E_IO;
}

static void bhi260ap_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

/* ---- FIFO parse callbacks ---- */

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    ESP_LOGD(TAG, "meta event: sid=%u data_size=%u", callback_info->sensor_id, callback_info->data_size);
}

static void parse_step_counter(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size < 4) {
        return;
    }
    uint32_t steps = BHY2_LE2U32(callback_info->data_ptr);
    s_step_count = steps;
    ESP_LOGI(TAG, "step counter: %lu", (unsigned long)steps);
}

/* Load the RAM firmware blob from SPIFFS into a PSRAM buffer. */
static esp_err_t load_firmware(uint8_t **out, size_t *out_len)
{
    FILE *f = fopen(FIRMWARE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "open %s failed", FIRMWARE_PATH);
        return ESP_ERR_NOT_FOUND;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return ESP_FAIL;
    }
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fclose(f);
        return ESP_FAIL;
    }

    uint8_t *buf = heap_caps_malloc((size_t)size, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "fw buffer alloc failed (%ld B)", size);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }
    size_t rd = fread(buf, 1, (size_t)size, f);
    fclose(f);
    if (rd != (size_t)size) {
        ESP_LOGE(TAG, "fw read %u/%ld", (unsigned)rd, size);
        free(buf);
        return ESP_FAIL;
    }

    *out = buf;
    *out_len = (size_t)size;
    ESP_LOGI(TAG, "loaded firmware: %u bytes", (unsigned)size);
    return ESP_OK;
}

esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev)
{
    int8_t rslt;
    uint8_t product_id = 0;
    uint8_t boot_status = 0;
    uint16_t kernel_version = 0;
    uint8_t *fw = NULL;
    size_t fw_len = 0;

    ESP_RETURN_ON_ERROR(load_firmware(&fw, &fw_len), TAG, "load firmware");

    rslt = bhy2_init(BHY2_I2C_INTERFACE, bhi260ap_i2c_read, bhi260ap_i2c_write,
                     bhi260ap_delay_us, BHY2_RD_WR_LEN, dev, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "bhy2_init failed: %d", rslt);
        free(fw);
        return ESP_FAIL;
    }

    rslt = bhy2_soft_reset(&s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "soft reset failed: %d", rslt);
        free(fw);
        return ESP_FAIL;
    }

    rslt = bhy2_get_product_id(&product_id, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "get product id failed: %d", rslt);
        free(fw);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "product id: 0x%02X (expected 0x%02X)", product_id, BHY2_PRODUCT_ID);

    /* Disable status/debug FIFO interrupts, enable normal FIFO. */
    bhy2_set_host_interrupt_ctrl(BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG, &s_bhy2);
    bhy2_set_host_intf_ctrl(0, &s_bhy2);

    rslt = bhy2_get_boot_status(&boot_status, &s_bhy2);
    if (rslt != BHY2_OK || !(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
        ESP_LOGE(TAG, "host interface not ready: rslt=%d status=0x%02X", rslt, boot_status);
        free(fw);
        return ESP_FAIL;
    }

    rslt = bhy2_upload_firmware_to_ram(fw, (uint32_t)fw_len, &s_bhy2);
    free(fw);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "firmware upload failed: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bhy2_boot_from_ram(&s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "boot from RAM failed: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bhy2_get_kernel_version(&kernel_version, &s_bhy2);
    if (rslt != BHY2_OK || kernel_version == 0) {
        ESP_LOGE(TAG, "kernel version check failed: rslt=%d ver=%u", rslt, kernel_version);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "RAM firmware booted, kernel version %u", kernel_version);

    /* Register FIFO parse callbacks and enable the step counter. */
    bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC, parse_step_counter, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC_WU, parse_step_counter, NULL, &s_bhy2);

    rslt = bhy2_update_virtual_sensor_list(&s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "update sensor list failed: %d", rslt);
    }

    rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_STC, 1.0f, 0, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "enable step counter failed: %d", rslt);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "BHI260AP ready");
    return ESP_OK;
}

/* Poll the FIFO once; returns ESP_OK on success. */
esp_err_t bhi260ap_process_fifo(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t work[BHY2_FIFO_BUFFER_SIZE];
    int8_t rslt = bhy2_get_and_process_fifo(work, sizeof(work), &s_bhy2);
    return (rslt == BHY2_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t bhi260ap_get_step_count(uint32_t *steps)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *steps = s_step_count;
    return ESP_OK;
}

esp_err_t bhi260ap_read_accel(i2c_master_dev_handle_t dev, bhi260ap_accel_t *accel)
{
    (void)dev;
    (void)accel;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bhi260ap_read_gyro(i2c_master_dev_handle_t dev, int16_t out[3])
{
    (void)dev;
    (void)out;
    return ESP_ERR_NOT_SUPPORTED;
}
