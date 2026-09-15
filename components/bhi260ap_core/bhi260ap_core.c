#include "bhi260ap_core.h"
#include "bhy2.h"
#include "twatch_board.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "bhi260ap_core";

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
    struct bhy2_dev dev = {0};
    int8_t result = bhy2_init(BHY2_I2C_INTERFACE, read_cb, write_cb, delay_cb, 32, NULL, &dev);
    if (result == BHY2_OK) result = bhy2_soft_reset(&dev);
    if (result == BHY2_OK) result = bhy2_get_product_id(product_id, &dev);
    *product_id_ok = result == BHY2_OK && *product_id == BHY2_PRODUCT_ID;
    ESP_LOGI(TAG, "core_probe result=%d product=0x%02x expected=0x%02x valid=%u",
             result, *product_id, BHY2_PRODUCT_ID, *product_id_ok);
    return result == BHY2_OK ? ESP_OK : ESP_FAIL;
}
