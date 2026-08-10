#include "co5300.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "co5300";

#define CO5300_SPI_HOST   SPI3_HOST
#define CO5300_X_GAP      22
#define CO5300_BAND_H     48   /* even fill-band height */

static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_panel_io_handle_t s_panel_io = NULL;
static bool s_initialized = false;

/* The CO5300 panel samples RGB565 big-endian (high byte first); our color
 * words are little-endian, so pixels are byte-swapped before transmission.
 * Sized for the largest draw used (a fill band). */
static uint16_t s_swap_buf[CO5300_RES_X * CO5300_BAND_H];

/* CO5300 init sequence (from LilyGO factory firmware). Sent after the
 * SH8601 driver's own MADCTL/COLMOD, via the QSPI panel IO. */
static const sh8601_lcd_init_cmd_t s_init_cmds[] = {
    { 0xFE, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0xC4, (uint8_t[]){ 0x80 }, 1, 0 },
    { 0x35, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x53, (uint8_t[]){ 0x20 }, 1, 25 },
    { 0x63, (uint8_t[]){ 0xFF }, 1, 0 },
    { 0x11, NULL, 0, 120 },                          /* SLPOUT */
    { 0x29, NULL, 0, 120 },                          /* DISPON */
    { 0x51, (uint8_t[]){ 0x80 }, 1, 0 },             /* brightness */
};

esp_err_t co5300_init(void)
{
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(CO5300_PIN_SCK,
                                                                 CO5300_PIN_DATA0,
                                                                 CO5300_PIN_DATA1,
                                                                 CO5300_PIN_DATA2,
                                                                 CO5300_PIN_DATA3,
                                                                 CO5300_RES_X * 80 * sizeof(uint16_t));
    ESP_RETURN_ON_ERROR(spi_bus_initialize(CO5300_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus init");

    esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(CO5300_PIN_CS, NULL, NULL);
    io_config.pclk_hz = 80 * 1000 * 1000;   /* ESP32-S3 SPI max; panel proven at 80 MHz by factory fw */
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CO5300_SPI_HOST, &io_config, &s_panel_io),
                        TAG, "panel io init");

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = CO5300_PIN_RESET,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_sh8601(s_panel_io, &panel_config, &s_panel), TAG, "panel driver init");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, CO5300_X_GAP, 0), TAG, "set gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "display on");

    s_initialized = true;
    ESP_LOGI(TAG, "CO5300 initialized via SH8601 driver (%dx%d)", CO5300_RES_X, CO5300_RES_Y);
    return ESP_OK;
}

esp_err_t co5300_deinit(void)
{
    if (s_panel) {
        esp_lcd_panel_del(s_panel);
        s_panel = NULL;
    }
    spi_bus_free(CO5300_SPI_HOST);
    s_initialized = false;
    return ESP_OK;
}

/* SH8601 requires even x/y draw boundaries, so callers must pass even
 * x0/y0 and odd x1/y1 (with buffers sized accordingly). Pass-through. */
esp_err_t co5300_draw_bitmap(int x0, int y0, int x1, int y1, const void *pixdata)
{
    if (!s_initialized || !pixdata) {
        return ESP_ERR_INVALID_STATE;
    }
    int w = x1 - x0 + 1;
    int h = y1 - y0 + 1;
    if (w <= 0 || h <= 0 || x0 < 0 || y0 < 0 || x1 >= CO5300_RES_X || y1 >= CO5300_RES_Y) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((size_t)w * h > sizeof(s_swap_buf) / sizeof(s_swap_buf[0])) {
        return ESP_ERR_INVALID_SIZE;
    }
    /* byte-swap each pixel: little-endian host -> big-endian panel */
    const uint8_t *src = pixdata;
    uint8_t *dst = (uint8_t *)s_swap_buf;
    for (size_t i = 0; i < (size_t)w * h; i++) {
        dst[i * 2] = src[i * 2 + 1];
        dst[i * 2 + 1] = src[i * 2];
    }
    return esp_lcd_panel_draw_bitmap(s_panel, x0, y0, x1 + 1, y1 + 1, s_swap_buf);
}

/* Fill in even-sized bands (CO5300_BAND_H rows each) to satisfy the SH8601
 * even-coordinate requirement. */
esp_err_t co5300_fill(uint16_t color)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    static uint16_t band[CO5300_RES_X * CO5300_BAND_H];
    for (int i = 0; i < CO5300_RES_X * CO5300_BAND_H; i++) {
        band[i] = color;
    }
    for (int y = 0; y < CO5300_RES_Y; y += CO5300_BAND_H) {
        int h = (y + CO5300_BAND_H <= CO5300_RES_Y) ? CO5300_BAND_H : (CO5300_RES_Y - y);
        esp_err_t err = co5300_draw_bitmap(0, y, CO5300_RES_X - 1, y + h - 1, band);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_lcd_panel_handle_t co5300_get_panel(void)
{
    return s_panel;
}

esp_lcd_panel_io_handle_t co5300_get_panel_io(void)
{
    return s_panel_io;
}

/* Send a MIPI command in the CO5300 QSPI encoding (same as the SH8601 driver). */
static esp_err_t co5300_send_cmd(uint8_t cmd)
{
    int lcd_cmd = (int)((0x02UL << 24) | ((uint32_t)cmd << 8));
    return esp_lcd_panel_io_tx_param(s_panel_io, lcd_cmd, NULL, 0);
}

esp_err_t co5300_set_brightness(uint8_t bri)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    int lcd_cmd = (int)((0x02UL << 24) | (0x51UL << 8));            /* Write Display Brightness */
    return esp_lcd_panel_io_tx_param(s_panel_io, lcd_cmd, &bri, 1);
}

esp_err_t co5300_sleep(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return co5300_send_cmd(0x10);   /* SLPIN */
}

esp_err_t co5300_wake(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(co5300_send_cmd(0x11), TAG, "slpout");      /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(co5300_send_cmd(0x29), TAG, "dison");       /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(120));
    return co5300_set_brightness(0x80);
}
