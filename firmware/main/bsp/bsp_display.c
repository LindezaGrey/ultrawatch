#include "bsp_display.h"
#include "bsp_twatch_ultra.h"

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_co5300.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "stdlib.h"

static const char *TAG = "bsp_display";

static esp_lcd_panel_handle_t s_panel = NULL;

/* --- SPI transfer instrumentation (JTAG/console diagnostics) --- */
volatile uint32_t s_flush_starts = 0;       /* draw_bitmap calls queued          */
volatile uint32_t s_trans_completions = 0;  /* on_color_trans_done invocations   */
volatile uint32_t s_xfer_overlaps = 0;      /* new flush while prev xfer pending */
volatile uint32_t s_xfer_inflight = 0;      /* 1 = color xfer queued, not done   */

/* The SPI/QSPI panel IO driver queues draw_bitmap transactions with DMA and
 * returns before the transfer finishes. LVGL must not recycle a color buffer
 * (lv_display_flush_ready) until the DMA has actually read it, otherwise the
 * panel shows mixed/teared content. We track completion via on_color_trans_done
 * and block the flusher on this semaphore. */
static SemaphoreHandle_t s_color_done_sem = NULL;

static bool IRAM_ATTR s_on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                            esp_lcd_panel_io_event_data_t *edata,
                                            void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    s_xfer_inflight = 0;
    s_trans_completions++;
    xSemaphoreGiveFromISR(s_color_done_sem, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

/* CO5300 init sequence for the 2.06" 410x502 AMOLED (from LilyGoLib LilyGoWatchUltra.cpp).
 * The address window places the visible area at columns 0x16..0x1AF and rows 0..0x1F5. */
static const co5300_lcd_init_cmd_t s_co5300_init_cmds[] = {
    { 0xFE, (uint8_t []){ 0x00 }, 1, 0 },
    { 0xC4, (uint8_t []){ 0x80 }, 1, 0 },
    { 0x3A, (uint8_t []){ 0x55 }, 1, 0 },
    { 0x35, (uint8_t []){ 0x00 }, 1, 0 },
    { 0x53, (uint8_t []){ 0x20 }, 1, 0 },
    { 0x63, (uint8_t []){ 0xFF }, 1, 0 },
    { 0x2A, (uint8_t []){ 0x00, 0x16, 0x01, 0xAF }, 4, 0 },
    { 0x2B, (uint8_t []){ 0x00, 0x00, 0x01, 0xF5 }, 4, 0 },
    { 0x11, NULL, 0, 120 },
    { 0x29, NULL, 0, 120 },
    { 0x51, (uint8_t []){ 0x00 }, 1, 0 },
};

esp_err_t bsp_display_init(void)
{
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_handle_t panel = NULL;

    s_color_done_sem = xSemaphoreCreateBinary();
    if (s_color_done_sem == NULL) {
        ESP_LOGE(TAG, "color-done semaphore alloc failed");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_cfg = CO5300_PANEL_BUS_QSPI_CONFIG(
        BSP_DISP_SCK_PIN, BSP_DISP_D0_PIN, BSP_DISP_D1_PIN,
        BSP_DISP_D2_PIN, BSP_DISP_D3_PIN,
        BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t));

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "init spi bus failed");

    esp_lcd_panel_io_spi_config_t io_cfg = CO5300_PANEL_IO_QSPI_CONFIG(BSP_DISP_CS_PIN,
                                                                       s_on_color_trans_done, NULL);
    /* 60MHz QSPI (~30MB/s) stays ahead of the panel's ~26MB/s GRAM scan rate
     * (410x502x2 @ 60Hz), so a full-frame write outruns the scan. The full-size
     * LVGL draw buffers live in PSRAM; the S3 GDMA reads PSRAM natively (do NOT
     * set psram_dma_direct here - the PSRAM-DMA mode it selects TX-underflows). */
    io_cfg.pclk_hz = 60 * 1000 * 1000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_cfg, &io),
                        TAG, "new panel io failed");

    co5300_vendor_config_t vendor_config = {
        .init_cmds = s_co5300_init_cmds,
        .init_cmds_size = sizeof(s_co5300_init_cmds) / sizeof(co5300_lcd_init_cmd_t),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .reset_gpio_num = BSP_DISP_RST_PIN,
        .vendor_config = &vendor_config,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(io, &panel_config, &panel), TAG, "new panel failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "panel on failed");

    /* Native portrait: the visible area spans columns 0x16..0x1AF (22..431) and
     * rows 0..0x1F5 (0..501). LVGL x/y map directly to panel columns/rows. */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel, false), TAG, "swap_xy failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(panel, 22, 0), TAG, "set_gap failed");

    s_panel = panel;
    ESP_LOGI(TAG, "display ready (%dx%d)", BSP_LCD_H_RES, BSP_LCD_V_RES);
    return ESP_OK;
}

esp_err_t bsp_display_wait_flush_done(void)
{
    if (s_color_done_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 40MHz QSPI, one 60-row band is ~10ms; full frame is ~82ms. 250ms is a
     * generous bound that still fails loudly if the pipeline stalls. */
    if (xSemaphoreTake(s_color_done_sem, pdMS_TO_TICKS(250)) != pdTRUE) {
        ESP_LOGW(TAG, "flush not done within 250ms (panel stalled?)");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_lcd_panel_handle_t bsp_display_get_panel(void)
{
    return s_panel;
}

void bsp_display_xfer_begin(void)
{
    s_flush_starts++;
    if (s_xfer_inflight) {
        s_xfer_overlaps++;
        ESP_LOGW(TAG, "OVERLAP: SPI color transfer still inflight when a new flush starts");
    }
    s_xfer_inflight = 1;
}

void bsp_display_get_stats(uint32_t *starts, uint32_t *completions, uint32_t *overlaps)
{
    if (starts) {
        *starts = s_flush_starts;
    }
    if (completions) {
        *completions = s_trans_completions;
    }
    if (overlaps) {
        *overlaps = s_xfer_overlaps;
    }
}

esp_err_t bsp_display_set_brightness(uint8_t percent)
{
    if (!s_panel) {
        return ESP_ERR_INVALID_STATE;
    }
    if (percent > 100) {
        percent = 100;
    }
    return esp_lcd_panel_co5300_set_brightness(s_panel, percent);
}

void bsp_display_show_test_pattern(void)
{
    if (!s_panel) {
        return;
    }
    /* Four quadrant colors to verify orientation, rotation and gap */
    uint16_t *buf = malloc(BSP_LCD_H_RES * 32 * sizeof(uint16_t));
    if (!buf) {
        return;
    }
    const uint16_t colors[] = {
        0xF800, /* red   */
        0x07E0, /* green */
        0x001F, /* blue  */
        0xFFFF, /* white */
    };
    for (int y = 0; y < BSP_LCD_V_RES; y += 32) {
        uint16_t color = colors[(y / 32) & 0x3];
        for (int i = 0; i < BSP_LCD_H_RES * 32; i++) {
            buf[i] = color;
        }
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, BSP_LCD_H_RES, y + 32, buf);
        bsp_display_wait_flush_done();
    }
    free(buf);
}
