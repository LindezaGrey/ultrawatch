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

static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_panel_io_handle_t s_panel_io = NULL;
static bool s_initialized = false;

/* CO5300 init sequence (from LilyGO factory firmware). Sent after the
 * SH8601 driver's own MADCTL/COLMOD, via the QSPI panel IO. */
static const sh8601_lcd_init_cmd_t s_init_cmds[] = {
    { 0xFE, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0xC4, (uint8_t[]){ 0x80 }, 1, 0 },
    { 0x35, (uint8_t[]){ 0x00 }, 1, 0 },
    { 0x53, (uint8_t[]){ 0x20 }, 1, 25 },
    { 0x63, (uint8_t[]){ 0xFF }, 1, 0 },
    { 0x11, NULL, 0, 120 },                          /* SLPOUT */
    { 0x51, (uint8_t[]){ 0x80 }, 1, 0 },             /* brightness */
};

esp_err_t co5300_init(bool leave_display_off)
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
    s_initialized = true;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, !leave_display_off), TAG, "display output");
    ESP_LOGI(TAG, "CO5300 initialized via SH8601 driver (%dx%d)", CO5300_RES_X, CO5300_RES_Y);
    return ESP_OK;
}

/* Full panel bring-up on the EXISTING handle, for recovering a panel that
 * stopped reflecting what it is sent (the shake-triggered white screen).
 *
 * debug_cmd_disppwr() cycles the DISP_PWR rail and then sends only
 * SLPOUT/DISPON/brightness. That is not enough: after a power cycle the
 * CO5300 is back in its OTP defaults, so MADCTL/COLMOD and the vendor init
 * list above (0xFE/0xC4/0x35/0x53/0x63) are gone, and LVGL keeps pushing
 * RGB565 at a controller that may no longer be in 16bpp mode. This redoes
 * the whole sequence co5300_init() performs after the handle exists -
 * hardware reset, init commands, column gap, display on.
 *
 * Deliberately does NOT delete/recreate the panel handle: esp_lv_adapter was
 * given co5300_get_panel() once at display registration (see lvgl_app.c), so
 * a new handle would leave the flush path using a freed pointer. The caller
 * must hold the LVGL lock so no flush runs against a half-initialized panel. */
esp_err_t co5300_reinit(bool with_hw_reset, bool leave_display_off)
{
    if (!s_initialized || !s_panel) {
        return ESP_ERR_INVALID_STATE;
    }
    /* with_hw_reset=false re-sends the configuration without pulsing the RST
     * pin. If that alone recovers the panel, the QSPI link is intact and the
     * controller had merely lost its register configuration - which is the
     * difference between "the data path glitched" and "the panel reset". */
    if (with_hw_reset) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset");
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, CO5300_X_GAP, 0), TAG, "set gap");
    /* A reset panel's GRAM is undefined, and the init command list above ends
     * with 0x29 DISPON - so turning the display on here would show whatever
     * noise the reset left behind until the next repaint lands. Callers that
     * repaint afterwards (the wake path) pass leave_display_off=true and send
     * DISPON themselves once the frame is in, preserving the "no stale frame
     * flashes" ordering power_mgmt_exit_sleep() has always relied on. */
    if (leave_display_off) {
        ESP_RETURN_ON_ERROR(co5300_display_off(), TAG, "display off");
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "display on");
    }
    ESP_LOGI(TAG, "panel re-initialized (display %s)", leave_display_off ? "off" : "on");
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

/* Turn the display output off (DCS 0x28). The panel blanks (black) regardless
 * of GRAM content. Used before SLPIN so the panel never shows a stale frame. */
esp_err_t co5300_display_off(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return co5300_send_cmd(0x28);   /* DISPOFF */
}

/* Turn the display output back on (DCS 0x29). */
esp_err_t co5300_display_on(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return co5300_send_cmd(0x29);   /* DISPON */
}

/* Clear the visible panel to black (0x0000) by looping a small static band.
 * Black is byte-symmetric, so no byte-swap is needed. Called before SLPIN so
 * the panel GRAM holds black instead of the last frame, avoiding a flash of
 * stale content on wake. */
#define CO5300_BLANK_BAND_H 8

esp_err_t co5300_blank(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    static uint16_t s_black_band[CO5300_RES_X * CO5300_BLANK_BAND_H];
    for (int y = 0; y < CO5300_RES_Y; y += CO5300_BLANK_BAND_H) {
        int h = CO5300_BLANK_BAND_H;
        if (y + h > CO5300_RES_Y) {
            h = CO5300_RES_Y - y;
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, 0, y,
                                                  CO5300_RES_X, y + h,
                                                  s_black_band);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

/* Wake the panel out of sleep (SLPOUT) and set brightness. DISPON is left to
 * the caller (co5300_display_on) so the panel stays black until the host has
 * repainted the screen, avoiding a flash of stale GRAM content. */
esp_err_t co5300_wake(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(co5300_send_cmd(0x11), TAG, "slpout");      /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    return co5300_set_brightness(0x80);
}
