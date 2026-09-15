#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io_interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "twatch_display.h"

static const char *TAG = "twatch_display";
enum {
    DISPLAY_RST = 37, DISPLAY_CS = 41, DISPLAY_SCLK = 40,
    DISPLAY_D0 = 38, DISPLAY_D1 = 39, DISPLAY_D2 = 42, DISPLAY_D3 = 45,
    DISPLAY_WIDTH = 410, DISPLAY_HEIGHT = 502, DISPLAY_OFFSET_X = 22,
    /* LilyGO's proven QSPI path limits each pixel transaction to 16,384 pixels. */
    DISPLAY_SEND_BUF_PIXELS = 16384,
};

typedef struct { uint8_t command; const uint8_t *data; uint8_t length; bool delay_after; } display_cmd_t;
static const uint8_t c4[] = {0x80}, pixfmt[] = {0x55}, te[] = {0x00};
static const uint8_t c53[] = {0x20}, c63[] = {0xff}, brightness[] = {0xff};
static const uint8_t madctl_portrait_rgb[] = {0x00};
static const uint8_t window_x[] = {0x00, 0x16, 0x01, 0xaf};
static const uint8_t window_y[] = {0x00, 0x00, 0x01, 0xf5};
static const display_cmd_t init_sequence[] = {
    {0xfe, NULL, 0, false}, {0xc4, c4, 1, false}, {0x3a, pixfmt, 1, false},
    {0x35, te, 1, false}, {0x53, c53, 1, false}, {0x63, c63, 1, false},
    {0x2a, window_x, 4, false}, {0x2b, window_y, 4, false},
    {0x11, NULL, 0, true}, {0x29, NULL, 0, true}, {0x51, brightness, 1, false},
};

static spi_device_handle_t device;

static bool initialized;
static uint16_t line_buffer[DISPLAY_WIDTH * 16];

static esp_err_t send_command(uint8_t command, const uint8_t *data, size_t length)
{
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    transaction.cmd = 0x02;
    transaction.addr = ((uint32_t)command) << 8;
    transaction.tx_buffer = data;
    transaction.length = length * 8;
    gpio_set_level(DISPLAY_CS, 0);
    esp_err_t err = spi_device_polling_transmit(device, &transaction);
    gpio_set_level(DISPLAY_CS, 1);
    return err;
}

esp_err_t twatch_display_init(void)
{
    if (initialized) return ESP_OK;
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << DISPLAY_CS) | (1ULL << DISPLAY_RST),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "Configure CO5300 control pins");
    gpio_set_level(DISPLAY_CS, 1);
    gpio_set_level(DISPLAY_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(DISPLAY_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(DISPLAY_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    const spi_bus_config_t bus = {
        .data0_io_num = DISPLAY_D0, .data1_io_num = DISPLAY_D1,
        .sclk_io_num = DISPLAY_SCLK, .data2_io_num = DISPLAY_D2,
        .data3_io_num = DISPLAY_D3, .data4_io_num = -1, .data5_io_num = -1,
        .data6_io_num = -1, .data7_io_num = -1, .max_transfer_sz = 64 * 1024,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO), TAG,
                        "Initialize CO5300 QSPI bus");
    const spi_device_interface_config_t config = {
        .command_bits = 8, .address_bits = 24, .mode = 0,
        .clock_speed_hz = 45 * 1000 * 1000, .spics_io_num = -1,
        .queue_size = 1, .flags = SPI_DEVICE_HALFDUPLEX,
    };
    esp_err_t err = spi_bus_add_device(SPI3_HOST, &config, &device);
    if (err != ESP_OK) {
        spi_bus_free(SPI3_HOST);
        ESP_LOGE(TAG, "Add CO5300 QSPI device: %s", esp_err_to_name(err));
        return err;
    }
    /* Recovery: GPIO41 remains owned by the custom software-CS renderer. */
    for (size_t pass = 0; pass < 2; ++pass) {
        for (size_t index = 0; index < sizeof(init_sequence) / sizeof(init_sequence[0]); ++index) {
            const display_cmd_t *entry = &init_sequence[index];
            ESP_RETURN_ON_ERROR(send_command(entry->command, entry->data, entry->length), TAG,
                                "CO5300 init command 0x%02x", entry->command);
            if (entry->delay_after) vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
    /* LilyGoDispQSPI::setRotation(0): portrait geometry, RGB order. */
    ESP_RETURN_ON_ERROR(send_command(0x36, madctl_portrait_rgb, 1), TAG,
                        "Set CO5300 portrait/RGB order");
    initialized = true;
    ESP_LOGI(TAG, "CO5300 ready %ux%u QSPI cs=%d rst=%d clk=%d d0=%d d1=%d d2=%d d3=%d",
             DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_CS, DISPLAY_RST, DISPLAY_SCLK,
             DISPLAY_D0, DISPLAY_D1, DISPLAY_D2, DISPLAY_D3);
    return ESP_OK;
}

esp_err_t twatch_display_set_inverted(bool inverted)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    return send_command(inverted ? 0x21 : 0x20, NULL, 0);
}

static esp_err_t set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    x0 += DISPLAY_OFFSET_X;
    x1 += DISPLAY_OFFSET_X;
    const uint8_t columns[] = {x0 >> 8, x0, x1 >> 8, x1};
    const uint8_t rows[] = {y0 >> 8, y0, y1 >> 8, y1};
    ESP_RETURN_ON_ERROR(send_command(0x2a, columns, sizeof(columns)), TAG, "Set CO5300 columns");
    ESP_RETURN_ON_ERROR(send_command(0x2b, rows, sizeof(rows)), TAG, "Set CO5300 rows");
    return ESP_OK;
}

static esp_err_t fill_region(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    const size_t width = x1 - x0 + 1;
    const size_t rows = y1 - y0 + 1;
    ESP_RETURN_ON_ERROR(set_window(x0, y0, x1, y1), TAG, "Set CO5300 test window");
    for (size_t index = 0; index < width * 16; ++index) line_buffer[index] = color;
    gpio_set_level(DISPLAY_CS, 0);
    size_t remaining = width * rows;
    bool first = true;
    while (remaining != 0) {
        const size_t chunk_rows = remaining / width > 16 ? 16 : remaining / width;
        const size_t pixels = chunk_rows * width;
        spi_transaction_ext_t transaction = {0};
        transaction.base.tx_buffer = line_buffer;
        transaction.base.length = pixels * 16;
        if (first) {
            transaction.base.flags = SPI_TRANS_MODE_QIO;
            transaction.base.cmd = 0x32;
            transaction.base.addr = 0x002c00;
            first = false;
        } else {
            transaction.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
            transaction.command_bits = 0;
            transaction.address_bits = 0;
            transaction.dummy_bits = 0;
        }
        esp_err_t err = spi_device_polling_transmit(device, &transaction.base);
        if (err != ESP_OK) {
            gpio_set_level(DISPLAY_CS, 1);
            return err;
        }
        remaining -= pixels;
    }
    gpio_set_level(DISPLAY_CS, 1);
    return ESP_OK;
}

/* Synchronous transfer: the buffer is released when this call returns. */
static esp_err_t send_pixels(const void *pixels, size_t pixel_count)
{
    gpio_set_level(DISPLAY_CS, 0);
    size_t remaining = pixel_count;
    const uint8_t *data = (const uint8_t *)pixels;
    bool first = true;
    while (remaining != 0) {
        const size_t pixels_now = remaining > DISPLAY_SEND_BUF_PIXELS
                                      ? DISPLAY_SEND_BUF_PIXELS : remaining;
        spi_transaction_ext_t transaction = {0};
        transaction.base.tx_buffer = data;
        transaction.base.length = pixels_now * 16;
        if (first) {
            transaction.base.flags = SPI_TRANS_MODE_QIO;
            transaction.base.cmd = 0x32;
            transaction.base.addr = 0x002c00;
            first = false;
        } else {
            transaction.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
            transaction.command_bits = 0;
            transaction.address_bits = 0;
            transaction.dummy_bits = 0;
        }
        esp_err_t err = spi_device_polling_transmit(device, &transaction.base);
        if (err != ESP_OK) {
            gpio_set_level(DISPLAY_CS, 1);
            return err;
        }
        data += pixels_now * sizeof(uint16_t);
        remaining -= pixels_now;
    }
    gpio_set_level(DISPLAY_CS, 1);
    return ESP_OK;
}

esp_err_t twatch_display_flush(uint16_t x, uint16_t y, uint16_t width,
                               uint16_t height, const void *pixels)
{
    if (!initialized || pixels == NULL || width == 0 || height == 0 ||
        x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT || width > DISPLAY_WIDTH - x ||
        height > DISPLAY_HEIGHT - y) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(set_window(x, y, x + width - 1, y + height - 1), TAG,
                        "Set CO5300 flush window");
    return send_pixels(pixels, (size_t)width * height);
}

esp_err_t twatch_display_test_pattern(void)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    const uint16_t mid_x = (DISPLAY_WIDTH / 2) - 1;
    const uint16_t mid_y = (DISPLAY_HEIGHT / 2) - 1;
    ESP_RETURN_ON_ERROR(set_window(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1), TAG,
                        "Set CO5300 full pattern window");
    gpio_set_level(DISPLAY_CS, 0);
    bool first = true;
    for (uint16_t y = 0; y < DISPLAY_HEIGHT;) {
        const uint16_t rows = DISPLAY_HEIGHT - y > 16 ? 16 : DISPLAY_HEIGHT - y;
        for (uint16_t row = 0; row < rows; ++row) {
            const uint16_t color_y = y + row < mid_y + 1 ? 0 : 1;
            for (uint16_t x = 0; x < DISPLAY_WIDTH; ++x) {
                const uint16_t color_x = x < mid_x + 1 ? 0 : 1;
                const uint16_t colors[2][2] = {{0xf800, 0x07e0}, {0x001f, 0xffff}};
                line_buffer[row * DISPLAY_WIDTH + x] = colors[color_y][color_x];
            }
        }
        spi_transaction_ext_t transaction = {0};
        transaction.base.tx_buffer = line_buffer;
        transaction.base.length = rows * DISPLAY_WIDTH * 16;
        if (first) {
            transaction.base.flags = SPI_TRANS_MODE_QIO;
            transaction.base.cmd = 0x32;
            transaction.base.addr = 0x002c00;
            first = false;
        } else {
            transaction.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
            transaction.command_bits = 0;
            transaction.address_bits = 0;
            transaction.dummy_bits = 0;
        }
        esp_err_t err = spi_device_polling_transmit(device, &transaction.base);
        if (err != ESP_OK) {
            gpio_set_level(DISPLAY_CS, 1);
            return err;
        }
        y += rows;
    }
    gpio_set_level(DISPLAY_CS, 1);
    ESP_LOGI(TAG, "TEST_PATTERN ready RGB565 quadrants one_window rows_per_chunk=16");
    return ESP_OK;
}

esp_err_t twatch_display_geometry_test(void)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(set_window(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1), TAG,
                        "Set CO5300 geometry window");
    gpio_set_level(DISPLAY_CS, 0);
    bool first = true;
    uint32_t seed = 0x6d2b79f5u;
    for (uint16_t y = 0; y < DISPLAY_HEIGHT;) {
        const uint16_t rows = DISPLAY_HEIGHT - y > 16 ? 16 : DISPLAY_HEIGHT - y;
        for (uint16_t row = 0; row < rows; ++row) {
            for (uint16_t x = 0; x < DISPLAY_WIDTH; ++x) {
                seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
                uint16_t color = (uint16_t)(((seed >> 19) & 0x1f) << 11 |
                                            ((seed >> 10) & 0x3f) << 5 |
                                            ((seed >> 3) & 0x1f));
                if ((x % 32 == 0) || ((y + row) % 32 == 0)) color = 0xffff;
                line_buffer[row * DISPLAY_WIDTH + x] = color;
            }
        }
        spi_transaction_ext_t transaction = {0};
        transaction.base.tx_buffer = line_buffer;
        transaction.base.length = rows * DISPLAY_WIDTH * 16;
        if (first) {
            transaction.base.flags = SPI_TRANS_MODE_QIO;
            transaction.base.cmd = 0x32;
            transaction.base.addr = 0x002c00;
            first = false;
        } else {
            transaction.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                      SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
            transaction.command_bits = 0; transaction.address_bits = 0; transaction.dummy_bits = 0;
        }
        ESP_RETURN_ON_ERROR(spi_device_polling_transmit(device, &transaction.base), TAG,
                            "Draw CO5300 geometry test");
        y += rows;
    }
    gpio_set_level(DISPLAY_CS, 1);
    ESP_LOGI(TAG, "GEOMETRY_TEST ready tile=32 grid=white");
    return ESP_OK;
}

esp_err_t twatch_display_test_solid(uint16_t color)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(fill_region(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1, color),
                        TAG, "Draw solid test window");
    ESP_LOGI(TAG, "TEST_SOLID ready color=0x%04x rows_per_chunk=16", color);
    return ESP_OK;
}

esp_err_t twatch_display_rotation_demo(void)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    static const uint8_t rotations[] = {0x00, 0x60, 0xc0, 0xa0, 0x00};
    enum { MARKER_INSET = 48, MARKER_SIZE = 40 };
    for (size_t cycle = 0; cycle < 3; ++cycle) {
        for (size_t index = 0; index < sizeof(rotations); ++index) {
            const uint8_t portrait = 0x00;
            const uint8_t value = rotations[index];
            ESP_RETURN_ON_ERROR(send_command(0x36, &portrait, 1), TAG, "Clear rotation mapping");
            ESP_RETURN_ON_ERROR(fill_region(MARKER_INSET, MARKER_INSET, 87, 87, 0x0000), TAG, "Clear marker TL");
            ESP_RETURN_ON_ERROR(fill_region(DISPLAY_WIDTH - 88, MARKER_INSET, DISPLAY_WIDTH - 49, 87, 0x0000), TAG, "Clear marker TR");
            ESP_RETURN_ON_ERROR(fill_region(MARKER_INSET, DISPLAY_HEIGHT - 88, 87, DISPLAY_HEIGHT - 49, 0x0000), TAG, "Clear marker BL");
            ESP_RETURN_ON_ERROR(fill_region(DISPLAY_WIDTH - 88, DISPLAY_HEIGHT - 88,
                                            DISPLAY_WIDTH - 49, DISPLAY_HEIGHT - 49, 0x0000), TAG,
                                "Clear marker BR");
            ESP_RETURN_ON_ERROR(send_command(0x36, &value, 1), TAG, "Set CO5300 demo rotation");
            ESP_RETURN_ON_ERROR(fill_region(MARKER_INSET, MARKER_INSET,
                                            MARKER_INSET + MARKER_SIZE - 1,
                                            MARKER_INSET + MARKER_SIZE - 1, 0xf800), TAG,
                                "Draw rotation marker TL");
            ESP_RETURN_ON_ERROR(fill_region(DISPLAY_WIDTH - MARKER_INSET - MARKER_SIZE,
                                            MARKER_INSET, DISPLAY_WIDTH - MARKER_INSET - 1,
                                            MARKER_INSET + MARKER_SIZE - 1, 0x07e0), TAG,
                                "Draw rotation marker TR");
            ESP_RETURN_ON_ERROR(fill_region(MARKER_INSET, DISPLAY_HEIGHT - MARKER_INSET - MARKER_SIZE,
                                            MARKER_INSET + MARKER_SIZE - 1,
                                            DISPLAY_HEIGHT - MARKER_INSET - 1, 0x001f), TAG,
                                "Draw rotation marker BL");
            ESP_RETURN_ON_ERROR(fill_region(DISPLAY_WIDTH - MARKER_INSET - MARKER_SIZE,
                                            DISPLAY_HEIGHT - MARKER_INSET - MARKER_SIZE,
                                            DISPLAY_WIDTH - MARKER_INSET - 1,
                                            DISPLAY_HEIGHT - MARKER_INSET - 1, 0xffff), TAG,
                                "Draw rotation marker BR");
            ESP_LOGI(TAG, "ROTATION_DEMO cycle=%u madctl=0x%02x", (unsigned)cycle, value);
            vTaskDelay(pdMS_TO_TICKS(1800));
        }
    }
    return ESP_OK;
}

esp_err_t twatch_display_sleep_wake_demo(void)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;
    static const uint8_t dark_brightness[] = {0x00};
    for (unsigned cycle = 0; cycle < 3; ++cycle) {
        ESP_RETURN_ON_ERROR(send_command(0x51, dark_brightness, 1), TAG, "CO5300 dim display");
        ESP_RETURN_ON_ERROR(send_command(0x10, NULL, 0), TAG, "CO5300 sleep");
        ESP_LOGI(TAG, "SLEEP_WAKE cycle=%u state=sleep", cycle);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_RETURN_ON_ERROR(send_command(0x11, NULL, 0), TAG, "CO5300 wake");
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_RETURN_ON_ERROR(send_command(0x29, NULL, 0), TAG, "CO5300 display on");
        ESP_RETURN_ON_ERROR(send_command(0x51, brightness, 1), TAG, "CO5300 restore brightness");
        ESP_LOGI(TAG, "SLEEP_WAKE cycle=%u state=awake", cycle);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return ESP_OK;
}

/* Borrowed, process-lifetime ESP LCD interfaces over the single existing SPI
 * device. These do not allocate another device or change GPIO routing. The
 * caller owns serialization, as with twatch_display_flush(). Only synchronous
 * drawing is supported; async/ISR callback registration is deliberately rejected.
 */
static esp_err_t panel_init(esp_lcd_panel_t *panel)
{
    (void)panel;
    return initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t panel_del(esp_lcd_panel_t *panel)
{
    (void)panel;
    return ESP_ERR_NOT_SUPPORTED; /* Borrowed singleton; board owns its lifetime. */
}

static esp_err_t panel_draw(esp_lcd_panel_t *panel, int x1, int y1, int x2, int y2,
                            const void *data)
{
    (void)panel;
    if (x1 < 0 || y1 < 0 || x2 <= x1 || y2 <= y1 ||
        x2 > DISPLAY_WIDTH || y2 > DISPLAY_HEIGHT) return ESP_ERR_INVALID_ARG;
    return twatch_display_flush(x1, y1, x2 - x1, y2 - y1, data);
}

static esp_err_t panel_invert(esp_lcd_panel_t *panel, bool inverted)
{
    (void)panel;
    return twatch_display_set_inverted(inverted);
}

static esp_err_t io_tx_param(esp_lcd_panel_io_t *io, int command,
                              const void *data, size_t size)
{
    (void)io;
    if (!initialized) return ESP_ERR_INVALID_STATE;
    if (command < 0 || command > 255 || size > 64 * 1024 || (size && !data))
        return ESP_ERR_INVALID_ARG;
    return send_command(command, data, size);
}

static esp_err_t io_tx_color(esp_lcd_panel_io_t *io, int command,
                              const void *data, size_t size)
{
    (void)io;
    if (!initialized) return ESP_ERR_INVALID_STATE;
    if (command != 0x2c) return ESP_ERR_NOT_SUPPORTED;
    if (!data || !size || (size & 1) || size > DISPLAY_WIDTH * DISPLAY_HEIGHT * 2)
        return ESP_ERR_INVALID_ARG;
    return send_pixels(data, size / 2);
}

static esp_err_t io_del(esp_lcd_panel_io_t *io)
{
    (void)io;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t io_register_callbacks(esp_lcd_panel_io_t *io,
                                        const esp_lcd_panel_io_callbacks_t *callbacks, void *ctx)
{
    (void)io;
    (void)callbacks;
    (void)ctx;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_lcd_panel_t lcd_panel = {
    .init = panel_init, .del = panel_del, .draw_bitmap = panel_draw,
    .invert_color = panel_invert,
};
static esp_lcd_panel_io_t lcd_io = {
    .tx_param = io_tx_param, .tx_color = io_tx_color, .del = io_del,
    .register_event_callbacks = io_register_callbacks,
};

esp_lcd_panel_handle_t twatch_display_get_panel(void)
{
    return initialized ? &lcd_panel : NULL;
}

esp_lcd_panel_io_handle_t twatch_display_get_panel_io(void)
{
    return initialized ? &lcd_io : NULL;
}
