#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "ble_rtc.h"

#define DISPLAY_BAND_ROWS 32
#define CONTOUR_OUTER_INSET_PIXELS 13
#define CONTOUR_WIDTH_PIXELS 3
#define SAFE_CONTENT_INSET_PIXELS \
    (CONTOUR_OUTER_INSET_PIXELS + CONTOUR_WIDTH_PIXELS)
#define CONTOUR_Y_OFFSET_PIXELS 1
#define TOP_CORNER_RADIUS_EXTRA_PIXELS 3
#define TIME_GLYPH_SCALE 7
#define TIME_GLYPH_WIDTH 5
#define TIME_GLYPH_HEIGHT 7
#define TIME_GLYPH_SPACING 1
#define TIME_TEXT_LENGTH 8
#define TIME_DISPLAY_ROWS (TIME_GLYPH_HEIGHT * TIME_GLYPH_SCALE)

typedef struct {
    uint8_t command;
    uint8_t parameters[4];
    uint8_t length;
} display_init_command_t;

static spi_device_handle_t display_spi;

static const display_init_command_t display_init_commands[] = {
    {0xFE, {0x00}, 0x01},
    {0xC4, {0x80}, 0x01},
    {0x3A, {0x55}, 0x01},
    {0x35, {0x00}, 0x01},
    {0x53, {0x20}, 0x01},
    {0x63, {0xFF}, 0x01},
    {0x2A, {0x00, 0x16, 0x01, 0xAF}, 0x04},
    {0x2B, {0x00, 0x00, 0x01, 0xF5}, 0x04},
    {0x11, {0x00}, 0x80},
    {0x29, {0x00}, 0x80},
    {0x51, {0x00}, 0x01},
};

static esp_err_t i2c_read_register(uint8_t address, uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(BOARD_I2C_PORT, address, &reg, 1,
                                        value, 1, portMAX_DELAY);
}

static esp_err_t i2c_write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(BOARD_I2C_PORT, address, data,
                                      sizeof(data), portMAX_DELAY);
}

static void initialize_i2c(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BOARD_I2C_SDA,
        .scl_io_num = BOARD_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BOARD_I2C_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(BOARD_I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(BOARD_I2C_PORT, config.mode, 0, 0, 0));
}

static void enable_display_power(void)
{
    uint8_t ldo_enable;
    uint8_t expander_config;
    uint8_t expander_output;

    ESP_ERROR_CHECK(i2c_read_register(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_ALDO2_VOLTAGE, &ldo_enable));
    ldo_enable = (ldo_enable & 0xE0) | 28;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_ALDO2_VOLTAGE, ldo_enable));

    ESP_ERROR_CHECK(i2c_read_register(BOARD_XL9555_ADDR,
                                      BOARD_XL9555_OUTPUT0, &expander_output));
    expander_output &= ~(1U << BOARD_XL9555_DRIVER_BIT);
    expander_output &= ~(1U << BOARD_XL9555_DISPLAY_BIT);
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_OUTPUT0, expander_output));

    ESP_ERROR_CHECK(i2c_read_register(BOARD_XL9555_ADDR,
                                      BOARD_XL9555_CONFIG0, &expander_config));
    expander_config &= ~(1U << BOARD_XL9555_DRIVER_BIT);
    expander_config &= ~(1U << BOARD_XL9555_DISPLAY_BIT);
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_CONFIG0, expander_config));

    ESP_ERROR_CHECK(i2c_read_register(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_LDO_ENABLE, &ldo_enable));
    ldo_enable &= ~(1U << BOARD_AXP2101_ALDO2_BIT);
    ESP_ERROR_CHECK(i2c_write_register(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_LDO_ENABLE, ldo_enable));
    vTaskDelay(pdMS_TO_TICKS(20));

    ldo_enable |= 1U << BOARD_AXP2101_ALDO2_BIT;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_LDO_ENABLE, ldo_enable));
    vTaskDelay(pdMS_TO_TICKS(20));

    expander_output |= 1U << BOARD_XL9555_DRIVER_BIT;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_OUTPUT0, expander_output));
    vTaskDelay(pdMS_TO_TICKS(1));

    expander_output |= 1U << BOARD_XL9555_DISPLAY_BIT;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_OUTPUT0, expander_output));
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void display_command(uint8_t command, const uint8_t *parameters,
                            size_t length)
{
    const uint8_t wrapper[] = {0x02, 0x00, command, 0x00};
    spi_transaction_t command_transaction = {
        .flags = length == 0 ? 0 : SPI_TRANS_CS_KEEP_ACTIVE,
        .length = sizeof(wrapper) * 8,
        .tx_buffer = wrapper,
    };
    spi_transaction_t parameter_transaction = {
        .length = length * 8,
        .tx_buffer = parameters,
    };

    ESP_ERROR_CHECK(spi_device_acquire_bus(display_spi, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_polling_transmit(display_spi,
                                                &command_transaction));
    if (length != 0) {
        ESP_ERROR_CHECK(spi_device_polling_transmit(display_spi,
                                                    &parameter_transaction));
    }
    spi_device_release_bus(display_spi);
}

static void initialize_display(void)
{
    const spi_bus_config_t bus = {
        .data0_io_num = BOARD_DISPLAY_D0,
        .data1_io_num = BOARD_DISPLAY_D1,
        .sclk_io_num = BOARD_DISPLAY_SCK,
        .data2_io_num = BOARD_DISPLAY_D2,
        .data3_io_num = BOARD_DISPLAY_D3,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 0x40000 + 8,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };
    const spi_device_interface_config_t device = {
        .mode = 0,
        .clock_speed_hz = BOARD_DISPLAY_QSPI_HZ,
        .spics_io_num = BOARD_DISPLAY_CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 10,
    };

    ESP_ERROR_CHECK(gpio_set_direction(BOARD_DISPLAY_RESET, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_DISPLAY_RESET, 1));
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_DISPLAY_RESET, 0));
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_DISPLAY_RESET, 1));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(spi_bus_initialize(BOARD_DISPLAY_SPI_HOST, &bus,
                                       SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(BOARD_DISPLAY_SPI_HOST, &device,
                                       &display_spi));

    for (int pass = 0; pass < 2; pass++) {
        for (size_t i = 0;
             i < sizeof(display_init_commands) / sizeof(display_init_commands[0]);
             i++) {
            const display_init_command_t *entry = &display_init_commands[i];
            display_command(entry->command, entry->parameters,
                            entry->length & 0x1F);
            if (entry->length & 0x80) {
                vTaskDelay(pdMS_TO_TICKS(120));
            }
        }
    }

    const uint8_t portrait = 0x00;
    const uint8_t maximum_brightness = 0xFF;
    display_command(0x36, &portrait, 1);
    display_command(0x51, &maximum_brightness, 1);
}

static bool pixel_is_safe_at_inset(int x, int y, int inset)
{
    const int32_t x_um = ((2 * x + 1) * BOARD_ACTIVE_WIDTH_UM) /
                         (2 * BOARD_DISPLAY_WIDTH);
    const int32_t y_um = ((2 * y + 1) * BOARD_ACTIVE_HEIGHT_UM) /
                         (2 * BOARD_DISPLAY_HEIGHT);
    const int32_t inset_um = ((int64_t)inset * BOARD_ACTIVE_WIDTH_UM) /
                             BOARD_DISPLAY_WIDTH;
    const int32_t top_radius_extra_um =
        ((int64_t)TOP_CORNER_RADIUS_EXTRA_PIXELS * BOARD_ACTIVE_WIDTH_UM) /
        BOARD_DISPLAY_WIDTH;

    if (x_um < inset_um || x_um > BOARD_ACTIVE_WIDTH_UM - inset_um ||
        y_um < inset_um || y_um > BOARD_ACTIVE_HEIGHT_UM - inset_um) {
        return false;
    }

    const int32_t top_radius = BOARD_TOP_RADIUS_UM - inset_um +
                               top_radius_extra_um;
    const int32_t bottom_radius = BOARD_BOTTOM_RADIUS_UM - inset_um;
    const int32_t top_center = BOARD_TOP_RADIUS_UM + top_radius_extra_um;
    int64_t dx;
    int64_t dy;

    if (x_um < top_center && y_um < top_center) {
        dx = (int64_t)x_um - top_center;
        dy = (int64_t)y_um - top_center;
        return dx * dx + dy * dy <= (int64_t)top_radius * top_radius;
    }
    if (x_um > BOARD_ACTIVE_WIDTH_UM - top_center && y_um < top_center) {
        dx = (int64_t)x_um -
             (BOARD_ACTIVE_WIDTH_UM - top_center);
        dy = (int64_t)y_um - top_center;
        return dx * dx + dy * dy <= (int64_t)top_radius * top_radius;
    }
    if (x_um < BOARD_BOTTOM_RADIUS_UM &&
        y_um > BOARD_ACTIVE_HEIGHT_UM - BOARD_BOTTOM_RADIUS_UM) {
        dx = (int64_t)x_um - BOARD_BOTTOM_RADIUS_UM;
        dy = (int64_t)y_um -
             (BOARD_ACTIVE_HEIGHT_UM - BOARD_BOTTOM_RADIUS_UM);
        return dx * dx + dy * dy <= (int64_t)bottom_radius * bottom_radius;
    }
    if (x_um > BOARD_ACTIVE_WIDTH_UM - BOARD_BOTTOM_RADIUS_UM &&
        y_um > BOARD_ACTIVE_HEIGHT_UM - BOARD_BOTTOM_RADIUS_UM) {
        dx = (int64_t)x_um -
             (BOARD_ACTIVE_WIDTH_UM - BOARD_BOTTOM_RADIUS_UM);
        dy = (int64_t)y_um -
             (BOARD_ACTIVE_HEIGHT_UM - BOARD_BOTTOM_RADIUS_UM);
        return dx * dx + dy * dy <= (int64_t)bottom_radius * bottom_radius;
    }
    return true;
}

/* The blue contour is excluded: only its enclosed black interior is safe. */
static bool pixel_is_safe_content(int x, int y)
{
    const int shape_y = y - CONTOUR_Y_OFFSET_PIXELS;

    return shape_y >= 0 &&
           pixel_is_safe_at_inset(x, shape_y, SAFE_CONTENT_INSET_PIXELS);
}

static void set_address_window(int y, int rows)
{
    const uint8_t columns[] = {0x00, 0x16, 0x01, 0xAF};
    const int end_y = y + rows - 1;
    const uint8_t row_address[] = {
        (uint8_t)(y >> 8), (uint8_t)y,
        (uint8_t)(end_y >> 8), (uint8_t)end_y,
    };
    display_command(0x2A, columns, sizeof(columns));
    display_command(0x2B, row_address, sizeof(row_address));
}

static void display_color_band(const uint16_t *pixels, size_t pixel_count)
{
    const uint8_t wrapper[] = {0x32, 0x00, 0x2C, 0x00};
    spi_transaction_t command_transaction = {
        .flags = SPI_TRANS_CS_KEEP_ACTIVE,
        .length = sizeof(wrapper) * 8,
        .tx_buffer = wrapper,
    };
    spi_transaction_t color_transaction = {
        .flags = SPI_TRANS_MODE_QIO,
        .length = pixel_count * 16,
        .tx_buffer = pixels,
    };

    ESP_ERROR_CHECK(spi_device_acquire_bus(display_spi, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_polling_transmit(display_spi,
                                                &command_transaction));
    ESP_ERROR_CHECK(spi_device_polling_transmit(display_spi,
                                                &color_transaction));
    spi_device_release_bus(display_spi);
}

static void display_pixel_rows(const uint16_t *pixels, int start_y, int rows)
{
    for (int row_offset = 0; row_offset < rows;
         row_offset += DISPLAY_BAND_ROWS) {
        int rows_in_band = rows - row_offset;
        if (rows_in_band > DISPLAY_BAND_ROWS) {
            rows_in_band = DISPLAY_BAND_ROWS;
        }

        set_address_window(start_y + row_offset, rows_in_band);
        display_color_band(pixels + row_offset * BOARD_DISPLAY_WIDTH,
                           BOARD_DISPLAY_WIDTH * rows_in_band);
    }
}

static void draw_validation_pattern(void)
{
    const size_t band_pixels = BOARD_DISPLAY_WIDTH * DISPLAY_BAND_ROWS;
    uint16_t *pixels = heap_caps_malloc(band_pixels * sizeof(*pixels),
                                        MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(pixels == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    for (int band_y = 0; band_y < BOARD_DISPLAY_HEIGHT;
         band_y += DISPLAY_BAND_ROWS) {
        int rows = BOARD_DISPLAY_HEIGHT - band_y;
        if (rows > DISPLAY_BAND_ROWS) {
            rows = DISPLAY_BAND_ROWS;
        }
        const size_t pixels_in_band = BOARD_DISPLAY_WIDTH * rows;

        for (size_t i = 0; i < pixels_in_band; i++) {
            const uint32_t index = band_y * BOARD_DISPLAY_WIDTH + i;
            const int x = index % BOARD_DISPLAY_WIDTH;
            const int y = index / BOARD_DISPLAY_WIDTH;
            const int shape_y = y - CONTOUR_Y_OFFSET_PIXELS;
            const bool contour =
                shape_y >= 0 &&
                pixel_is_safe_at_inset(x, shape_y,
                                       CONTOUR_OUTER_INSET_PIXELS) &&
                !pixel_is_safe_content(x, y);
            pixels[i] = contour ? 0x1F00 : 0x0000;
        }

        set_address_window(band_y, rows);
        display_color_band(pixels, pixels_in_band);
    }

    heap_caps_free(pixels);
}

static uint8_t time_glyph_row(char character, int row)
{
    static const uint8_t digits[10][TIME_GLYPH_HEIGHT] = {
        {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e},
        {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e},
        {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f},
        {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e},
        {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02},
        {0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e},
        {0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e},
        {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
        {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e},
        {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e},
    };
    static const uint8_t colon[TIME_GLYPH_HEIGHT] = {
        0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00,
    };
    static const uint8_t dash[TIME_GLYPH_HEIGHT] = {
        0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00,
    };

    if (character >= '0' && character <= '9') {
        return digits[character - '0'][row];
    }
    if (character == ':') {
        return colon[row];
    }
    return dash[row];
}

static bool pixel_is_time_text(const char text[TIME_TEXT_LENGTH], int x, int y)
{
    const int cell_width = (TIME_GLYPH_WIDTH + TIME_GLYPH_SPACING) *
                           TIME_GLYPH_SCALE;
    const int text_width = TIME_TEXT_LENGTH * cell_width -
                           TIME_GLYPH_SPACING * TIME_GLYPH_SCALE;
    const int start_x = (BOARD_DISPLAY_WIDTH - text_width) / 2;
    const int start_y = (BOARD_DISPLAY_HEIGHT - TIME_DISPLAY_ROWS) / 2;
    const int relative_x = x - start_x;
    const int relative_y = y - start_y;

    if (relative_x < 0 || relative_y < 0 ||
        relative_x >= text_width || relative_y >= TIME_DISPLAY_ROWS) {
        return false;
    }

    const int character_index = relative_x / cell_width;
    const int character_x = relative_x % cell_width;
    if (character_index >= TIME_TEXT_LENGTH ||
        character_x >= TIME_GLYPH_WIDTH * TIME_GLYPH_SCALE) {
        return false;
    }

    const int glyph_row = relative_y / TIME_GLYPH_SCALE;
    const int glyph_column = character_x / TIME_GLYPH_SCALE;
    const uint8_t row_bits = time_glyph_row(text[character_index], glyph_row);
    return (row_bits & (1U << (TIME_GLYPH_WIDTH - 1 - glyph_column))) != 0;
}

static void display_time_task(void *parameter)
{
    (void)parameter;
    const int start_y = (BOARD_DISPLAY_HEIGHT - TIME_DISPLAY_ROWS) / 2;
    const size_t pixel_count = BOARD_DISPLAY_WIDTH * TIME_DISPLAY_ROWS;
    uint16_t *pixels = heap_caps_malloc(pixel_count * sizeof(*pixels),
                                        MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(pixels == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    TickType_t last_update = xTaskGetTickCount();

    for (;;) {
        char payload[20];
        char time_text[TIME_TEXT_LENGTH + 1] = "--:--:--";
        if (ble_rtc_get_time_payload(payload, sizeof(payload)) == ESP_OK) {
            memcpy(time_text, payload + 11, TIME_TEXT_LENGTH);
        }

        for (size_t i = 0; i < pixel_count; i++) {
            const int x = i % BOARD_DISPLAY_WIDTH;
            const int y = start_y + i / BOARD_DISPLAY_WIDTH;
            const int shape_y = y - CONTOUR_Y_OFFSET_PIXELS;
            const bool contour =
                shape_y >= 0 &&
                pixel_is_safe_at_inset(x, shape_y,
                                       CONTOUR_OUTER_INSET_PIXELS) &&
                !pixel_is_safe_content(x, y);
            const bool text = pixel_is_safe_content(x, y) &&
                              pixel_is_time_text(time_text, x, y);
            pixels[i] = text ? 0xffff : (contour ? 0x1f00 : 0x0000);
        }

        display_pixel_rows(pixels, start_y, TIME_DISPLAY_ROWS);
        vTaskDelayUntil(&last_update, pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    initialize_i2c();
    ESP_ERROR_CHECK(ble_rtc_initialize());
    enable_display_power();
    initialize_display();
    draw_validation_pattern();
    ESP_ERROR_CHECK(ble_rtc_start());
    BaseType_t task_result = xTaskCreate(display_time_task, "display_time",
                                         4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(task_result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
