#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

#include "ble_rtc.h"
#include "board.h"
#include "cascadia_code_72.h"
#include "screen_control.h"

#define DISPLAY_BAND_ROWS 32
#define CONTOUR_OUTER_INSET_PIXELS 13
#define CONTOUR_WIDTH_PIXELS 3
#define SAFE_CONTENT_INSET_PIXELS 16
#define CONTOUR_Y_OFFSET_PIXELS 1
#define TOP_CORNER_RADIUS_EXTRA_PIXELS 3
#define SCREEN_IDLE_TICKS pdMS_TO_TICKS(10000)
#define UI_EVENT_REFRESH (1U << 0)
#define UI_EVENT_TOUCH   (1U << 1)
#define UI_EVENT_ASSETS_READY (1U << 2)
#define ICON_SIZE 48
#define ICON_COUNT 11
#define ICON_ATLAS_BYTES (ICON_COUNT * ICON_SIZE * ICON_SIZE * 2)
#define ICON_ATLAS_PATH "/sdcard/ultrawatch/ui/icons.rgb565"

typedef enum {
    UI_WATCH,
    UI_LAUNCHER,
    UI_SETTINGS,
    UI_BLACK,
} ui_screen_t;

typedef enum {
    ICON_LAUNCHER,
    ICON_CLOCK,
    ICON_SETTINGS,
    ICON_ACTIVITY,
    ICON_HEART,
    ICON_SLEEP,
    ICON_WELLNESS,
    ICON_WEATHER,
    ICON_MUSIC,
    ICON_MESSAGES,
    ICON_RINGS,
} icon_id_t;

typedef struct {
    uint8_t command;
    uint8_t parameters[4];
    uint8_t length;
} display_init_command_t;

typedef struct {
    int x;
    int y;
    int radius;
    icon_id_t icon;
} bubble_t;

typedef struct {
    bool wake;
    bool pending[4];
    uint16_t x[4];
    uint16_t y[4];
} ui_input_mailbox_t;

static const char *TAG = "window_manager";
static spi_device_handle_t display_spi;
static TaskHandle_t ui_task_handle;
static TaskHandle_t bootstrap_task_handle;
static uint16_t *band_pixels;
static uint8_t *icon_atlas;
static volatile uint8_t display_brightness_percentage = 50;
static volatile ui_screen_t active_screen = UI_WATCH;
static bool consume_touch_until_up;
static TickType_t last_touch_tick;
static int64_t button_down_us;
static int16_t content_left[BOARD_DISPLAY_HEIGHT];
static int16_t content_right[BOARD_DISPLAY_HEIGHT];
static int16_t contour_left[BOARD_DISPLAY_HEIGHT];
static int16_t contour_right[BOARD_DISPLAY_HEIGHT];
static portMUX_TYPE ui_input_lock = portMUX_INITIALIZER_UNLOCKED;
static ui_input_mailbox_t ui_input;

static const display_init_command_t display_init_commands[] = {
    {0xFE, {0x00}, 0x01}, {0xC4, {0x80}, 0x01},
    {0x3A, {0x55}, 0x01}, {0x35, {0x00}, 0x01},
    {0x53, {0x20}, 0x01}, {0x55, {0x03}, 0x01},
    {0x63, {0xFF}, 0x01}, {0x2A, {0x00, 0x16, 0x01, 0xAF}, 0x04},
    {0x2B, {0x00, 0x00, 0x01, 0xF5}, 0x04},
    {0x11, {0x00}, 0x80}, {0x29, {0x00}, 0x80},
    {0x51, {0x00}, 0x01},
};

static const bubble_t launcher_bubbles[] = {
    {205, 92, 42, ICON_ACTIVITY}, {110, 143, 42, ICON_HEART},
    {300, 143, 42, ICON_SLEEP}, {73, 241, 42, ICON_WELLNESS},
    {337, 241, 42, ICON_WEATHER}, {111, 340, 42, ICON_MUSIC},
    {299, 340, 42, ICON_MESSAGES}, {205, 385, 42, ICON_RINGS},
    {205, 235, 70, ICON_CLOCK}, {82, 410, 30, ICON_SETTINGS},
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
    uint8_t value;
    ESP_ERROR_CHECK(i2c_read_register(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_ALDO2_VOLTAGE, &value));
    value = (value & 0xE0) | 28;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_ALDO2_VOLTAGE, value));

    ESP_ERROR_CHECK(i2c_read_register(BOARD_XL9555_ADDR,
                                      BOARD_XL9555_OUTPUT0, &value));
    value &= ~((1U << BOARD_XL9555_DRIVER_BIT) |
               (1U << BOARD_XL9555_DISPLAY_BIT));
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_OUTPUT0, value));
    ESP_ERROR_CHECK(i2c_read_register(BOARD_XL9555_ADDR,
                                      BOARD_XL9555_CONFIG0, &value));
    value &= ~((1U << BOARD_XL9555_DRIVER_BIT) |
               (1U << BOARD_XL9555_DISPLAY_BIT));
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_CONFIG0, value));

    ESP_ERROR_CHECK(i2c_read_register(BOARD_AXP2101_ADDR,
                                      BOARD_AXP2101_LDO_ENABLE, &value));
    value &= ~(1U << BOARD_AXP2101_ALDO2_BIT);
    ESP_ERROR_CHECK(i2c_write_register(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_LDO_ENABLE, value));
    vTaskDelay(pdMS_TO_TICKS(20));
    value |= 1U << BOARD_AXP2101_ALDO2_BIT;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_AXP2101_ADDR,
                                       BOARD_AXP2101_LDO_ENABLE, value));
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(i2c_read_register(BOARD_XL9555_ADDR,
                                      BOARD_XL9555_OUTPUT0, &value));
    value |= 1U << BOARD_XL9555_DRIVER_BIT;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_OUTPUT0, value));
    vTaskDelay(pdMS_TO_TICKS(1));
    value |= 1U << BOARD_XL9555_DISPLAY_BIT;
    ESP_ERROR_CHECK(i2c_write_register(BOARD_XL9555_ADDR,
                                       BOARD_XL9555_OUTPUT0, value));
    vTaskDelay(pdMS_TO_TICKS(20));
}

static esp_err_t display_command(uint8_t command, const uint8_t *parameters,
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
    esp_err_t result = spi_device_acquire_bus(display_spi, portMAX_DELAY);
    if (result == ESP_OK) {
        result = spi_device_polling_transmit(display_spi, &command_transaction);
        if (result == ESP_OK && length != 0) {
            result = spi_device_polling_transmit(display_spi,
                                                 &parameter_transaction);
        }
        spi_device_release_bus(display_spi);
    }
    return result;
}

esp_err_t screen_set_brightness(uint8_t percentage)
{
    if (percentage > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    if (display_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t panel_level =
        (uint8_t)(((unsigned)percentage * 255U + 50U) / 100U);
    esp_err_t result = display_command(0x51, &panel_level, 1);
    if (result == ESP_OK) {
        display_brightness_percentage = percentage;
    }
    return result;
}

uint8_t screen_get_brightness(void)
{
    return display_brightness_percentage;
}

void screen_request_refresh(void)
{
    if (ui_task_handle != NULL) {
        xTaskNotify(ui_task_handle, UI_EVENT_REFRESH, eSetBits);
    }
}

void screen_wake_from_touch(void)
{
    portENTER_CRITICAL(&ui_input_lock);
    ui_input.wake = true;
    portEXIT_CRITICAL(&ui_input_lock);
    if (ui_task_handle != NULL) {
        xTaskNotify(ui_task_handle, UI_EVENT_TOUCH, eSetBits);
    }
}

static bool point_in_circle(int x, int y, int center_x, int center_y, int radius)
{
    const int dx = x - center_x;
    const int dy = y - center_y;
    return dx * dx + dy * dy <= radius * radius;
}

void screen_handle_touch(screen_touch_event_t event, uint16_t x, uint16_t y)
{
    if (event < SCREEN_TOUCH_DOWN || event > SCREEN_TOUCH_UP) {
        return;
    }
    portENTER_CRITICAL(&ui_input_lock);
    ui_input.pending[event] = true;
    ui_input.x[event] = x;
    ui_input.y[event] = y;
    portEXIT_CRITICAL(&ui_input_lock);
    if (ui_task_handle != NULL) {
        xTaskNotify(ui_task_handle, UI_EVENT_TOUCH, eSetBits);
    }
}

static bool touch_is_button(ui_screen_t screen, uint16_t x, uint16_t y)
{
    return (screen == UI_WATCH && point_in_circle(x, y, 205, 410, 34)) ||
           (screen == UI_LAUNCHER &&
            (point_in_circle(x, y, 205, 235, 70) ||
             point_in_circle(x, y, 82, 410, 30))) ||
           (screen == UI_SETTINGS &&
            (point_in_circle(x, y, 205, 420, 34) ||
             (x >= 70 && x <= 340 && y >= 270 && y <= 334)));
}

static bool process_touch_event(screen_touch_event_t event, uint16_t x,
                                uint16_t y)
{
    last_touch_tick = xTaskGetTickCount();
    if (consume_touch_until_up) {
        if (event == SCREEN_TOUCH_UP) {
            consume_touch_until_up = false;
        }
        return false;
    }

    if (event == SCREEN_TOUCH_DOWN && touch_is_button(active_screen, x, y)) {
        button_down_us = esp_timer_get_time();
        esp_err_t result = ble_haptic_click();
        if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "UI haptic click failed: %s",
                     esp_err_to_name(result));
        }
    }

    if (active_screen == UI_SETTINGS &&
        (event == SCREEN_TOUCH_DOWN || event == SCREEN_TOUCH_MOVE) &&
        y >= 155 && y <= 215) {
        int clamped_x = x < 48 ? 48 : (x > 362 ? 362 : x);
        uint8_t brightness = (uint8_t)(((clamped_x - 48) * 100 + 157) / 314);
        if (screen_set_brightness(brightness) != ESP_OK) {
            ESP_LOGW(TAG, "brightness update failed");
        }
        return true;
    }
    if (event != SCREEN_TOUCH_UP) {
        return false;
    }

    bool changed = false;
    if (active_screen == UI_WATCH && point_in_circle(x, y, 205, 410, 34)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_LAUNCHER &&
               point_in_circle(x, y, 205, 235, 70)) {
        active_screen = UI_WATCH;
        changed = true;
    } else if (active_screen == UI_LAUNCHER &&
               point_in_circle(x, y, 82, 410, 30)) {
        active_screen = UI_SETTINGS;
        changed = true;
    } else if (active_screen == UI_SETTINGS &&
               point_in_circle(x, y, 205, 420, 34)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_SETTINGS &&
               x >= 70 && x <= 340 && y >= 270 && y <= 334) {
        esp_err_t result = ble_rtc_set_advertising_enabled(
            !ble_rtc_advertising_enabled());
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "BLE advertising update failed: %s",
                     esp_err_to_name(result));
        } else {
            changed = true;
        }
    }
    return changed;
}

static bool process_ui_input(void)
{
    ui_input_mailbox_t input;
    bool changed = false;
    portENTER_CRITICAL(&ui_input_lock);
    input = ui_input;
    memset(&ui_input, 0, sizeof(ui_input));
    portEXIT_CRITICAL(&ui_input_lock);

    if (input.wake) {
        last_touch_tick = xTaskGetTickCount();
        if (active_screen == UI_BLACK) {
            active_screen = UI_WATCH;
            consume_touch_until_up = true;
            changed = true;
        }
    }
    for (int event = SCREEN_TOUCH_DOWN; event <= SCREEN_TOUCH_UP; event++) {
        if (input.pending[event]) {
            changed |= process_touch_event((screen_touch_event_t)event,
                                           input.x[event], input.y[event]);
        }
    }
    return changed;
}

static void initialize_display(void)
{
    const spi_bus_config_t bus = {
        .data0_io_num = BOARD_DISPLAY_D0,
        .data1_io_num = BOARD_DISPLAY_D1,
        .sclk_io_num = BOARD_DISPLAY_SCK,
        .data2_io_num = BOARD_DISPLAY_D2,
        .data3_io_num = BOARD_DISPLAY_D3,
        .data4_io_num = -1, .data5_io_num = -1,
        .data6_io_num = -1, .data7_io_num = -1,
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
        for (size_t i = 0; i < sizeof(display_init_commands) /
                                sizeof(display_init_commands[0]); i++) {
            const display_init_command_t *entry = &display_init_commands[i];
            ESP_ERROR_CHECK(display_command(entry->command, entry->parameters,
                                            entry->length & 0x1F));
            if (entry->length & 0x80) {
                vTaskDelay(pdMS_TO_TICKS(120));
            }
        }
    }
    ESP_ERROR_CHECK(display_command(0x36, &(uint8_t){0}, 1));
    ESP_ERROR_CHECK(screen_set_brightness(display_brightness_percentage));
}

static bool pixel_is_safe_at_inset(int x, int y, int inset)
{
    const int32_t x_um = ((2 * x + 1) * BOARD_ACTIVE_WIDTH_UM) /
                         (2 * BOARD_DISPLAY_WIDTH);
    const int32_t y_um = ((2 * y + 1) * BOARD_ACTIVE_HEIGHT_UM) /
                         (2 * BOARD_DISPLAY_HEIGHT);
    const int32_t inset_um = ((int64_t)inset * BOARD_ACTIVE_WIDTH_UM) /
                             BOARD_DISPLAY_WIDTH;
    const int32_t top_extra =
        ((int64_t)TOP_CORNER_RADIUS_EXTRA_PIXELS * BOARD_ACTIVE_WIDTH_UM) /
        BOARD_DISPLAY_WIDTH;
    if (x_um < inset_um || x_um > BOARD_ACTIVE_WIDTH_UM - inset_um ||
        y_um < inset_um || y_um > BOARD_ACTIVE_HEIGHT_UM - inset_um) {
        return false;
    }
    const int32_t top_radius = BOARD_TOP_RADIUS_UM - inset_um + top_extra;
    const int32_t bottom_radius = BOARD_BOTTOM_RADIUS_UM - inset_um;
    const int32_t top_center = BOARD_TOP_RADIUS_UM + top_extra;
    int64_t dx;
    int64_t dy;
    if (x_um < top_center && y_um < top_center) {
        dx = (int64_t)x_um - top_center; dy = (int64_t)y_um - top_center;
        return dx * dx + dy * dy <= (int64_t)top_radius * top_radius;
    }
    if (x_um > BOARD_ACTIVE_WIDTH_UM - top_center && y_um < top_center) {
        dx = (int64_t)x_um - (BOARD_ACTIVE_WIDTH_UM - top_center);
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

static bool pixel_is_safe_content(int x, int y)
{
    return y >= 0 && y < BOARD_DISPLAY_HEIGHT &&
           x >= content_left[y] && x <= content_right[y];
}

static bool pixel_is_contour(int x, int y)
{
    return y >= 0 && y < BOARD_DISPLAY_HEIGHT &&
           x >= contour_left[y] && x <= contour_right[y] &&
           !pixel_is_safe_content(x, y);
}

static void initialize_display_bounds(void)
{
    for (int y = 0; y < BOARD_DISPLAY_HEIGHT; y++) {
        content_left[y] = 1;
        content_right[y] = 0;
        contour_left[y] = 1;
        contour_right[y] = 0;
        int shape_y = y - CONTOUR_Y_OFFSET_PIXELS;
        if (shape_y < 0) {
            continue;
        }
        for (int x = 0; x < BOARD_DISPLAY_WIDTH; x++) {
            if (pixel_is_safe_at_inset(x, shape_y,
                                       CONTOUR_OUTER_INSET_PIXELS)) {
                contour_left[y] = x;
                break;
            }
        }
        for (int x = BOARD_DISPLAY_WIDTH - 1; x >= contour_left[y]; x--) {
            if (pixel_is_safe_at_inset(x, shape_y,
                                       CONTOUR_OUTER_INSET_PIXELS)) {
                contour_right[y] = x;
                break;
            }
        }
        for (int x = contour_left[y]; x <= contour_right[y]; x++) {
            if (pixel_is_safe_at_inset(x, shape_y,
                                       SAFE_CONTENT_INSET_PIXELS)) {
                content_left[y] = x;
                break;
            }
        }
        for (int x = contour_right[y]; x >= content_left[y]; x--) {
            if (pixel_is_safe_at_inset(x, shape_y,
                                       SAFE_CONTENT_INSET_PIXELS)) {
                content_right[y] = x;
                break;
            }
        }
    }
}

static uint16_t wire_rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t value = ((uint16_t)(red & 0xF8) << 8) |
                           ((uint16_t)(green & 0xFC) << 3) | (blue >> 3);
    return (uint16_t)((value >> 8) | (value << 8));
}

static void set_address_window(int y, int rows)
{
    const uint8_t columns[] = {0x00, 0x16, 0x01, 0xAF};
    const int end_y = y + rows - 1;
    const uint8_t row_address[] = {
        (uint8_t)(y >> 8), (uint8_t)y,
        (uint8_t)(end_y >> 8), (uint8_t)end_y,
    };
    ESP_ERROR_CHECK(display_command(0x2A, columns, sizeof(columns)));
    ESP_ERROR_CHECK(display_command(0x2B, row_address, sizeof(row_address)));
}

static void display_color_band(const uint16_t *pixels, size_t count)
{
    const uint8_t wrapper[] = {0x32, 0x00, 0x2C, 0x00};
    spi_transaction_t command = {
        .flags = SPI_TRANS_CS_KEEP_ACTIVE,
        .length = sizeof(wrapper) * 8, .tx_buffer = wrapper,
    };
    spi_transaction_t color = {
        .flags = SPI_TRANS_MODE_QIO,
        .length = count * 16, .tx_buffer = pixels,
    };
    ESP_ERROR_CHECK(spi_device_acquire_bus(display_spi, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_polling_transmit(display_spi, &command));
    ESP_ERROR_CHECK(spi_device_polling_transmit(display_spi, &color));
    spi_device_release_bus(display_spi);
}

static int glyph_index(char character)
{
    for (int index = 0; index < CASCADIA_CODE_GLYPH_COUNT; index++) {
        if (cascadia_code_characters[index] == character) {
            return index;
        }
    }
    return 0;
}

static bool text_pixel(const char *text, int start_x, int start_y,
                       int divisor, int x, int y)
{
    const int cell_width = (CASCADIA_CODE_CELL_WIDTH + divisor - 1) / divisor;
    const int height = (CASCADIA_CODE_GLYPH_HEIGHT + divisor - 1) / divisor;
    const int rx = x - start_x;
    const int ry = y - start_y;
    const size_t length = strlen(text);
    if (rx < 0 || ry < 0 || rx >= (int)length * cell_width || ry >= height) {
        return false;
    }
    const int column = (rx % cell_width) * divisor;
    const int row = ry * divisor;
    if (column >= CASCADIA_CODE_CELL_WIDTH || row >= CASCADIA_CODE_GLYPH_HEIGHT) {
        return false;
    }
    const uint8_t bits = cascadia_code_glyphs[glyph_index(text[rx / cell_width])]
                                                [row][column / 8];
    return (bits & (1U << (7 - column % 8))) != 0;
}

static int centered_text_x(const char *text, int divisor)
{
    int cell_width = (CASCADIA_CODE_CELL_WIDTH + divisor - 1) / divisor;
    return (BOARD_DISPLAY_WIDTH - (int)strlen(text) * cell_width) / 2;
}

static uint16_t icon_pixel(icon_id_t icon, int center_x, int center_y,
                           int x, int y)
{
    int ix = x - (center_x - ICON_SIZE / 2);
    int iy = y - (center_y - ICON_SIZE / 2);
    if (ix < 0 || iy < 0 || ix >= ICON_SIZE || iy >= ICON_SIZE) {
        return 0;
    }
    if (icon_atlas != NULL) {
        size_t offset = ((size_t)icon * ICON_SIZE * ICON_SIZE +
                         iy * ICON_SIZE + ix) * 2;
        if (icon_atlas[offset] == 0 && icon_atlas[offset + 1] == 0) {
            return 0;
        }
        return (uint16_t)icon_atlas[offset] |
               ((uint16_t)icon_atlas[offset + 1] << 8);
    }

    int dx = ix - ICON_SIZE / 2;
    int dy = iy - ICON_SIZE / 2;
    if (icon == ICON_LAUNCHER) {
        if (abs(dx) <= 15 && abs(dy) <= 15 &&
            ((abs(dx + 10) <= 3 || abs(dx) <= 3 || abs(dx - 10) <= 3) &&
             (abs(dy + 10) <= 3 || abs(dy) <= 3 || abs(dy - 10) <= 3))) {
            return wire_rgb565(36, 132, 255);
        }
    } else if (icon == ICON_CLOCK) {
        int d2 = dx * dx + dy * dy;
        if ((d2 >= 225 && d2 <= 324) ||
            (abs(dx) <= 2 && dy <= 2 && dy >= -12) ||
            (abs(dy - dx / 2) <= 2 && dx >= 0 && dx <= 11)) {
            return wire_rgb565(90, 164, 255);
        }
    } else if (icon == ICON_SETTINGS) {
        int d2 = dx * dx + dy * dy;
        if ((d2 >= 90 && d2 <= 200) ||
            ((abs(dx) <= 3 || abs(dy) <= 3) && d2 <= 300 && d2 >= 180)) {
            return wire_rgb565(95, 150, 255);
        }
    }
    return 0;
}

static uint16_t icon_pixel_sized(icon_id_t icon, int center_x, int center_y,
                                 int display_size, int x, int y)
{
    int ix = x - (center_x - display_size / 2);
    int iy = y - (center_y - display_size / 2);
    if (ix < 0 || iy < 0 || ix >= display_size || iy >= display_size) {
        return 0;
    }
    int source_x = ix * ICON_SIZE / display_size;
    int source_y = iy * ICON_SIZE / display_size;
    return icon_pixel(icon, center_x, center_y,
                      center_x - ICON_SIZE / 2 + source_x,
                      center_y - ICON_SIZE / 2 + source_y);
}

static uint16_t glass_bubble_pixel(const bubble_t *bubble, int x, int y)
{
    int dx = x - bubble->x;
    int dy = y - bubble->y;
    int d2 = dx * dx + dy * dy;
    int r2 = bubble->radius * bubble->radius;
    if (d2 > (bubble->radius + 4) * (bubble->radius + 4)) {
        return 0;
    }
    if (d2 >= (bubble->radius - 2) * (bubble->radius - 2)) {
        return wire_rgb565(45, 102, 190);
    }
    if (d2 <= r2) {
        uint8_t blue = (uint8_t)(12 + (r2 - d2) * 18 / r2);
        return wire_rgb565(3, 8, blue);
    }
    return wire_rgb565(5, 30, 75);
}

static void read_watch_values(rtc_datetime_t *datetime, char time[6],
                              char date[12], char battery[5])
{
    static const char *weekdays[] = {"SO", "MO", "DI", "MI", "DO", "FR", "SA"};
    static const char *months[] = {
        "JAN", "FEB", "MRZ", "APR", "MAI", "JUN",
        "JUL", "AUG", "SEP", "OKT", "NOV", "DEZ",
    };
    strcpy(time, "--:--");
    strcpy(date, "-- -- ---");
    strcpy(battery, "--%");
    if (ble_rtc_get_datetime(datetime) == ESP_OK) {
        snprintf(time, 6, "%02d:%02d", datetime->hour, datetime->minute);
        snprintf(date, 12, "%s %02d %s", weekdays[datetime->weekday],
                 datetime->day, months[datetime->month - 1]);
    } else {
        memset(datetime, 0, sizeof(*datetime));
    }
    char payload[24];
    unsigned percentage;
    if (ble_power_get_payload(payload, sizeof(payload)) == ESP_OK &&
        sscanf(payload, "%u,", &percentage) == 1 && percentage <= 100) {
        snprintf(battery, 5, "%u%%", percentage);
    }
}

static uint16_t render_watch_pixel(int x, int y, const char *time,
                                   const char *date, const char *battery)
{
    if (text_pixel(time, centered_text_x(time, 1), 174, 1, x, y)) {
        return wire_rgb565(244, 248, 255);
    }
    if (text_pixel(date, centered_text_x(date, 3), 265, 3, x, y)) {
        return wire_rgb565(74, 137, 255);
    }
    bubble_t launcher = {205, 410, 34, ICON_LAUNCHER};
    uint16_t color = glass_bubble_pixel(&launcher, x, y);
    uint16_t icon = icon_pixel(ICON_LAUNCHER, 205, 410, x, y);
    if (icon != 0) {
        return icon;
    }
    if (color != 0) {
        return color;
    }
    const char *ble = ble_rtc_advertising_enabled() ? "BLE AN" : "BLE AUS";
    if (text_pixel(ble, 52, 450, 4, x, y) ||
        text_pixel(battery, BOARD_DISPLAY_WIDTH - 52 -
                   (int)strlen(battery) *
                   ((CASCADIA_CODE_CELL_WIDTH + 3) / 4), 450, 4, x, y)) {
        return wire_rgb565(170, 196, 230);
    }
    return 0;
}

static uint16_t render_launcher_pixel(int x, int y, const char *battery)
{
    uint16_t color = 0;
    if (((x * x + y * 7 + (x - y) * (x - y) / 8) % 43) < 2) {
        color = wire_rgb565(2, 10, 24);
    }
    for (size_t index = 0; index < sizeof(launcher_bubbles) /
                                      sizeof(launcher_bubbles[0]); index++) {
        uint16_t bubble_color = glass_bubble_pixel(&launcher_bubbles[index], x, y);
        if (bubble_color != 0) {
            color = bubble_color;
        }
        uint16_t icon = launcher_bubbles[index].icon == ICON_CLOCK
                            ? icon_pixel_sized(ICON_CLOCK,
                                               launcher_bubbles[index].x,
                                               launcher_bubbles[index].y,
                                               72, x, y)
                            : icon_pixel(launcher_bubbles[index].icon,
                                         launcher_bubbles[index].x,
                                         launcher_bubbles[index].y, x, y);
        if (icon != 0) {
            color = icon;
        }
    }
    if (text_pixel(battery, 346 -
                   ((int)strlen(battery) * ((CASCADIA_CODE_CELL_WIDTH + 3) / 4)) / 2,
                   430, 4, x, y)) {
        color = wire_rgb565(105, 238, 166);
    }
    return color;
}

static uint16_t render_settings_pixel(int x, int y)
{
    if (text_pixel("EINSTELLUNGEN", centered_text_x("EINSTELLUNGEN", 3),
                   70, 3, x, y) ||
        text_pixel("HELLIGKEIT", 48, 125, 4, x, y)) {
        return wire_rgb565(105, 160, 255);
    }
    if (y >= 180 && y <= 190 && x >= 48 && x <= 362) {
        int position = 48 + screen_get_brightness() * 314 / 100;
        return x <= position ? wire_rgb565(50, 133, 255)
                             : wire_rgb565(34, 42, 58);
    }
    int position = 48 + screen_get_brightness() * 314 / 100;
    if ((x - position) * (x - position) + (y - 185) * (y - 185) <= 144) {
        return wire_rgb565(144, 196, 255);
    }
    if (x >= 70 && x <= 340 && y >= 270 && y <= 334) {
        bool on = ble_rtc_advertising_enabled();
        bool edge = x < 73 || x > 337 || y < 273 || y > 331;
        if (edge) {
            return on ? wire_rgb565(55, 151, 255) : wire_rgb565(70, 78, 92);
        }
        if (text_pixel(on ? "BLUETOOTH AN" : "BLUETOOTH AUS",
                       centered_text_x(on ? "BLUETOOTH AN" : "BLUETOOTH AUS", 3),
                       294, 3, x, y)) {
            return wire_rgb565(240, 246, 255);
        }
        return wire_rgb565(3, 18, on ? 48 : 22);
    }
    bubble_t launcher = {205, 420, 34, ICON_LAUNCHER};
    uint16_t icon = icon_pixel(ICON_LAUNCHER, 205, 420, x, y);
    if (icon != 0) {
        return icon;
    }
    return glass_bubble_pixel(&launcher, x, y);
}

static int64_t render_screen(ui_screen_t screen)
{
    int64_t started_us = esp_timer_get_time();
    rtc_datetime_t datetime;
    char time[6];
    char date[12];
    char battery[5];
    read_watch_values(&datetime, time, date, battery);
    for (int band_y = 0; band_y < BOARD_DISPLAY_HEIGHT;
         band_y += DISPLAY_BAND_ROWS) {
        int rows = BOARD_DISPLAY_HEIGHT - band_y;
        if (rows > DISPLAY_BAND_ROWS) {
            rows = DISPLAY_BAND_ROWS;
        }
        for (int row = 0; row < rows; row++) {
            int y = band_y + row;
            for (int x = 0; x < BOARD_DISPLAY_WIDTH; x++) {
                uint16_t color = 0;
                if (screen != UI_BLACK && pixel_is_safe_content(x, y)) {
                    if (screen == UI_WATCH) {
                        color = render_watch_pixel(x, y, time, date, battery);
                    } else if (screen == UI_LAUNCHER) {
                        color = render_launcher_pixel(x, y, battery);
                    } else if (screen == UI_SETTINGS) {
                        color = render_settings_pixel(x, y);
                    }
                }
                if (screen != UI_BLACK && pixel_is_contour(x, y)) {
                    color = wire_rgb565(24, 99, 255);
                }
                band_pixels[row * BOARD_DISPLAY_WIDTH + x] = color;
            }
        }
        set_address_window(band_y, rows);
        display_color_band(band_pixels, BOARD_DISPLAY_WIDTH * rows);
    }
    return esp_timer_get_time() - started_us;
}

static TickType_t minute_wait_ticks(void)
{
    rtc_datetime_t datetime;
    if (ble_rtc_get_datetime(&datetime) == ESP_OK) {
        return pdMS_TO_TICKS((60 - datetime.second) * 1000U);
    }
    return pdMS_TO_TICKS(60000);
}

static void ui_task(void *parameter)
{
    (void)parameter;
    last_touch_tick = xTaskGetTickCount();
    int64_t first_frame_us = render_screen(UI_WATCH);
    ESP_LOGI(TAG, "first Watch frame rendered");
    ESP_LOGI(TAG, "Watch frame render: %lld ms", first_frame_us / 1000);
    xTaskNotifyGive(bootstrap_task_handle);
    uint32_t bootstrap_events = 0;
    do {
        xTaskNotifyWait(0, UINT32_MAX, &bootstrap_events, portMAX_DELAY);
    } while ((bootstrap_events & UI_EVENT_ASSETS_READY) == 0);

    bool render_pending = true;
    for (;;) {
        render_pending |= process_ui_input();
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - last_touch_tick;
        if (active_screen != UI_BLACK && elapsed >= SCREEN_IDLE_TICKS) {
            active_screen = UI_BLACK;
            render_screen(UI_BLACK);
            render_pending = false;
            button_down_us = 0;
        }
        if (active_screen == UI_BLACK) {
            uint32_t events;
            xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);
            continue;
        }

        if (render_pending) {
            ui_screen_t screen = active_screen;
            int64_t render_us = render_screen(screen);
            if (button_down_us != 0) {
                int64_t response_us = esp_timer_get_time() - button_down_us;
                ESP_LOGI(TAG, "UI response: %lld ms total, %lld ms rendering",
                         response_us / 1000, render_us / 1000);
                button_down_us = 0;
            } else {
                ESP_LOGD(TAG, "UI frame render: %lld ms", render_us / 1000);
            }
            render_pending = false;
        }
        now = xTaskGetTickCount();
        elapsed = now - last_touch_tick;
        TickType_t idle_wait = elapsed >= SCREEN_IDLE_TICKS
                                   ? 0 : SCREEN_IDLE_TICKS - elapsed;
        TickType_t minute_wait = minute_wait_ticks();
        TickType_t wait = minute_wait < idle_wait ? minute_wait : idle_wait;
        uint32_t events = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &events, wait) == pdFALSE ||
            (events & UI_EVENT_REFRESH) != 0) {
            render_pending = true;
        }
    }
}

static esp_err_t sd_power(bool enabled)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(i2c_read_register(BOARD_AXP2101_ADDR,
                                          BOARD_AXP2101_LDO_ENABLE, &value),
                        TAG, "ALDO1 state read failed");
    if (enabled) {
        uint8_t voltage;
        ESP_RETURN_ON_ERROR(i2c_read_register(BOARD_AXP2101_ADDR,
                                              BOARD_AXP2101_ALDO1_VOLTAGE,
                                              &voltage),
                            TAG, "ALDO1 voltage read failed");
        voltage = (voltage & 0xE0) | 28;
        ESP_RETURN_ON_ERROR(i2c_write_register(BOARD_AXP2101_ADDR,
                                               BOARD_AXP2101_ALDO1_VOLTAGE,
                                               voltage),
                            TAG, "ALDO1 voltage write failed");
        value |= 1U << BOARD_AXP2101_ALDO1_BIT;
    } else {
        value &= ~(1U << BOARD_AXP2101_ALDO1_BIT);
    }
    return i2c_write_register(BOARD_AXP2101_ADDR,
                              BOARD_AXP2101_LDO_ENABLE, value);
}

static bool sd_card_present(void)
{
    uint8_t config;
    uint8_t input;
    if (i2c_read_register(BOARD_XL9555_ADDR, BOARD_XL9555_CONFIG1,
                          &config) != ESP_OK) {
        return false;
    }
    config |= 1U << BOARD_XL9555_SD_DETECT_BIT;
    if (i2c_write_register(BOARD_XL9555_ADDR, BOARD_XL9555_CONFIG1,
                           config) != ESP_OK ||
        i2c_read_register(BOARD_XL9555_ADDR, BOARD_XL9555_INPUT1,
                          &input) != ESP_OK) {
        return false;
    }
    return (input & (1U << BOARD_XL9555_SD_DETECT_BIT)) == 0;
}

static void load_icon_atlas_from_sd(void)
{
    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << BOARD_NFC_CS) | (1ULL << BOARD_LORA_CS) |
                        (1ULL << BOARD_LORA_RESET),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_NFC_CS, 1));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_LORA_CS, 1));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_LORA_RESET, 1));

    ESP_ERROR_CHECK_WITHOUT_ABORT(sd_power(false));
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_ERROR_CHECK_WITHOUT_ABORT(sd_power(true));
    vTaskDelay(pdMS_TO_TICKS(250));
    if (!sd_card_present()) {
        ESP_LOGI(TAG, "no SD card; using procedural UI symbols");
        ESP_ERROR_CHECK_WITHOUT_ABORT(sd_power(false));
        return;
    }

    const spi_bus_config_t bus = {
        .mosi_io_num = BOARD_SD_MOSI,
        .miso_io_num = BOARD_SD_MISO,
        .sclk_io_num = BOARD_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t result = spi_bus_initialize(BOARD_SD_SPI_HOST, &bus,
                                           SPI_DMA_CH_AUTO);
    bool bus_ready = result == ESP_OK;
    bool mounted = false;
    sdmmc_card_t *card = NULL;
    if (result == ESP_OK) {
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        host.slot = BOARD_SD_SPI_HOST;
        host.max_freq_khz = BOARD_SD_SPI_HZ / 1000;
        sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot.host_id = BOARD_SD_SPI_HOST;
        slot.gpio_cs = BOARD_SD_CS;
        const esp_vfs_fat_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 1,
            .allocation_unit_size = 0,
        };
        result = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot,
                                         &mount_config, &card);
        mounted = result == ESP_OK;
    }
    if (mounted) {
        FILE *file = fopen(ICON_ATLAS_PATH, "rb");
        long file_size = -1;
        if (file == NULL) {
            ESP_LOGW(TAG, "cannot open %s: errno=%d (%s)", ICON_ATLAS_PATH,
                     errno, strerror(errno));
        } else if (fseek(file, 0, SEEK_END) != 0 ||
                   (file_size = ftell(file)) < 0 ||
                   fseek(file, 0, SEEK_SET) != 0) {
            ESP_LOGW(TAG, "cannot determine atlas size: errno=%d (%s)",
                     errno, strerror(errno));
        } else if (file_size != ICON_ATLAS_BYTES) {
            ESP_LOGW(TAG, "atlas is %ld bytes; expected %u", file_size,
                     ICON_ATLAS_BYTES);
        } else {
            uint8_t *candidate = heap_caps_malloc(
                ICON_ATLAS_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (candidate != NULL &&
                fread(candidate, 1, ICON_ATLAS_BYTES, file) == ICON_ATLAS_BYTES) {
                icon_atlas = candidate;
                ESP_LOGI(TAG, "loaded %u-byte UI icon atlas", ICON_ATLAS_BYTES);
            } else {
                free(candidate);
                ESP_LOGW(TAG, "UI icon atlas read failed; using fallback symbols");
            }
        }
        if (file != NULL) {
            fclose(file);
        }
        esp_err_t unmount = esp_vfs_fat_sdcard_unmount("/sdcard", card);
        if (unmount != ESP_OK) {
            ESP_LOGW(TAG, "SD unmount failed: %s", esp_err_to_name(unmount));
            free(icon_atlas);
            icon_atlas = NULL;
        }
    } else {
        ESP_LOGW(TAG, "SD mount skipped/failed: %s", esp_err_to_name(result));
    }
    if (bus_ready) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_free(BOARD_SD_SPI_HOST));
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(sd_power(false));
    ESP_LOGI(TAG, "SD power disabled after asset load attempt");
}

void app_main(void)
{
    initialize_i2c();
    ESP_ERROR_CHECK(ble_rtc_initialize());
    enable_display_power();
    initialize_display();
    initialize_display_bounds();
    band_pixels = heap_caps_malloc(
        BOARD_DISPLAY_WIDTH * DISPLAY_BAND_ROWS * sizeof(*band_pixels),
        MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(band_pixels == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    bootstrap_task_handle = xTaskGetCurrentTaskHandle();
    BaseType_t task_result = xTaskCreate(ui_task, "ui", 6144, NULL, 4,
                                         &ui_task_handle);
    ESP_ERROR_CHECK(task_result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    load_icon_atlas_from_sd();
    xTaskNotify(ui_task_handle, UI_EVENT_ASSETS_READY, eSetBits);
    ESP_ERROR_CHECK(ble_rtc_start());
}
