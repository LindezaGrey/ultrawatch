#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
#include "cascadia_time_120.h"
#include "cJSON.h"
#include "offline_map.h"
#include "mesh_service.h"
#include "screen_control.h"
#include "sd_storage.h"
#include "wifi_manager.h"
#include "weather.h"

#ifndef CONFIG_SPIRAM
#error "The cached window manager requires CONFIG_SPIRAM"
#endif

#define DISPLAY_BAND_ROWS 32
#define WATCH_TIME_REGION_X 0
#define WATCH_TIME_DRAW_Y 144
#define WATCH_TIME_REGION_Y (3 * DISPLAY_BAND_ROWS)
#define WATCH_TIME_REGION_WIDTH BOARD_DISPLAY_WIDTH
#define WATCH_TIME_REGION_HEIGHT (6 * DISPLAY_BAND_ROWS)
#define DISPLAY_BUFFER_COUNT 2
#define DISPLAY_TE_WAIT_US 50000
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
#define LEGACY_ICON_COUNT 11
#define PREVIOUS_ICON_COUNT 17
#define MAP_ICON_COUNT 18
#define CURRENT_ICON_COUNT 21
#define ICON_COUNT 25
#define ICON_TILE_BYTES (ICON_SIZE * ICON_SIZE * 2)
#define LEGACY_ICON_ATLAS_BYTES (LEGACY_ICON_COUNT * ICON_TILE_BYTES)
#define PREVIOUS_ICON_ATLAS_BYTES (PREVIOUS_ICON_COUNT * ICON_TILE_BYTES)
#define MAP_ICON_ATLAS_BYTES (MAP_ICON_COUNT * ICON_TILE_BYTES)
#define CURRENT_ICON_ATLAS_BYTES (CURRENT_ICON_COUNT * ICON_TILE_BYTES)
#define ICON_ATLAS_BYTES (ICON_COUNT * ICON_TILE_BYTES)
#define ICON_ATLAS_PATH "/sdcard/ultrawatch/ui/icons.rgb565"
#define WEATHER_PICTURE_SIZE 128
#define WEATHER_PICTURE_COUNT 9
#define WEATHER_PICTURE_BYTES \
    (WEATHER_PICTURE_SIZE * WEATHER_PICTURE_SIZE * 2)
#define WEATHER_ATLAS_BYTES (WEATHER_PICTURE_COUNT * WEATHER_PICTURE_BYTES)
#define WEATHER_ATLAS_PATH "/sdcard/ultrawatch/weather/images.rgb565"
#define ALARM_ROLL_STEP_PIXELS 60
#define DISPLAY_FRAME_BYTES \
    (BOARD_DISPLAY_WIDTH * BOARD_DISPLAY_HEIGHT * sizeof(uint16_t))

typedef enum {
    UI_WATCH,
    UI_LAUNCHER,
    UI_SETTINGS,
    UI_ALARM,
    UI_MAP,
    UI_WEATHER,
    UI_MESSAGES,
    UI_BLACK,
} ui_screen_t;

static bool screen_keeps_display_awake(ui_screen_t screen)
{
    return screen == UI_SETTINGS || screen == UI_ALARM ||
           screen == UI_MAP || screen == UI_WEATHER || screen == UI_MESSAGES;
}

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
    ICON_BLE_OFF,
    ICON_BLE_ON,
    ICON_BATTERY_EMPTY,
    ICON_BATTERY_LOW,
    ICON_BATTERY_MEDIUM,
    ICON_BATTERY_FULL,
    ICON_MAP,
    ICON_ZOOM_IN,
    ICON_ZOOM_OUT,
    ICON_MAP_CENTER,
    ICON_WIFI_OFF,
    ICON_WIFI_CONNECTING,
    ICON_WIFI_CONNECTED,
    ICON_WIFI_ERROR,
} icon_id_t;

typedef enum {
    LAUNCHER_ACTION_NONE,
    LAUNCHER_ACTION_WATCH,
    LAUNCHER_ACTION_SETTINGS,
    LAUNCHER_ACTION_ALARM,
    LAUNCHER_ACTION_MAP,
    LAUNCHER_ACTION_WEATHER,
    LAUNCHER_ACTION_MESSAGES,
} launcher_action_t;

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
    launcher_action_t action;
} bubble_t;

typedef struct {
    bool wake;
    bool alarm_ring;
    bool show_messages;
    bool pending[4];
    uint16_t x[4];
    uint16_t y[4];
} ui_input_mailbox_t;

typedef struct {
    char time[6];
    char date[12];
    char battery[5];
    bool ble_enabled;
    watch_wifi_state_t wifi_state;
    bool alarm_enabled;
} watch_values_t;

typedef struct {
    spi_transaction_t command;
    spi_transaction_t color;
    bool pending;
} display_transfer_t;

static const char *TAG = "window_manager";
static spi_device_handle_t display_spi;
static TaskHandle_t ui_task_handle;
static TaskHandle_t bootstrap_task_handle;
static uint16_t *band_pixels[DISPLAY_BUFFER_COUNT];
static display_transfer_t display_transfer;
static uint8_t *icon_atlas;
static size_t icon_atlas_tile_count;
static uint8_t *weather_atlas;
static uint16_t *watch_frame;
static uint16_t *launcher_frame;
static uint16_t *settings_frame;
static uint16_t *alarm_frame;
static uint16_t *map_frame;
static uint16_t *weather_frame;
static uint16_t *messages_frame;
static volatile uint8_t display_brightness_percentage = 50;
static volatile uint32_t theme_rgb = 0x1863FF;
static bool panel_hidden = true;
static volatile ui_screen_t active_screen = UI_WATCH;
static bool consume_touch_until_up;
static bool wake_restore_pending;
static bool assets_refresh_pending;
static volatile bool theme_refresh_pending;
static bool settings_slider_dirty;
static bool settings_ble_dirty;
static bool settings_wifi_dirty;
static bool alarm_controls_dirty;
static bool alarm_swipe_active;
static bool alarm_swipe_hours;
static bool alarm_swipe_changed;
static bool map_drag_active;
static uint8_t weather_selected_day;
static int map_drag_x;
static int map_drag_y;
static int alarm_swipe_y;
static alarm_config_t alarm_edit = {.hour = 7, .minute = 0,
                                    .enabled = false};
static TickType_t last_touch_tick;
static int64_t button_down_us;
static int16_t content_left[BOARD_DISPLAY_HEIGHT];
static int16_t content_right[BOARD_DISPLAY_HEIGHT];
static int16_t contour_left[BOARD_DISPLAY_HEIGHT];
static int16_t contour_right[BOARD_DISPLAY_HEIGHT];
static portMUX_TYPE ui_input_lock = portMUX_INITIALIZER_UNLOCKED;
static ui_input_mailbox_t ui_input;
static watch_values_t displayed_watch_values;
static bool displayed_watch_values_valid;
static char displayed_launcher_battery[5];
static bool displayed_launcher_ble_enabled;
static watch_wifi_state_t displayed_launcher_wifi_state;
static uint8_t glyph_indices[256];

static bool touch_is_button(ui_screen_t screen, uint16_t x, uint16_t y);

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
    {205, 92, 42, ICON_ACTIVITY, LAUNCHER_ACTION_NONE},
    {110, 143, 42, ICON_HEART, LAUNCHER_ACTION_NONE},
    {300, 143, 42, ICON_SLEEP, LAUNCHER_ACTION_ALARM},
    {73, 241, 42, ICON_MAP, LAUNCHER_ACTION_MAP},
    {337, 241, 42, ICON_WEATHER, LAUNCHER_ACTION_WEATHER},
    {111, 340, 42, ICON_MUSIC, LAUNCHER_ACTION_NONE},
    {299, 340, 42, ICON_MESSAGES, LAUNCHER_ACTION_MESSAGES},
    {205, 385, 42, ICON_SETTINGS, LAUNCHER_ACTION_SETTINGS},
    {205, 235, 70, ICON_CLOCK, LAUNCHER_ACTION_WATCH},
};

static void *cjson_psram_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static void cjson_psram_free(void *pointer)
{
    heap_caps_free(pointer);
}

static void initialize_cjson_allocator(void)
{
    cJSON_Hooks hooks = {
        .malloc_fn = cjson_psram_malloc,
        .free_fn = cjson_psram_free,
    };
    cJSON_InitHooks(&hooks);
}

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

static esp_err_t display_apply_brightness(uint8_t percentage)
{
    if (percentage > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    if (display_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t panel_level =
        (uint8_t)(((unsigned)percentage * 255U + 50U) / 100U);
    return display_command(0x51, &panel_level, 1);
}

esp_err_t screen_set_brightness(uint8_t percentage)
{
    if (percentage > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!panel_hidden) {
        ESP_RETURN_ON_ERROR(display_apply_brightness(percentage), TAG,
                            "display brightness update failed");
    }
    display_brightness_percentage = percentage;
    return ESP_OK;
}

uint8_t screen_get_brightness(void)
{
    return display_brightness_percentage;
}

esp_err_t screen_set_theme_color(uint8_t red, uint8_t green, uint8_t blue)
{
    theme_rgb = ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
    theme_refresh_pending = true;
    screen_request_refresh();
    return ESP_OK;
}

void screen_get_theme_color(uint8_t *red, uint8_t *green, uint8_t *blue)
{
    const uint32_t color = theme_rgb;
    if (red != NULL) {
        *red = (uint8_t)(color >> 16);
    }
    if (green != NULL) {
        *green = (uint8_t)(color >> 8);
    }
    if (blue != NULL) {
        *blue = (uint8_t)color;
    }
}

void screen_request_refresh(void)
{
    if (ui_task_handle != NULL) {
        xTaskNotify(ui_task_handle, UI_EVENT_REFRESH, eSetBits);
    }
}

void screen_show_messages(void)
{
    portENTER_CRITICAL(&ui_input_lock);
    ui_input.show_messages = true;
    portEXIT_CRITICAL(&ui_input_lock);
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

void screen_alarm_ring_started(void)
{
    portENTER_CRITICAL(&ui_input_lock);
    ui_input.alarm_ring = true;
    portEXIT_CRITICAL(&ui_input_lock);
    if (ui_task_handle != NULL) {
        xTaskNotify(ui_task_handle, UI_EVENT_REFRESH, eSetBits);
    }
}

static bool point_in_circle(int x, int y, int center_x, int center_y, int radius)
{
    const int dx = x - center_x;
    const int dy = y - center_y;
    return dx * dx + dy * dy <= radius * radius;
}

static launcher_action_t launcher_action_at(int x, int y)
{
    for (size_t index = 0;
         index < sizeof(launcher_bubbles) / sizeof(launcher_bubbles[0]);
         index++) {
        const bubble_t *bubble = &launcher_bubbles[index];
        if (point_in_circle(x, y, bubble->x, bubble->y, bubble->radius)) {
            return bubble->action;
        }
    }
    return LAUNCHER_ACTION_NONE;
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
    bool valid_button = touch_is_button(active_screen, x, y);
    if (valid_button && active_screen == UI_WEATHER &&
        (point_in_circle(x, y, 70, 420, 38) ||
         point_in_circle(x, y, 340, 420, 38))) {
        weather_snapshot_t weather;
        weather_get_snapshot(&weather);
        valid_button = point_in_circle(x, y, 70, 420, 38)
                           ? weather_selected_day > 0
                           : weather_selected_day + 1 < weather.day_count;
    }
    if (event == SCREEN_TOUCH_DOWN && active_screen != UI_BLACK &&
        !consume_touch_until_up && valid_button) {
        button_down_us = esp_timer_get_time();
        esp_err_t result = ble_haptic_click();
        if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "UI haptic click failed: %s",
                     esp_err_to_name(result));
        }
    }
    if (ui_task_handle != NULL) {
        xTaskNotify(ui_task_handle, UI_EVENT_TOUCH, eSetBits);
    }
}

static bool touch_is_button(ui_screen_t screen, uint16_t x, uint16_t y)
{
    return (screen == UI_WATCH && point_in_circle(x, y, 205, 410, 34)) ||
           (screen == UI_LAUNCHER &&
            launcher_action_at(x, y) != LAUNCHER_ACTION_NONE) ||
           (screen == UI_SETTINGS &&
            (point_in_circle(x, y, 205, 435, 38) ||
             (x >= 42 && x <= 368 && y >= 232 && y <= 294) ||
             (x >= 42 && x <= 368 && y >= 305 && y <= 367))) ||
           (screen == UI_ALARM &&
            (ble_alarm_is_ringing()
                 ? (x >= 55 && x <= 355 && y >= 292 && y <= 378)
                 : (point_in_circle(x, y, 205, 430, 40) ||
                    (x >= 70 && x <= 340 && y >= 295 && y <= 365)))) ||
           (screen == UI_MAP &&
            (point_in_circle(x, y, 350, 74, 28) ||
             point_in_circle(x, y, 350, 142, 28) ||
             point_in_circle(x, y, 350, 210, 28) ||
             point_in_circle(x, y, 94, 68, 22) ||
             point_in_circle(x, y, 94, 172, 22) ||
             point_in_circle(x, y, 42, 120, 22) ||
             point_in_circle(x, y, 146, 120, 22) ||
             point_in_circle(x, y, 205, 445, 36))) ||
           (screen == UI_WEATHER &&
            (point_in_circle(x, y, 205, 435, 38) ||
             point_in_circle(x, y, 70, 420, 38) ||
             point_in_circle(x, y, 340, 420, 38))) ||
           (screen == UI_MESSAGES &&
            (point_in_circle(x, y, 205, 445, 36) ||
             (x >= 70 && x <= 340 && y >= 105 && y <= 170)));
}

static bool activate_launcher_action(launcher_action_t action)
{
    switch (action) {
    case LAUNCHER_ACTION_WATCH:
        active_screen = UI_WATCH;
        return true;
    case LAUNCHER_ACTION_SETTINGS:
        active_screen = UI_SETTINGS;
        return true;
    case LAUNCHER_ACTION_ALARM:
        ble_alarm_get_config(&alarm_edit);
        active_screen = UI_ALARM;
        return true;
    case LAUNCHER_ACTION_MAP: {
        esp_err_t result = offline_map_start();
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "map start failed: %s", esp_err_to_name(result));
        }
        active_screen = UI_MAP;
        return true;
    }
    case LAUNCHER_ACTION_WEATHER: {
        weather_selected_day = 0;
        esp_err_t result = watch_wifi_ensure_enabled();
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "automatic Wi-Fi start failed: %s",
                     esp_err_to_name(result));
        }
        result = weather_start();
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "weather start failed: %s", esp_err_to_name(result));
        }
        active_screen = UI_WEATHER;
        return true;
    }
    case LAUNCHER_ACTION_MESSAGES:
        active_screen = UI_MESSAGES;
        return true;
    case LAUNCHER_ACTION_NONE:
    default:
        return false;
    }
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

    if (active_screen == UI_SETTINGS &&
        (event == SCREEN_TOUCH_DOWN || event == SCREEN_TOUCH_MOVE) &&
        y >= 120 && y <= 220) {
        int clamped_x = x < 42 ? 42 : (x > 368 ? 368 : x);
        uint8_t brightness = (uint8_t)(((clamped_x - 42) * 100 + 163) / 326);
        if (screen_set_brightness(brightness) != ESP_OK) {
            ESP_LOGW(TAG, "brightness update failed");
        }
        settings_slider_dirty = true;
        return false;
    }
    if (active_screen == UI_ALARM && !ble_alarm_is_ringing()) {
        const bool in_hours = x >= 55 && x <= 195 && y >= 60 && y <= 260;
        const bool in_minutes = x >= 215 && x <= 355 && y >= 60 && y <= 260;
        if (event == SCREEN_TOUCH_DOWN && (in_hours || in_minutes)) {
            alarm_swipe_active = true;
            alarm_swipe_hours = in_hours;
            alarm_swipe_changed = false;
            alarm_swipe_y = y;
            return false;
        }
        if (event == SCREEN_TOUCH_MOVE && alarm_swipe_active) {
            int delta = (int)y - alarm_swipe_y;
            int steps = 0;
            while (delta <= -ALARM_ROLL_STEP_PIXELS) {
                steps++;
                delta += ALARM_ROLL_STEP_PIXELS;
                alarm_swipe_y -= ALARM_ROLL_STEP_PIXELS;
            }
            while (delta >= ALARM_ROLL_STEP_PIXELS) {
                steps--;
                delta -= ALARM_ROLL_STEP_PIXELS;
                alarm_swipe_y += ALARM_ROLL_STEP_PIXELS;
            }
            if (steps != 0) {
                if (alarm_swipe_hours) {
                    alarm_edit.hour =
                        (uint8_t)((alarm_edit.hour + 24 + steps) % 24);
                } else {
                    alarm_edit.minute =
                        (uint8_t)((alarm_edit.minute + 60 + steps) % 60);
                }
                alarm_swipe_changed = true;
                alarm_controls_dirty = true;
            }
            return false;
        }
    }
    if (active_screen == UI_MAP) {
        const bool control = point_in_circle(x, y, 350, 74, 34) ||
            point_in_circle(x, y, 350, 142, 34) ||
            point_in_circle(x, y, 350, 210, 34) ||
            point_in_circle(x, y, 94, 68, 28) ||
            point_in_circle(x, y, 94, 172, 28) ||
            point_in_circle(x, y, 42, 120, 28) ||
            point_in_circle(x, y, 146, 120, 28) ||
            point_in_circle(x, y, 205, 445, 42);
        if (event == SCREEN_TOUCH_DOWN && !control) {
            map_drag_active = true;
            map_drag_x = x;
            map_drag_y = y;
            return false;
        }
        if (event == SCREEN_TOUCH_MOVE && map_drag_active) {
            return false;
        }
        if (event == SCREEN_TOUCH_UP && map_drag_active) {
            map_drag_active = false;
            int delta_x = (int)x - map_drag_x;
            int delta_y = (int)y - map_drag_y;
            if (delta_x != 0 || delta_y != 0) {
                offline_map_pan(delta_x, delta_y);
            }
            return false;
        }
    }
    if (event != SCREEN_TOUCH_UP) {
        return false;
    }

    if (active_screen == UI_ALARM && ble_alarm_is_ringing()) {
        if (x >= 55 && x <= 355 && y >= 292 && y <= 378) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ble_alarm_dismiss());
            alarm_controls_dirty = true;
        }
        return false;
    }
    if (active_screen == UI_ALARM && alarm_swipe_active) {
        alarm_swipe_active = false;
        if (alarm_swipe_changed) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ble_alarm_set(
                alarm_edit.hour, alarm_edit.minute, alarm_edit.enabled));
        }
        alarm_swipe_changed = false;
        return false;
    }

    bool changed = false;
    if (active_screen == UI_WATCH && point_in_circle(x, y, 205, 410, 34)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_LAUNCHER) {
        changed = activate_launcher_action(launcher_action_at(x, y));
    } else if (active_screen == UI_SETTINGS &&
               point_in_circle(x, y, 205, 435, 42)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_SETTINGS &&
               x >= 42 && x <= 368 && y >= 232 && y <= 294) {
        esp_err_t result = ble_rtc_set_advertising_enabled(
            !ble_rtc_advertising_enabled());
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "BLE advertising update failed: %s",
                     esp_err_to_name(result));
        } else {
            settings_ble_dirty = true;
        }
    } else if (active_screen == UI_SETTINGS &&
               x >= 42 && x <= 368 && y >= 305 && y <= 367) {
        watch_wifi_status_t wifi;
        watch_wifi_get_status(&wifi);
        esp_err_t result = watch_wifi_set_enabled(!wifi.requested);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Wi-Fi update failed: %s", esp_err_to_name(result));
        } else {
            settings_wifi_dirty = true;
        }
    } else if (active_screen == UI_ALARM &&
               point_in_circle(x, y, 205, 430, 40)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_ALARM &&
               x >= 70 && x <= 340 && y >= 295 && y <= 365) {
        alarm_edit.enabled = !alarm_edit.enabled;
        esp_err_t result = ble_alarm_set(alarm_edit.hour, alarm_edit.minute,
                                         alarm_edit.enabled);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "alarm toggle failed: %s", esp_err_to_name(result));
            alarm_edit.enabled = !alarm_edit.enabled;
        } else {
            alarm_controls_dirty = true;
        }
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 350, 74, 34)) {
        offline_map_zoom(1);
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 350, 142, 34)) {
        offline_map_zoom(-1);
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 350, 210, 34)) {
        offline_map_recenter();
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 94, 68, 28)) {
        offline_map_pan_step(0, -1);
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 94, 172, 28)) {
        offline_map_pan_step(0, 1);
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 42, 120, 28)) {
        offline_map_pan_step(-1, 0);
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 146, 120, 28)) {
        offline_map_pan_step(1, 0);
    } else if (active_screen == UI_MAP &&
               point_in_circle(x, y, 205, 445, 42)) {
        offline_map_stop();
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_WEATHER &&
               point_in_circle(x, y, 205, 435, 44)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_WEATHER &&
               point_in_circle(x, y, 70, 420, 44)) {
        if (weather_selected_day > 0) {
            weather_selected_day--;
            changed = true;
        }
    } else if (active_screen == UI_WEATHER &&
               point_in_circle(x, y, 340, 420, 44)) {
        weather_snapshot_t weather;
        weather_get_snapshot(&weather);
        if (weather_selected_day + 1 < weather.day_count) {
            weather_selected_day++;
            changed = true;
        }
    } else if (active_screen == UI_MESSAGES &&
               point_in_circle(x, y, 205, 445, 36)) {
        active_screen = UI_LAUNCHER;
        changed = true;
    } else if (active_screen == UI_MESSAGES &&
               x >= 70 && x <= 340 && y >= 105 && y <= 170) {
        mesh_config_t config;
        mesh_service_get_config(&config);
        config.enabled = !config.enabled;
        esp_err_t result = mesh_service_set_config(&config);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Meshtastic toggle failed: %s",
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

    if (input.alarm_ring) {
        if (active_screen == UI_MAP) {
            offline_map_stop();
        }
        ble_alarm_get_config(&alarm_edit);
        active_screen = UI_ALARM;
        consume_touch_until_up = false;
        wake_restore_pending = false;
        last_touch_tick = xTaskGetTickCount();
        changed = true;
    }

    if (input.show_messages) {
        if (active_screen == UI_MAP) offline_map_stop();
        active_screen = UI_MESSAGES;
        consume_touch_until_up = false;
        wake_restore_pending = false;
        last_touch_tick = xTaskGetTickCount();
        changed = true;
    }

    if (input.wake) {
        last_touch_tick = xTaskGetTickCount();
        if (active_screen == UI_BLACK) {
            active_screen = UI_WATCH;
            consume_touch_until_up = true;
            wake_restore_pending = true;
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
    ESP_ERROR_CHECK(gpio_set_direction(BOARD_DISPLAY_TE, GPIO_MODE_INPUT));
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
    for (size_t i = 0; i < sizeof(display_init_commands) /
                            sizeof(display_init_commands[0]); i++) {
        const display_init_command_t *entry = &display_init_commands[i];
        ESP_ERROR_CHECK(display_command(entry->command, entry->parameters,
                                        entry->length & 0x1F));
        if (entry->length & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
    ESP_ERROR_CHECK(display_command(0x36, &(uint8_t){0}, 1));
    ESP_ERROR_CHECK(display_apply_brightness(0));
    panel_hidden = true;
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

static uint16_t theme_wire_color(void)
{
    const uint32_t color = theme_rgb;
    return wire_rgb565((uint8_t)(color >> 16), (uint8_t)(color >> 8),
                       (uint8_t)color);
}

static uint16_t theme_wire_scaled(uint8_t scale)
{
    const uint32_t color = theme_rgb;
    return wire_rgb565((uint8_t)(((color >> 16) & 0xFF) * scale / 255),
                       (uint8_t)(((color >> 8) & 0xFF) * scale / 255),
                       (uint8_t)((color & 0xFF) * scale / 255));
}

static uint16_t theme_wire_tinted(uint8_t white)
{
    const uint32_t color = theme_rgb;
    return wire_rgb565(
        (uint8_t)((((color >> 16) & 0xFF) * (255 - white) + 255 * white) /
                  255),
        (uint8_t)((((color >> 8) & 0xFF) * (255 - white) + 255 * white) /
                  255),
        (uint8_t)(((color & 0xFF) * (255 - white) + 255 * white) / 255));
}

static void wait_for_te_edge(void)
{
    const int initial_level = gpio_get_level(BOARD_DISPLAY_TE);
    const int64_t started_us = esp_timer_get_time();
    while (gpio_get_level(BOARD_DISPLAY_TE) == initial_level &&
           esp_timer_get_time() - started_us < DISPLAY_TE_WAIT_US) {
        taskYIELD();
    }
}

static void set_address_window(int x, int y, int width, int rows)
{
    const int start_x = x + BOARD_DISPLAY_COLUMN_OFFSET;
    const int end_x = start_x + width - 1;
    const uint8_t columns[] = {
        (uint8_t)(start_x >> 8), (uint8_t)start_x,
        (uint8_t)(end_x >> 8), (uint8_t)end_x,
    };
    const int end_y = y + rows - 1;
    const uint8_t row_address[] = {
        (uint8_t)(y >> 8), (uint8_t)y,
        (uint8_t)(end_y >> 8), (uint8_t)end_y,
    };
    ESP_ERROR_CHECK(display_command(0x2A, columns, sizeof(columns)));
    ESP_ERROR_CHECK(display_command(0x2B, row_address, sizeof(row_address)));
}

static void display_wait_for_transfer(void)
{
    if (!display_transfer.pending) {
        return;
    }
    spi_transaction_t *completed;
    ESP_ERROR_CHECK(spi_device_get_trans_result(display_spi, &completed,
                                                portMAX_DELAY));
    ESP_ERROR_CHECK(spi_device_get_trans_result(display_spi, &completed,
                                                portMAX_DELAY));
    spi_device_release_bus(display_spi);
    display_transfer.pending = false;
}

static bool display_queue_color_band(const uint16_t *pixels, size_t count)
{
    static const uint8_t wrapper[] = {0x32, 0x00, 0x2C, 0x00};
    memset(&display_transfer.command, 0, sizeof(display_transfer.command));
    memset(&display_transfer.color, 0, sizeof(display_transfer.color));
    display_transfer.command.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    display_transfer.command.length = sizeof(wrapper) * 8;
    display_transfer.command.tx_buffer = wrapper;
    display_transfer.color.flags = SPI_TRANS_MODE_QIO;
    display_transfer.color.length = count * 16;
    display_transfer.color.tx_buffer = pixels;
    esp_err_t result = spi_device_acquire_bus(display_spi, portMAX_DELAY);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "display bus acquire failed: %s",
                 esp_err_to_name(result));
        return false;
    }
    result = spi_device_queue_trans(display_spi, &display_transfer.command,
                                    portMAX_DELAY);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "display command queue failed: %s",
                 esp_err_to_name(result));
        spi_device_release_bus(display_spi);
        return false;
    }
    result = spi_device_queue_trans(display_spi, &display_transfer.color,
                                    portMAX_DELAY);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "display color queue failed: %s",
                 esp_err_to_name(result));
        spi_transaction_t *completed;
        esp_err_t wait_result = spi_device_get_trans_result(
            display_spi, &completed, portMAX_DELAY);
        if (wait_result != ESP_OK) {
            ESP_LOGE(TAG, "display command completion failed: %s",
                     esp_err_to_name(wait_result));
        }
        spi_device_release_bus(display_spi);
        return false;
    }
    display_transfer.pending = true;
    return true;
}

static int glyph_index(char character)
{
    unsigned value = (unsigned char)character;
    if (value < sizeof(glyph_indices)) {
        return glyph_indices[value];
    }
    return 0;
}

static void initialize_glyph_indices(void)
{
    memset(glyph_indices, 0, sizeof(glyph_indices));
    for (int index = 0; index < CASCADIA_CODE_GLYPH_COUNT; index++) {
        unsigned value = (unsigned char)cascadia_code_characters[index];
        if (value < sizeof(glyph_indices)) {
            glyph_indices[value] = (uint8_t)index;
        }
    }
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
    if (icon_atlas != NULL && (size_t)icon < icon_atlas_tile_count) {
        size_t offset = ((size_t)icon * ICON_SIZE * ICON_SIZE +
                         iy * ICON_SIZE + ix) * 2;
        if (icon_atlas[offset] == 0 && icon_atlas[offset + 1] == 0) {
            return 0;
        }
        if (icon == ICON_WIFI_OFF) return wire_rgb565(112, 126, 146);
        if (icon == ICON_WIFI_CONNECTING || icon == ICON_WIFI_CONNECTED) {
            return theme_wire_tinted(icon == ICON_WIFI_CONNECTED ? 100 : 62);
        }
        if (icon == ICON_WIFI_ERROR) return wire_rgb565(238, 92, 99);
        return (uint16_t)icon_atlas[offset] |
               ((uint16_t)icon_atlas[offset + 1] << 8);
    }

    int dx = ix - ICON_SIZE / 2;
    int dy = iy - ICON_SIZE / 2;
    if (icon == ICON_LAUNCHER) {
        if (abs(dx) <= 15 && abs(dy) <= 15 &&
            ((abs(dx + 10) <= 3 || abs(dx) <= 3 || abs(dx - 10) <= 3) &&
             (abs(dy + 10) <= 3 || abs(dy) <= 3 || abs(dy - 10) <= 3))) {
            return theme_wire_color();
        }
    } else if (icon == ICON_CLOCK) {
        int d2 = dx * dx + dy * dy;
        if ((d2 >= 225 && d2 <= 324) ||
            (abs(dx) <= 2 && dy <= 2 && dy >= -12) ||
            (abs(dy - dx / 2) <= 2 && dx >= 0 && dx <= 11)) {
            return theme_wire_tinted(64);
        }
    } else if (icon == ICON_SETTINGS) {
        int d2 = dx * dx + dy * dy;
        if ((d2 >= 90 && d2 <= 200) ||
            ((abs(dx) <= 3 || abs(dy) <= 3) && d2 <= 300 && d2 >= 180)) {
            return theme_wire_tinted(64);
        }
    } else if (icon == ICON_BLE_OFF || icon == ICON_BLE_ON) {
        const bool stem = abs(dx) <= 2 && abs(dy) <= 17;
        const bool upper = abs(dy - dx + 2) <= 2 && dy <= 1 && dx >= -1;
        const bool lower = abs(dy + dx - 2) <= 2 && dy >= -1 && dx >= -1;
        const bool slash = icon == ICON_BLE_OFF && abs(dy - dx) <= 2;
        if (stem || upper || lower || slash) {
            return icon == ICON_BLE_ON ? theme_wire_tinted(48)
                                       : wire_rgb565(112, 126, 146);
        }
    } else if (icon >= ICON_BATTERY_EMPTY && icon <= ICON_BATTERY_FULL) {
        const bool outline = abs(dx) >= 14 || abs(dy) >= 10;
        const bool terminal = dx >= 18 && abs(dy) <= 5;
        int level = (int)icon - (int)ICON_BATTERY_EMPTY;
        int fill_right = -13 + level * 9;
        const bool fill = dx >= -13 && dx <= fill_right && abs(dy) <= 8;
        if ((outline && abs(dx) <= 17 && abs(dy) <= 12) || terminal || fill) {
            return wire_rgb565(level == 0 ? 238 : 79,
                               level == 0 ? 92 : 226,
                               level == 0 ? 99 : 157);
        }
    } else if (icon == ICON_MAP) {
        const bool left_edge = abs(dx + 15) <= 2 && abs(dy) <= 15;
        const bool right_edge = abs(dx - 15) <= 2 && abs(dy) <= 15;
        const bool fold_left = abs(dx + 5) <= 2 && abs(dy) <= 15;
        const bool fold_right = abs(dx - 5) <= 2 && abs(dy) <= 15;
        const bool top_bottom = abs(abs(dy) - 15) <= 2 && abs(dx) <= 15;
        const bool pin = (dx - 5) * (dx - 5) + (dy + 3) * (dy + 3) <= 30 ||
                         (dy >= 1 && dy <= 10 && abs(dx - 5) <= 5 - dy / 2);
        if (left_edge || right_edge || fold_left || fold_right ||
            top_bottom || pin) {
            return theme_wire_tinted(56);
        }
    } else if (icon == ICON_ZOOM_IN || icon == ICON_ZOOM_OUT) {
        if (abs(dy) <= 3 && abs(dx) <= 16) {
            return theme_wire_tinted(72);
        }
        if (icon == ICON_ZOOM_IN && abs(dx) <= 3 && abs(dy) <= 16) {
            return theme_wire_tinted(72);
        }
    } else if (icon == ICON_MAP_CENTER) {
        int d2 = dx * dx + dy * dy;
        if ((d2 >= 196 && d2 <= 324) ||
            (abs(dx) <= 2 && abs(dy) <= 18) ||
            (abs(dy) <= 2 && abs(dx) <= 18) || d2 <= 25) {
            return theme_wire_tinted(72);
        }
    } else if (icon >= ICON_WIFI_OFF && icon <= ICON_WIFI_ERROR) {
        const int adx = abs(dx);
        const int ady = abs(dy);
        const bool outer = ady <= 2 && adx <= 2;
        const int radius2 = dx * dx + dy * dy;
        const bool lower_arc = radius2 >= 36 && radius2 <= 70 && dy >= 0;
        const bool middle_arc = radius2 >= 120 && radius2 <= 180 && dy >= -2;
        const bool upper_arc = radius2 >= 260 && radius2 <= 350 && dy >= -4;
        const bool slash = icon == ICON_WIFI_OFF && abs(dy - dx) <= 2;
        const bool alert = icon == ICON_WIFI_ERROR &&
                           ((abs(dx) <= 2 && dy >= -8 && dy <= 5) ||
                            (radius2 <= 7 && dy >= 9));
        if (outer || lower_arc || middle_arc || upper_arc || slash || alert) {
            if (icon == ICON_WIFI_OFF) return wire_rgb565(112, 126, 146);
            if (icon == ICON_WIFI_ERROR) return wire_rgb565(238, 92, 99);
            return theme_wire_tinted(icon == ICON_WIFI_CONNECTED ? 100 : 62);
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
        return theme_wire_scaled(190);
    }
    if (d2 <= r2) {
        uint8_t scale = (uint8_t)(12 + (r2 - d2) * 18 / r2);
        return theme_wire_scaled(scale);
    }
    return theme_wire_scaled(75);
}

static void read_watch_values(watch_values_t *values)
{
    static const char *weekdays[] = {"SO", "MO", "DI", "MI", "DO", "FR", "SA"};
    static const char *months[] = {
        "JAN", "FEB", "MRZ", "APR", "MAI", "JUN",
        "JUL", "AUG", "SEP", "OKT", "NOV", "DEZ",
    };
    rtc_datetime_t datetime;
    strcpy(values->time, "--:--");
    strcpy(values->date, "-- -- ---");
    strcpy(values->battery, "--%");
    values->ble_enabled = ble_rtc_advertising_enabled();
    watch_wifi_status_t wifi;
    watch_wifi_get_status(&wifi);
    values->wifi_state = wifi.state;
    alarm_config_t alarm;
    ble_alarm_get_config(&alarm);
    values->alarm_enabled = alarm.enabled;
    if (ble_rtc_get_datetime(&datetime) == ESP_OK) {
        snprintf(values->time, sizeof(values->time), "%02d:%02d",
                 datetime.hour, datetime.minute);
        snprintf(values->date, sizeof(values->date), "%s %02d %s",
                 weekdays[datetime.weekday], datetime.day,
                 months[datetime.month - 1]);
    }
    char payload[24];
    unsigned percentage;
    if (ble_power_get_payload(payload, sizeof(payload)) == ESP_OK &&
        sscanf(payload, "%u,", &percentage) == 1 && percentage <= 100) {
        snprintf(values->battery, sizeof(values->battery), "%u%%", percentage);
    }
}

static void frame_set_content(uint16_t *frame, int x, int y, uint16_t color)
{
    if (x >= 0 && x < BOARD_DISPLAY_WIDTH && pixel_is_safe_content(x, y)) {
        frame[y * BOARD_DISPLAY_WIDTH + x] = color;
    }
}

static void clear_content_rect(uint16_t *frame, int x, int y,
                               int width, int height)
{
    for (int py = y; py < y + height && py < BOARD_DISPLAY_HEIGHT; py++) {
        for (int px = x; px < x + width && px < BOARD_DISPLAY_WIDTH; px++) {
            frame_set_content(frame, px, py, 0);
        }
    }
}

static void draw_text(uint16_t *frame, const char *text, int start_x,
                      int start_y, int divisor, uint16_t color)
{
    const int cell_width = (CASCADIA_CODE_CELL_WIDTH + divisor - 1) / divisor;
    const int height = (CASCADIA_CODE_GLYPH_HEIGHT + divisor - 1) / divisor;
    for (size_t character = 0; text[character] != '\0'; character++) {
        const uint8_t (*glyph)[(CASCADIA_CODE_CELL_WIDTH + 7) / 8] =
            cascadia_code_glyphs[glyph_index(text[character])];
        for (int py = 0; py < height; py++) {
            const int source_y = py * divisor;
            if (source_y >= CASCADIA_CODE_GLYPH_HEIGHT) {
                continue;
            }
            for (int px = 0; px < cell_width; px++) {
                const int source_x = px * divisor;
                if (source_x >= CASCADIA_CODE_CELL_WIDTH) {
                    continue;
                }
                if ((glyph[source_y][source_x / 8] &
                     (1U << (7 - source_x % 8))) != 0) {
                    frame_set_content(frame,
                                      start_x + (int)character * cell_width + px,
                                      start_y + py, color);
                }
            }
        }
    }
}

static int time_glyph_index(char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    return character == ':' ? 10 : 0;
}

static int time_text_width(const char *text)
{
    return (int)strlen(text) * CASCADIA_TIME_CELL_WIDTH;
}

static void draw_time_text(uint16_t *frame, const char *text, int start_x,
                           int start_y, uint16_t color)
{
    for (size_t character = 0; text[character] != '\0'; character++) {
        const uint8_t (*glyph)[CASCADIA_TIME_BYTES_PER_ROW] =
            cascadia_time_glyphs[time_glyph_index(text[character])];
        for (int py = 0; py < CASCADIA_TIME_GLYPH_HEIGHT; py++) {
            for (int px = 0; px < CASCADIA_TIME_CELL_WIDTH; px++) {
                if ((glyph[py][px / 8] & (1U << (7 - px % 8))) != 0) {
                    frame_set_content(frame,
                                      start_x +
                                          (int)character *
                                              CASCADIA_TIME_CELL_WIDTH +
                                          px,
                                      start_y + py, color);
                }
            }
        }
    }
}

static icon_id_t battery_icon_for(const char *battery)
{
    unsigned percentage;
    if (sscanf(battery, "%u%%", &percentage) != 1) {
        return ICON_BATTERY_EMPTY;
    }
    if (percentage >= 75) {
        return ICON_BATTERY_FULL;
    }
    if (percentage >= 40) {
        return ICON_BATTERY_MEDIUM;
    }
    if (percentage >= 15) {
        return ICON_BATTERY_LOW;
    }
    return ICON_BATTERY_EMPTY;
}

static icon_id_t wifi_icon_for(watch_wifi_state_t state)
{
    if (state == WATCH_WIFI_OFF) return ICON_WIFI_OFF;
    if (state == WATCH_WIFI_LOADING || state == WATCH_WIFI_CONNECTING) {
        return ICON_WIFI_CONNECTING;
    }
    if (state == WATCH_WIFI_CONNECTED) return ICON_WIFI_CONNECTED;
    return ICON_WIFI_ERROR;
}

static void draw_contour(uint16_t *frame)
{
    const uint16_t color = theme_wire_color();
    for (int y = 0; y < BOARD_DISPLAY_HEIGHT; y++) {
        for (int x = contour_left[y]; x <= contour_right[y]; x++) {
            if (!pixel_is_safe_content(x, y)) {
                frame[y * BOARD_DISPLAY_WIDTH + x] = color;
            }
        }
    }
}

static void draw_bubble(uint16_t *frame, const bubble_t *bubble)
{
    const int extent = bubble->radius + 4;
    for (int y = bubble->y - extent; y <= bubble->y + extent; y++) {
        for (int x = bubble->x - extent; x <= bubble->x + extent; x++) {
            uint16_t color = glass_bubble_pixel(bubble, x, y);
            if (color != 0) {
                frame_set_content(frame, x, y, color);
            }
        }
    }
}

static void draw_icon(uint16_t *frame, icon_id_t icon, int center_x,
                      int center_y, int size)
{
    for (int y = center_y - size / 2; y < center_y + (size + 1) / 2; y++) {
        for (int x = center_x - size / 2; x < center_x + (size + 1) / 2; x++) {
            uint16_t color = size == ICON_SIZE
                                 ? icon_pixel(icon, center_x, center_y, x, y)
                                 : icon_pixel_sized(icon, center_x, center_y,
                                                    size, x, y);
            if (color != 0) {
                frame_set_content(frame, x, y, color);
            }
        }
    }
}

static void draw_watch_time(uint16_t *frame, const watch_values_t *values)
{
    clear_content_rect(
        frame, WATCH_TIME_REGION_X, WATCH_TIME_REGION_Y,
        WATCH_TIME_REGION_WIDTH, WATCH_TIME_REGION_HEIGHT);
    const int width = time_text_width(values->time);
    draw_time_text(frame, values->time, (BOARD_DISPLAY_WIDTH - width) / 2,
                   WATCH_TIME_DRAW_Y, wire_rgb565(244, 248, 255));
}

static void draw_watch_date(uint16_t *frame, const watch_values_t *values)
{
    clear_content_rect(frame, 70, 45, 270, 42);
    draw_text(frame, values->date, centered_text_x(values->date, 2), 53, 2,
              theme_wire_tinted(50));
}

static void draw_watch_alarm(uint16_t *frame, const watch_values_t *values)
{
    clear_content_rect(frame, 185, 230, 40, 40);
    if (!values->alarm_enabled) {
        return;
    }
    const int center_x = 205;
    const int center_y = 250;
    const uint16_t color = theme_wire_tinted(90);
    for (int y = center_y - 14; y <= center_y + 14; y++) {
        for (int x = center_x - 14; x <= center_x + 14; x++) {
            const int dx = x - center_x;
            const int dy = y - center_y;
            const int distance = dx * dx + dy * dy;
            const bool face = distance >= 72 && distance <= 110;
            const bool hour_hand = abs(dx) <= 1 && dy >= -6 && dy <= 1;
            const bool minute_hand = abs(dy) <= 1 && dx >= 0 && dx <= 6;
            const bool bells = dy >= -13 && dy <= -10 &&
                ((dx >= -10 && dx <= -4) || (dx >= 4 && dx <= 10));
            const bool feet = dy >= 9 && dy <= 13 &&
                ((dx >= -9 && dx <= -6) || (dx >= 6 && dx <= 9));
            if (face || hour_hand || minute_hand || bells || feet) {
                frame_set_content(frame, x, y, color);
            }
        }
    }
}

static void draw_watch_launcher(uint16_t *frame)
{
    clear_content_rect(frame, 165, 370, 80, 80);
    const bubble_t launcher = {
        205, 410, 34, ICON_LAUNCHER, LAUNCHER_ACTION_NONE};
    draw_bubble(frame, &launcher);
    draw_icon(frame, ICON_LAUNCHER, 205, 410, ICON_SIZE);
}

static void draw_watch_status(uint16_t *frame, const watch_values_t *values)
{
    clear_content_rect(frame, 40, 390, 120, 40);
    clear_content_rect(frame, 270, 390, 100, 40);
    draw_icon(frame, values->ble_enabled ? ICON_BLE_ON : ICON_BLE_OFF,
              70, 410, 36);
    draw_icon(frame, wifi_icon_for(values->wifi_state), 115, 410, 36);
    draw_icon(frame, battery_icon_for(values->battery), 315, 410, 36);
    draw_text(frame, values->battery, 338, 403, 5,
              theme_wire_tinted(150));
}

static void compose_watch_frame(uint16_t *frame,
                                const watch_values_t *values)
{
    memset(frame, 0, BOARD_DISPLAY_WIDTH * BOARD_DISPLAY_HEIGHT *
                     sizeof(*frame));
    draw_watch_time(frame, values);
    draw_watch_date(frame, values);
    draw_watch_alarm(frame, values);
    draw_watch_launcher(frame);
    draw_watch_status(frame, values);
    draw_contour(frame);
}

static void draw_launcher_background_region(uint16_t *frame, int start_x,
                                            int start_y, int width, int height)
{
    for (int y = start_y; y < start_y + height && y < BOARD_DISPLAY_HEIGHT; y++) {
        for (int x = start_x; x < start_x + width && x < BOARD_DISPLAY_WIDTH; x++) {
            if (pixel_is_safe_content(x, y)) {
                frame[y * BOARD_DISPLAY_WIDTH + x] =
                    ((x * x + y * 7 + (x - y) * (x - y) / 8) % 43) < 2
                        ? theme_wire_scaled(24) : 0;
            }
        }
    }
}

static void draw_launcher_ble(uint16_t *frame, bool enabled)
{
    draw_launcher_background_region(frame, 42, 430, 56, 40);
    draw_icon(frame, enabled ? ICON_BLE_ON : ICON_BLE_OFF, 70, 450, 36);
}

static void draw_launcher_wifi(uint16_t *frame, watch_wifi_state_t state)
{
    draw_launcher_background_region(frame, 92, 430, 46, 40);
    draw_icon(frame, wifi_icon_for(state), 115, 450, 36);
}

static void draw_launcher_battery(uint16_t *frame, const char *battery)
{
    draw_launcher_background_region(frame, 295, 430, 95, 40);
    draw_icon(frame, battery_icon_for(battery), 315, 450, 36);
    draw_text(frame, battery, 338, 443, 5, wire_rgb565(105, 238, 166));
}

static void compose_launcher_frame(uint16_t *frame,
                                   const watch_values_t *values)
{
    memset(frame, 0, BOARD_DISPLAY_WIDTH * BOARD_DISPLAY_HEIGHT *
                     sizeof(*frame));
    draw_launcher_background_region(frame, 0, 0, BOARD_DISPLAY_WIDTH,
                                    BOARD_DISPLAY_HEIGHT);
    for (size_t index = 0; index < sizeof(launcher_bubbles) /
                                      sizeof(launcher_bubbles[0]); index++) {
        draw_bubble(frame, &launcher_bubbles[index]);
        draw_icon(frame, launcher_bubbles[index].icon,
                  launcher_bubbles[index].x, launcher_bubbles[index].y,
                  launcher_bubbles[index].icon == ICON_CLOCK ? 72 : ICON_SIZE);
    }
    draw_launcher_ble(frame, values->ble_enabled);
    draw_launcher_wifi(frame, values->wifi_state);
    draw_launcher_battery(frame, values->battery);
    draw_contour(frame);
}

static void draw_settings_slider(uint16_t *frame)
{
    clear_content_rect(frame, 35, 120, 340, 104);
    const int position = 42 + screen_get_brightness() * 326 / 100;
    for (int y = 164; y <= 180; y++) {
        for (int x = 42; x <= 368; x++) {
            frame_set_content(frame, x, y,
                              x <= position ? theme_wire_color()
                                            : wire_rgb565(34, 42, 58));
        }
    }
    for (int y = 154; y <= 190; y++) {
        for (int x = position - 18; x <= position + 18; x++) {
            const int dx = x - position;
            const int dy = y - 172;
            if (dx * dx + dy * dy <= 324) {
                frame_set_content(frame, x, y, theme_wire_tinted(120));
            }
        }
    }
}

static void draw_settings_ble(uint16_t *frame)
{
    clear_content_rect(frame, 40, 230, 330, 67);
    const bool on = ble_rtc_advertising_enabled();
    for (int y = 232; y <= 294; y++) {
        for (int x = 42; x <= 368; x++) {
            const bool edge = x < 46 || x > 364 || y < 236 || y > 290;
            frame_set_content(frame, x, y,
                              edge ? (on ? theme_wire_color()
                                         : wire_rgb565(70, 78, 92))
                                   : (on ? theme_wire_scaled(48)
                                         : wire_rgb565(3, 18, 22)));
        }
    }
    draw_icon(frame, on ? ICON_BLE_ON : ICON_BLE_OFF, 78, 263, 40);
    const char *label = on ? "BLUETOOTH AN" : "BLUETOOTH AUS";
    draw_text(frame, label, 116, 248, 4,
              wire_rgb565(240, 246, 255));
}

static void draw_settings_wifi(uint16_t *frame)
{
    clear_content_rect(frame, 40, 303, 330, 67);
    watch_wifi_status_t wifi;
    watch_wifi_get_status(&wifi);
    const bool on = wifi.requested;
    for (int y = 305; y <= 367; y++) {
        for (int x = 42; x <= 368; x++) {
            const bool edge = x < 46 || x > 364 || y < 309 || y > 363;
            frame_set_content(frame, x, y,
                              edge ? (on ? theme_wire_color()
                                         : wire_rgb565(70, 78, 92))
                                   : (on ? theme_wire_scaled(48)
                                         : wire_rgb565(3, 18, 22)));
        }
    }
    draw_icon(frame, wifi_icon_for(wifi.state), 78, 336, 40);
    const char *label = on ? "WLAN AN" : "WLAN AUS";
    draw_text(frame, label, 116, 321, 4,
              wire_rgb565(240, 246, 255));
}

static void compose_settings_frame(uint16_t *frame)
{
    memset(frame, 0, BOARD_DISPLAY_WIDTH * BOARD_DISPLAY_HEIGHT *
                     sizeof(*frame));
    draw_text(frame, "EINSTELLUNGEN", centered_text_x("EINSTELLUNGEN", 3),
              48, 3, theme_wire_tinted(72));
    draw_text(frame, "HELLIGKEIT", 42, 104, 4,
              theme_wire_tinted(72));
    draw_settings_slider(frame);
    draw_settings_ble(frame);
    draw_settings_wifi(frame);
    const bubble_t launcher = {
        205, 435, 38, ICON_LAUNCHER, LAUNCHER_ACTION_NONE};
    draw_bubble(frame, &launcher);
    draw_icon(frame, ICON_LAUNCHER, 205, 435, 50);
    draw_contour(frame);
}

static void draw_alarm_toggle(uint16_t *frame)
{
    const uint16_t edge = alarm_edit.enabled
                              ? theme_wire_color()
                              : wire_rgb565(70, 78, 92);
    const uint16_t fill = alarm_edit.enabled
                              ? theme_wire_scaled(58)
                              : wire_rgb565(8, 12, 20);
    for (int y = 295; y <= 365; y++) {
        for (int x = 70; x <= 340; x++) {
            const bool border = x < 74 || x > 336 || y < 299 || y > 361;
            frame_set_content(frame, x, y, border ? edge : fill);
        }
    }
    draw_text(frame, "AKTIV", 96, 318, 3,
              wire_rgb565(240, 246, 255));
    const int switch_center = alarm_edit.enabled ? 302 : 270;
    for (int y = 313; y <= 347; y++) {
        for (int x = 250; x <= 322; x++) {
            const bool track = (x >= 267 && x <= 305) ||
                ((x - 267) * (x - 267) + (y - 330) * (y - 330) <= 289) ||
                ((x - 305) * (x - 305) + (y - 330) * (y - 330) <= 289);
            if (track) {
                frame_set_content(frame, x, y,
                                  alarm_edit.enabled
                                      ? theme_wire_color()
                                      : wire_rgb565(45, 52, 66));
            }
            if ((x - switch_center) * (x - switch_center) +
                    (y - 330) * (y - 330) <= 169) {
                frame_set_content(frame, x, y,
                                  wire_rgb565(238, 246, 255));
            }
        }
    }
}

static void draw_alarm_rolling_time(uint16_t *frame)
{
    char current_hour[4];
    char current_minute[4];
    char previous_hour[4];
    char previous_minute[4];
    char next_hour[4];
    char next_minute[4];
    snprintf(current_hour, sizeof(current_hour), "%02u", alarm_edit.hour);
    snprintf(current_minute, sizeof(current_minute), "%02u", alarm_edit.minute);
    snprintf(previous_hour, sizeof(previous_hour), "%02u",
             (alarm_edit.hour + 23) % 24);
    snprintf(previous_minute, sizeof(previous_minute), "%02u",
             (alarm_edit.minute + 59) % 60);
    snprintf(next_hour, sizeof(next_hour), "%02u",
             (alarm_edit.hour + 1) % 24);
    snprintf(next_minute, sizeof(next_minute), "%02u",
             (alarm_edit.minute + 1) % 60);

    for (int y = 112; y <= 188; y++) {
        for (int x = 55; x <= 355; x++) {
            if (y < 115 || y > 185) {
                frame_set_content(frame, x, y, theme_wire_scaled(170));
            } else if (x == 205) {
                frame_set_content(frame, x, y, theme_wire_scaled(94));
            }
        }
    }
    const uint16_t adjacent = theme_wire_scaled(158);
    draw_text(frame, previous_hour, 109, 65, 2, adjacent);
    draw_text(frame, previous_minute, 259, 65, 2, adjacent);
    draw_text(frame, current_hour, 88, 125, 1,
              wire_rgb565(246, 249, 255));
    draw_text(frame, ":", 184, 125, 1, theme_wire_tinted(60));
    draw_text(frame, current_minute, 238, 125, 1,
              wire_rgb565(246, 249, 255));
    draw_text(frame, next_hour, 109, 218, 2, adjacent);
    draw_text(frame, next_minute, 259, 218, 2, adjacent);
}

static void compose_alarm_frame(uint16_t *frame)
{
    memset(frame, 0, DISPLAY_FRAME_BYTES);
    if (ble_alarm_is_ringing()) {
        char time[8];
        snprintf(time, sizeof(time), "%02u:%02u", alarm_edit.hour,
                 alarm_edit.minute);
        draw_text(frame, "ALARM", centered_text_x("ALARM", 2), 62, 2,
                  wire_rgb565(255, 98, 108));
        const int width = time_text_width(time);
        draw_time_text(frame, time, (BOARD_DISPLAY_WIDTH - width) / 2,
                       147, wire_rgb565(250, 250, 255));
        for (int y = 292; y <= 378; y++) {
            for (int x = 55; x <= 355; x++) {
                const bool border = x < 60 || x > 350 || y < 297 || y > 373;
                frame_set_content(frame, x, y,
                                  border ? wire_rgb565(255, 82, 96)
                                         : wire_rgb565(58, 5, 14));
            }
        }
        draw_text(frame, "STOP", centered_text_x("STOP", 2), 322, 2,
                  wire_rgb565(255, 244, 246));
    } else {
        draw_alarm_rolling_time(frame);
        draw_alarm_toggle(frame);
        const bubble_t launcher = {
            205, 430, 40, ICON_LAUNCHER, LAUNCHER_ACTION_NONE};
        draw_bubble(frame, &launcher);
        draw_icon(frame, ICON_LAUNCHER, 205, 430, 52);
    }
    draw_contour(frame);
}

static void draw_weather_line(uint16_t *frame, int x0, int y0, int x1, int y1,
                              int width, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        for (int y = y0 - width; y <= y0 + width; y++) {
            for (int x = x0 - width; x <= x0 + width; x++) {
                frame_set_content(frame, x, y, color);
            }
        }
        if (x0 == x1 && y0 == y1) break;
        int doubled = 2 * error;
        if (doubled >= dy) {
            error += dy;
            x0 += sx;
        }
        if (doubled <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

static void draw_weather_disc(uint16_t *frame, int center_x, int center_y,
                              int radius, uint16_t color)
{
    for (int y = center_y - radius; y <= center_y + radius; y++) {
        for (int x = center_x - radius; x <= center_x + radius; x++) {
            int dx = x - center_x;
            int dy = y - center_y;
            if (dx * dx + dy * dy <= radius * radius) {
                frame_set_content(frame, x, y, color);
            }
        }
    }
}

static void draw_weather_cloud(uint16_t *frame, int center_x, int center_y,
                               uint16_t color)
{
    draw_weather_disc(frame, center_x - 28, center_y + 4, 22, color);
    draw_weather_disc(frame, center_x, center_y - 9, 30, color);
    draw_weather_disc(frame, center_x + 32, center_y + 5, 21, color);
    for (int y = center_y; y <= center_y + 25; y++) {
        for (int x = center_x - 48; x <= center_x + 50; x++) {
            frame_set_content(frame, x, y, color);
        }
    }
}

static void draw_weather_symbol_fallback(uint16_t *frame,
                                         uint16_t condition_id)
{
    const int center_x = 205;
    const int center_y = 154;
    const uint16_t primary = theme_wire_tinted(85);
    const uint16_t white = wire_rgb565(235, 242, 250);
    if (condition_id == 800) {
        draw_weather_disc(frame, center_x, center_y, 32, primary);
        for (int ray = 0; ray < 8; ray++) {
            double angle = ray * 3.141592653589793 / 4.0;
            draw_weather_line(frame,
                center_x + (int)(cos(angle) * 43),
                center_y + (int)(sin(angle) * 43),
                center_x + (int)(cos(angle) * 61),
                center_y + (int)(sin(angle) * 61), 2, primary);
        }
        return;
    }
    if (condition_id >= 700 && condition_id < 800) {
        for (int offset = -32; offset <= 32; offset += 16) {
            draw_weather_line(frame, center_x - 58, center_y + offset,
                              center_x + 58, center_y + offset, 3, primary);
        }
        return;
    }
    if (condition_id >= 600 && condition_id < 700) {
        for (int arm = 0; arm < 3; arm++) {
            double angle = arm * 3.141592653589793 / 3.0;
            int dx = (int)(cos(angle) * 54);
            int dy = (int)(sin(angle) * 54);
            draw_weather_line(frame, center_x - dx, center_y - dy,
                              center_x + dx, center_y + dy, 2, white);
        }
        return;
    }
    draw_weather_cloud(frame, center_x, center_y - 8, white);
    if (condition_id >= 200 && condition_id < 300) {
        draw_weather_line(frame, center_x + 8, center_y + 12,
                          center_x - 8, center_y + 44, 5, primary);
        draw_weather_line(frame, center_x - 8, center_y + 44,
                          center_x + 6, center_y + 41, 5, primary);
    } else if (condition_id >= 300 && condition_id < 600) {
        for (int x = center_x - 30; x <= center_x + 30; x += 30) {
            draw_weather_line(frame, x + 8, center_y + 32,
                              x - 3, center_y + 55, 3, primary);
        }
    }
}

static int weather_picture_index(uint16_t condition_id)
{
    if (condition_id >= 200 && condition_id < 300) return 6;
    if (condition_id >= 300 && condition_id < 400) return 4;
    if (condition_id == 511) return 7;
    if (condition_id >= 500 && condition_id <= 504) return 5;
    if (condition_id >= 520 && condition_id < 600) return 4;
    if (condition_id >= 600 && condition_id < 700) return 7;
    if (condition_id >= 700 && condition_id < 800) return 8;
    if (condition_id == 800) return 0;
    if (condition_id == 801) return 1;
    if (condition_id == 802) return 2;
    if (condition_id == 803 || condition_id == 804) return 3;
    return 3;
}

static void draw_weather_picture(uint16_t *frame, uint16_t condition_id)
{
    if (weather_atlas == NULL) {
        draw_weather_symbol_fallback(frame, condition_id);
        return;
    }
    const int picture = weather_picture_index(condition_id);
    const int left = (BOARD_DISPLAY_WIDTH - WEATHER_PICTURE_SIZE) / 2;
    const int top = 96;
    const size_t picture_offset = (size_t)picture * WEATHER_PICTURE_BYTES;
    for (int y = 0; y < WEATHER_PICTURE_SIZE; y++) {
        for (int x = 0; x < WEATHER_PICTURE_SIZE; x++) {
            size_t offset = picture_offset +
                ((size_t)y * WEATHER_PICTURE_SIZE + x) * 2;
            uint16_t pixel = (uint16_t)weather_atlas[offset] |
                             ((uint16_t)weather_atlas[offset + 1] << 8);
            frame_set_content(frame, left + x, top + y, pixel);
        }
    }
}

static void draw_weather_control(uint16_t *frame, int center_x, int center_y,
                                 bool points_right, bool enabled)
{
    const uint16_t edge = enabled ? theme_wire_color()
                                  : wire_rgb565(52, 58, 68);
    const uint16_t fill = enabled ? theme_wire_scaled(38)
                                  : wire_rgb565(7, 10, 15);
    draw_weather_disc(frame, center_x, center_y, 34, edge);
    draw_weather_disc(frame, center_x, center_y, 29, fill);
    int direction = points_right ? 1 : -1;
    draw_weather_line(frame, center_x - 8 * direction, center_y - 13,
                      center_x + 8 * direction, center_y, 3, edge);
    draw_weather_line(frame, center_x + 8 * direction, center_y,
                      center_x - 8 * direction, center_y + 13, 3, edge);
}

static void compose_weather_frame(uint16_t *frame)
{
    memset(frame, 0, DISPLAY_FRAME_BYTES);
    weather_snapshot_t snapshot;
    weather_get_snapshot(&snapshot);
    draw_text(frame, "WETTER", centered_text_x("WETTER", 3), 44, 3,
              theme_wire_tinted(72));

    const bool ready = snapshot.state == WEATHER_READY_ONLINE ||
                       snapshot.state == WEATHER_READY_CACHE;
    if (ready && snapshot.day_count != 0) {
        if (weather_selected_day >= snapshot.day_count) {
            weather_selected_day = snapshot.day_count - 1;
        }
        weather_day_t *day = &snapshot.days[weather_selected_day];
        time_t local_time = (time_t)(day->timestamp +
                                     snapshot.timezone_offset_seconds);
        struct tm calendar;
        gmtime_r(&local_time, &calendar);
        static const char *weekdays[] = {
            "SO", "MO", "DI", "MI", "DO", "FR", "SA"};
        char heading[32];
        unsigned month_day = (unsigned)calendar.tm_mday;
        unsigned month = (unsigned)(calendar.tm_mon + 1);
        if (weather_selected_day == 0) {
            snprintf(heading, sizeof(heading), "HEUTE  %02u.%02u.",
                     month_day, month);
        } else {
            snprintf(heading, sizeof(heading), "%s  %02u.%02u.",
                     weekdays[calendar.tm_wday], month_day, month);
        }
        draw_text(frame, heading, centered_text_x(heading, 4), 82, 4,
                  theme_wire_tinted(120));
        draw_weather_picture(frame, day->condition_id);

        char temperature[10];
        int rounded = day->temperature_tenths >= 0
                          ? (day->temperature_tenths + 5) / 10
                          : (day->temperature_tenths - 5) / 10;
        snprintf(temperature, sizeof(temperature), "%d\260", rounded);
        draw_text(frame, temperature, centered_text_x(temperature, 1),
                  245, 1, wire_rgb565(246, 249, 255));
        char range[24];
        snprintf(range, sizeof(range), "MIN %d\260  MAX %d\260",
                 day->minimum_tenths / 10, day->maximum_tenths / 10);
        draw_text(frame, range, centered_text_x(range, 4), 322, 4,
                  theme_wire_tinted(125));
        char source[24];
        if (snapshot.cache_updated_at > 0) {
            time_t cache_time = (time_t)(snapshot.cache_updated_at +
                                         snapshot.timezone_offset_seconds);
            struct tm cache_calendar;
            gmtime_r(&cache_time, &cache_calendar);
            snprintf(source, sizeof(source), "%s %02d:%02d",
                     snapshot.state == WEATHER_READY_ONLINE
                         ? "AKTUELL"
                         : "CACHE",
                     cache_calendar.tm_hour, cache_calendar.tm_min);
        } else {
            snprintf(source, sizeof(source), "%s",
                     snapshot.state == WEATHER_READY_ONLINE
                         ? "AKTUELL"
                         : "SD CACHE");
        }
        draw_text(frame, source, centered_text_x(source, 6), 356, 6,
                  snapshot.state == WEATHER_READY_ONLINE
                      ? theme_wire_tinted(125)
                      : wire_rgb565(150, 158, 172));
    } else {
        const char *message = "WIRD GELADEN";
        if (snapshot.state == WEATHER_NO_POSITION) {
            message = "GPS POSITION FEHLT";
        } else if (snapshot.state == WEATHER_NO_API_KEY) {
            message = "API KEY FEHLT";
        } else if (snapshot.state == WEATHER_API_ACCESS_ERROR) {
            message = "ONE CALL NICHT AKTIV";
        } else if (snapshot.state == WEATHER_NO_CACHE) {
            message = "WLAN AUS  KEIN CACHE";
        } else if (snapshot.state == WEATHER_NETWORK_ERROR) {
            message = "WETTER NICHT ERREICHBAR";
        } else if (snapshot.state == WEATHER_DATA_ERROR) {
            message = "WETTERDATEN FEHLER";
        } else if (snapshot.state == WEATHER_SD_ERROR) {
            message = "SD KARTE FEHLT";
        }
        draw_text(frame, message, centered_text_x(message, 4), 210, 4,
                  snapshot.state == WEATHER_LOADING
                      ? theme_wire_tinted(100)
                      : wire_rgb565(255, 102, 112));
    }

    draw_weather_control(frame, 70, 420, false,
                         ready && weather_selected_day > 0);
    draw_weather_control(frame, 340, 420, true,
                         ready && weather_selected_day + 1 <
                                      snapshot.day_count);
    const bubble_t launcher = {
        205, 435, 38, ICON_LAUNCHER, LAUNCHER_ACTION_NONE};
    draw_bubble(frame, &launcher);
    draw_icon(frame, ICON_LAUNCHER, 205, 435, 50);
    draw_contour(frame);
}

static void mesh_text_line(char *output, size_t output_size,
                           const char *input, size_t maximum_characters)
{
    size_t length = strnlen(input, maximum_characters);
    if (length >= output_size) length = output_size - 1;
    memmove(output, input, length);
    output[length] = '\0';
    for (size_t index = 0; index < length; index++) {
        if ((unsigned char)output[index] < 0x20) output[index] = ' ';
    }
}

static void compose_messages_frame(uint16_t *frame)
{
    memset(frame, 0, DISPLAY_FRAME_BYTES);
    draw_text(frame, "MESSAGES", centered_text_x("MESSAGES", 3), 38, 3,
              theme_wire_tinted(72));

    mesh_config_t config;
    mesh_service_get_config(&config);
    const bool active = mesh_service_radio_active();
    const char *radio_state = !config.enabled ? "LORA OFF" :
                              active ? "LORA RECEIVING" : "LORA STARTING";
    draw_text(frame, radio_state, centered_text_x(radio_state, 5), 78, 5,
              active ? wire_rgb565(105, 238, 166)
                     : wire_rgb565(180, 188, 202));

    const uint16_t edge = config.enabled ? theme_wire_color()
                                         : wire_rgb565(70, 78, 92);
    for (int y = 108; y <= 166; y++) {
        for (int x = 70; x <= 340; x++) {
            const bool border = x < 74 || x > 336 || y < 112 || y > 162;
            frame_set_content(frame, x, y,
                              border ? edge : theme_wire_scaled(20));
        }
    }
    const char *toggle = config.enabled ? "TURN LORA OFF" : "TURN LORA ON";
    draw_text(frame, toggle, centered_text_x(toggle, 4), 126, 4, edge);

    mesh_message_t recent[5];
    size_t count = mesh_service_get_recent(recent, 5);
    if (count == 0) {
        draw_text(frame, "NO MESSAGES YET",
                  centered_text_x("NO MESSAGES YET", 4), 236, 4,
                  wire_rgb565(145, 153, 168));
    }
    for (size_t index = 0; index < count; index++) {
        const int top = 184 + (int)index * 48;
        char sender[18];
        mesh_text_line(sender, sizeof(sender), recent[index].sender, 17);
        char header[64];
        snprintf(header, sizeof(header), "%.17s  %.31s  %ddBm", sender,
                 recent[index].channel[0] != '\0' ? recent[index].channel :
                 "UNKNOWN", recent[index].rssi_dbm);
        mesh_text_line(header, sizeof(header), header, 52);
        draw_text(frame, header, 42, top, 6,
                  recent[index].kind == MESH_MESSAGE_UNKNOWN
                      ? wire_rgb565(118, 118, 105)
                      : theme_wire_tinted(115));
        char body[43];
        mesh_text_line(body, sizeof(body), recent[index].text, 42);
        draw_text(frame, body, 42, top + 18, 5,
                  wire_rgb565(230, 234, 242));
    }

    const bubble_t launcher = {
        205, 445, 36, ICON_LAUNCHER, LAUNCHER_ACTION_NONE};
    draw_bubble(frame, &launcher);
    draw_icon(frame, ICON_LAUNCHER, 205, 445, 48);
    draw_contour(frame);
}

static void draw_map_round_control(uint16_t *frame, int center_x, int center_y,
                                   int radius, bool enabled)
{
    const uint16_t edge = enabled ? theme_wire_color()
                                  : wire_rgb565(70, 78, 92);
    const uint16_t fill = enabled ? theme_wire_scaled(44)
                                  : wire_rgb565(10, 13, 19);
    for (int y = center_y - radius; y <= center_y + radius; y++) {
        for (int x = center_x - radius; x <= center_x + radius; x++) {
            int dx = x - center_x;
            int dy = y - center_y;
            int distance = dx * dx + dy * dy;
            if (distance <= radius * radius) {
                frame_set_content(frame, x, y,
                                  distance >= (radius - 4) * (radius - 4)
                                      ? edge : fill);
            }
        }
    }
}

static void draw_map_marker(uint16_t *frame, int center_x, int center_y)
{
    for (int y = center_y - 15; y <= center_y + 15; y++) {
        for (int x = center_x - 15; x <= center_x + 15; x++) {
            int dx = x - center_x;
            int dy = y - center_y;
            int distance = dx * dx + dy * dy;
            if ((distance >= 100 && distance <= 196) ||
                (abs(dx) <= 2 && dy >= -10 && dy <= 10) ||
                (abs(dy) <= 2 && dx >= -10 && dx <= 10)) {
                frame_set_content(frame, x, y, theme_wire_tinted(48));
            }
        }
    }
}

static void draw_map_control_icon(uint16_t *frame, icon_id_t icon,
                                  int center_x, int center_y, bool enabled)
{
    const uint16_t color = enabled ? theme_wire_tinted(72)
                                   : wire_rgb565(90, 96, 108);
    const int size = 52;
    for (int y = center_y - size / 2; y < center_y + size / 2; y++) {
        for (int x = center_x - size / 2; x < center_x + size / 2; x++) {
            if (icon_pixel_sized(icon, center_x, center_y, size, x, y) != 0) {
                frame_set_content(frame, x, y, color);
            }
        }
    }
}

static void draw_map_pan_control(uint16_t *frame, int center_x, int center_y,
                                 int direction_x, int direction_y)
{
    draw_map_round_control(frame, center_x, center_y, 22, true);
    const uint16_t color = theme_wire_tinted(72);
    for (int y = center_y - 11; y <= center_y + 11; y++) {
        for (int x = center_x - 11; x <= center_x + 11; x++) {
            const int dx = x - center_x;
            const int dy = y - center_y;
            const int forward = dx * direction_x + dy * direction_y;
            const int side = -dx * direction_y + dy * direction_x;
            const bool shaft = forward >= -10 && forward <= 2 &&
                               abs(side) <= 2;
            const bool head = forward >= 2 && forward <= 11 &&
                              abs(side) <= 11 - forward;
            if (shaft || head) {
                frame_set_content(frame, x, y, color);
            }
        }
    }
}

static void compose_map_frame(uint16_t *frame)
{
    offline_map_snapshot_t snapshot = {.state = OFFLINE_MAP_LOADING,
                                       .following = true};
    if (!offline_map_copy_frame(frame, &snapshot)) {
        memset(frame, 0, DISPLAY_FRAME_BYTES);
    }
    for (int y = 0; y < BOARD_DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < BOARD_DISPLAY_WIDTH; x++) {
            if (!pixel_is_safe_content(x, y)) {
                frame[y * BOARD_DISPLAY_WIDTH + x] = 0;
            }
        }
    }

    if (snapshot.state == OFFLINE_MAP_LOADING ||
        snapshot.state == OFFLINE_MAP_GPS_SEARCH ||
        snapshot.state == OFFLINE_MAP_ERROR) {
        clear_content_rect(frame, 105, 20, 200, 42);
        const char *status = snapshot.state == OFFLINE_MAP_LOADING
                                 ? "KARTE"
                                 : snapshot.state == OFFLINE_MAP_GPS_SEARCH
                                       ? "GPS SUCHE" : snapshot.error;
        draw_text(frame, status, centered_text_x(status, 5), 29, 5,
                  snapshot.state == OFFLINE_MAP_ERROR
                      ? wire_rgb565(255, 98, 108)
                      : theme_wire_tinted(64));
    }

    draw_map_round_control(frame, 350, 74, 28, snapshot.can_zoom_in);
    draw_map_control_icon(frame, ICON_ZOOM_IN, 350, 74,
                          snapshot.can_zoom_in);
    draw_map_round_control(frame, 350, 142, 28, snapshot.can_zoom_out);
    draw_map_control_icon(frame, ICON_ZOOM_OUT, 350, 142,
                          snapshot.can_zoom_out);
    draw_map_round_control(frame, 350, 210, 28, true);
    draw_map_control_icon(frame, ICON_MAP_CENTER, 350, 210, true);
    draw_map_pan_control(frame, 94, 68, 0, -1);
    draw_map_pan_control(frame, 94, 172, 0, 1);
    draw_map_pan_control(frame, 42, 120, -1, 0);
    draw_map_pan_control(frame, 146, 120, 1, 0);
    if (snapshot.gps_fix && snapshot.marker_x >= 18 &&
        snapshot.marker_x < BOARD_DISPLAY_WIDTH - 18 &&
        snapshot.marker_y >= 18 && snapshot.marker_y < BOARD_DISPLAY_HEIGHT - 18) {
        draw_map_marker(frame, snapshot.marker_x, snapshot.marker_y);
    }
    const bubble_t launcher = {
        205, 445, 36, ICON_LAUNCHER, LAUNCHER_ACTION_NONE};
    draw_bubble(frame, &launcher);
    draw_icon(frame, ICON_LAUNCHER, 205, 445, 48);

    draw_contour(frame);
}

static void display_frame_region(const uint16_t *frame, int x, int y,
                                 int width, int height, bool synchronize)
{
    if (synchronize) {
        wait_for_te_edge();
    }
    int buffer_index = 0;
    for (int band_y = y; band_y < y + height; band_y += DISPLAY_BAND_ROWS) {
        int rows = y + height - band_y;
        if (rows > DISPLAY_BAND_ROWS) {
            rows = DISPLAY_BAND_ROWS;
        }
        for (int row = 0; row < rows; row++) {
            memcpy(&band_pixels[buffer_index][row * width],
                   &frame[(band_y + row) * BOARD_DISPLAY_WIDTH + x],
                   width * sizeof(uint16_t));
        }
        display_wait_for_transfer();
        set_address_window(x, band_y, width, rows);
        if (!display_queue_color_band(band_pixels[buffer_index], width * rows)) {
            ESP_LOGW(TAG, "display update stopped at row %d", band_y);
            break;
        }
        buffer_index = (buffer_index + 1) % DISPLAY_BUFFER_COUNT;
    }
    display_wait_for_transfer();
}

static void update_watch_cache(bool transfer_changes)
{
    watch_values_t values;
    read_watch_values(&values);
    const bool time_changed = !displayed_watch_values_valid ||
        strcmp(values.time, displayed_watch_values.time) != 0;
    const bool date_changed = !displayed_watch_values_valid ||
        strcmp(values.date, displayed_watch_values.date) != 0;
    const bool alarm_changed = !displayed_watch_values_valid ||
        values.alarm_enabled != displayed_watch_values.alarm_enabled;
    const bool status_changed = !displayed_watch_values_valid ||
        strcmp(values.battery, displayed_watch_values.battery) != 0 ||
        values.ble_enabled != displayed_watch_values.ble_enabled ||
        values.wifi_state != displayed_watch_values.wifi_state;

    if (time_changed) {
        draw_watch_time(watch_frame, &values);
        draw_watch_alarm(watch_frame, &values);
        if (transfer_changes) {
            display_frame_region(
                watch_frame, WATCH_TIME_REGION_X, WATCH_TIME_REGION_Y,
                WATCH_TIME_REGION_WIDTH, WATCH_TIME_REGION_HEIGHT, true);
        }
    }
    if (date_changed) {
        draw_watch_date(watch_frame, &values);
        if (transfer_changes) {
            display_frame_region(watch_frame, 70, 45, 270, 42, false);
        }
    }
    if (alarm_changed) {
        draw_watch_alarm(watch_frame, &values);
        if (transfer_changes) {
            display_frame_region(watch_frame, 185, 230, 40, 40, false);
        }
    }
    if (status_changed) {
        draw_watch_status(watch_frame, &values);
        if (transfer_changes) {
            display_frame_region(watch_frame, 40, 390, 330, 40, false);
        }
    }
    displayed_watch_values = values;
    displayed_watch_values_valid = true;
}

static void update_launcher_cache(bool transfer_change)
{
    watch_values_t values;
    read_watch_values(&values);
    const bool ble_changed =
        values.ble_enabled != displayed_launcher_ble_enabled;
    const bool wifi_changed =
        values.wifi_state != displayed_launcher_wifi_state;
    const bool battery_changed =
        strcmp(values.battery, displayed_launcher_battery) != 0;
    if (ble_changed) {
        draw_launcher_ble(launcher_frame, values.ble_enabled);
        displayed_launcher_ble_enabled = values.ble_enabled;
        if (transfer_change) {
            display_frame_region(launcher_frame, 42, 430, 56, 40, false);
        }
    }
    if (wifi_changed) {
        draw_launcher_wifi(launcher_frame, values.wifi_state);
        displayed_launcher_wifi_state = values.wifi_state;
        if (transfer_change) {
            display_frame_region(launcher_frame, 92, 430, 46, 40, false);
        }
    }
    if (battery_changed) {
        draw_launcher_battery(launcher_frame, values.battery);
        strcpy(displayed_launcher_battery, values.battery);
        if (transfer_change) {
            display_frame_region(launcher_frame, 295, 430, 95, 40, false);
        }
    }
}

static void rebuild_theme_frames(void)
{
    watch_values_t values;
    read_watch_values(&values);
    compose_watch_frame(watch_frame, &values);
    compose_launcher_frame(launcher_frame, &values);
    compose_settings_frame(settings_frame);
    compose_alarm_frame(alarm_frame);
    if (active_screen == UI_MAP) {
        compose_map_frame(map_frame);
    } else if (active_screen == UI_WEATHER) {
        compose_weather_frame(weather_frame);
    } else if (active_screen == UI_MESSAGES) {
        compose_messages_frame(messages_frame);
    }
    displayed_watch_values = values;
    displayed_watch_values_valid = true;
    strcpy(displayed_launcher_battery, values.battery);
    displayed_launcher_ble_enabled = values.ble_enabled;
    displayed_launcher_wifi_state = values.wifi_state;
}

static int64_t present_screen(ui_screen_t screen)
{
    const int64_t started_us = esp_timer_get_time();
    ESP_ERROR_CHECK(display_apply_brightness(0));
    panel_hidden = true;
    const uint16_t *frame = watch_frame;
    if (screen == UI_WATCH) {
        update_watch_cache(false);
    } else if (screen == UI_LAUNCHER) {
        update_launcher_cache(false);
        frame = launcher_frame;
    } else if (screen == UI_SETTINGS) {
        draw_settings_slider(settings_frame);
        draw_settings_ble(settings_frame);
        draw_settings_wifi(settings_frame);
        frame = settings_frame;
    } else if (screen == UI_ALARM) {
        compose_alarm_frame(alarm_frame);
        frame = alarm_frame;
    } else if (screen == UI_MAP) {
        compose_map_frame(map_frame);
        frame = map_frame;
    } else if (screen == UI_WEATHER) {
        compose_weather_frame(weather_frame);
        frame = weather_frame;
    } else if (screen == UI_MESSAGES) {
        compose_messages_frame(messages_frame);
        frame = messages_frame;
    }
    display_frame_region(frame, 0, 0, BOARD_DISPLAY_WIDTH,
                         BOARD_DISPLAY_HEIGHT, true);
    ESP_ERROR_CHECK(display_apply_brightness(display_brightness_percentage));
    panel_hidden = false;
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

static void refresh_active_screen(void)
{
    if (panel_hidden || active_screen == UI_BLACK) {
        return;
    }
    if (active_screen == UI_WATCH) {
        update_watch_cache(true);
    } else if (active_screen == UI_LAUNCHER) {
        update_launcher_cache(true);
    } else if (active_screen == UI_SETTINGS) {
        draw_settings_slider(settings_frame);
        draw_settings_ble(settings_frame);
        draw_settings_wifi(settings_frame);
        display_frame_region(settings_frame, 35, 120, 340, 104, false);
        display_frame_region(settings_frame, 40, 230, 330, 140, false);
    } else if (active_screen == UI_ALARM) {
        if (!alarm_swipe_active) {
            ble_alarm_get_config(&alarm_edit);
        }
        compose_alarm_frame(alarm_frame);
        display_frame_region(alarm_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                             BOARD_DISPLAY_HEIGHT, false);
    } else if (active_screen == UI_MAP) {
        compose_map_frame(map_frame);
        display_frame_region(map_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                             BOARD_DISPLAY_HEIGHT, false);
    } else if (active_screen == UI_WEATHER) {
        compose_weather_frame(weather_frame);
        display_frame_region(weather_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                             BOARD_DISPLAY_HEIGHT, false);
    } else if (active_screen == UI_MESSAGES) {
        compose_messages_frame(messages_frame);
        display_frame_region(messages_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                             BOARD_DISPLAY_HEIGHT, false);
    }
}

static void ui_task(void *parameter)
{
    (void)parameter;
    last_touch_tick = xTaskGetTickCount();
    watch_values_t initial_values;
    read_watch_values(&initial_values);
    int64_t first_frame_started_us = esp_timer_get_time();
    compose_watch_frame(watch_frame, &initial_values);
    displayed_watch_values = initial_values;
    displayed_watch_values_valid = true;
    display_frame_region(watch_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                         BOARD_DISPLAY_HEIGHT, false);
    ESP_ERROR_CHECK(display_apply_brightness(display_brightness_percentage));
    panel_hidden = false;
    int64_t first_frame_us = esp_timer_get_time() - first_frame_started_us;
    ESP_LOGI(TAG, "first Watch frame rendered");
    ESP_LOGI(TAG, "Watch frame render: %lld ms", first_frame_us / 1000);
    xTaskNotifyGive(bootstrap_task_handle);
    uint32_t bootstrap_events = 0;
    do {
        xTaskNotifyWait(0, UINT32_MAX, &bootstrap_events, portMAX_DELAY);
    } while ((bootstrap_events & UI_EVENT_ASSETS_READY) == 0);

    if (assets_refresh_pending) {
        draw_watch_launcher(watch_frame);
        draw_watch_status(watch_frame, &displayed_watch_values);
        display_frame_region(watch_frame, 165, 370, 80, 80, false);
        display_frame_region(watch_frame, 40, 390, 330, 40, false);
        assets_refresh_pending = false;
    }
    bool refresh_pending = true;
    for (;;) {
        bool navigation_pending = process_ui_input();
        if (theme_refresh_pending) {
            theme_refresh_pending = false;
            rebuild_theme_frames();
            if (!panel_hidden && active_screen != UI_BLACK) {
                present_screen(active_screen);
            }
            refresh_pending = false;
            ESP_LOGI(TAG, "theme color applied");
        }
        if (wake_restore_pending) {
            update_watch_cache(true);
            ESP_ERROR_CHECK(display_apply_brightness(
                display_brightness_percentage));
            panel_hidden = false;
            wake_restore_pending = false;
            refresh_pending = false;
            ESP_LOGI(TAG, "Watch restored after touch");
        }
        if (navigation_pending) {
            ui_screen_t screen = active_screen;
            int64_t render_us = present_screen(screen);
            if (button_down_us != 0) {
                int64_t response_us = esp_timer_get_time() - button_down_us;
                ESP_LOGI(TAG, "UI response: %lld ms total, %lld ms presenting",
                         response_us / 1000, render_us / 1000);
                button_down_us = 0;
            }
            refresh_pending = false;
        }
        if (settings_slider_dirty && active_screen == UI_SETTINGS &&
            !panel_hidden) {
            draw_settings_slider(settings_frame);
            display_frame_region(settings_frame, 35, 120, 340, 104, false);
            settings_slider_dirty = false;
        }
        if (settings_ble_dirty && active_screen == UI_SETTINGS &&
            !panel_hidden) {
            draw_settings_ble(settings_frame);
            display_frame_region(settings_frame, 40, 230, 330, 67, false);
            settings_ble_dirty = false;
        }
        if (settings_wifi_dirty && active_screen == UI_SETTINGS &&
            !panel_hidden) {
            draw_settings_wifi(settings_frame);
            display_frame_region(settings_frame, 40, 303, 330, 67, false);
            settings_wifi_dirty = false;
        }
        if (alarm_controls_dirty && active_screen == UI_ALARM &&
            !panel_hidden) {
            compose_alarm_frame(alarm_frame);
            display_frame_region(alarm_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                                 BOARD_DISPLAY_HEIGHT, false);
            alarm_controls_dirty = false;
        }
        if (refresh_pending) {
            refresh_active_screen();
            refresh_pending = false;
        }

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - last_touch_tick;
        if (active_screen != UI_BLACK &&
            !screen_keeps_display_awake(active_screen) &&
            !ble_alarm_is_ringing() &&
            elapsed >= SCREEN_IDLE_TICKS) {
            ui_screen_t previous_screen = active_screen;
            if (previous_screen == UI_MAP) {
                offline_map_stop();
            }
            ESP_ERROR_CHECK(display_apply_brightness(0));
            panel_hidden = true;
            update_watch_cache(previous_screen == UI_WATCH);
            if (previous_screen != UI_WATCH) {
                display_frame_region(watch_frame, 0, 0, BOARD_DISPLAY_WIDTH,
                                     BOARD_DISPLAY_HEIGHT, false);
            }
            active_screen = UI_BLACK;
            button_down_us = 0;
            ESP_LOGI(TAG, "display black; Watch staged for wake");
        }
        if (active_screen == UI_BLACK) {
            uint32_t events;
            xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);
            continue;
        }
        now = xTaskGetTickCount();
        elapsed = now - last_touch_tick;
        TickType_t idle_wait = screen_keeps_display_awake(active_screen)
                                   ? portMAX_DELAY
                                   : (elapsed >= SCREEN_IDLE_TICKS
                                          ? 0
                                          : SCREEN_IDLE_TICKS - elapsed);
        TickType_t minute_wait = minute_wait_ticks();
        TickType_t wait = minute_wait < idle_wait ? minute_wait : idle_wait;
        uint32_t events = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &events, wait) == pdFALSE ||
            (events & UI_EVENT_REFRESH) != 0) {
            refresh_pending = true;
        }
    }
}

static void load_icon_atlas_from_sd(void)
{
    esp_err_t result = sd_storage_acquire();
    if (result != ESP_OK) {
        ESP_LOGI(TAG, "SD unavailable; using procedural UI symbols: %s",
                 esp_err_to_name(result));
        return;
    }
    {
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
        } else if (file_size != ICON_ATLAS_BYTES &&
                   file_size != CURRENT_ICON_ATLAS_BYTES &&
                   file_size != MAP_ICON_ATLAS_BYTES &&
                   file_size != PREVIOUS_ICON_ATLAS_BYTES &&
                   file_size != LEGACY_ICON_ATLAS_BYTES) {
            ESP_LOGW(TAG,
                     "atlas is %ld bytes; expected %u, %u, %u, %u, or %u",
                     file_size, ICON_ATLAS_BYTES, CURRENT_ICON_ATLAS_BYTES,
                     MAP_ICON_ATLAS_BYTES,
                     PREVIOUS_ICON_ATLAS_BYTES, LEGACY_ICON_ATLAS_BYTES);
        } else {
            uint8_t *candidate = heap_caps_malloc(
                (size_t)file_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (candidate != NULL &&
                fread(candidate, 1, (size_t)file_size, file) ==
                    (size_t)file_size) {
                icon_atlas = candidate;
                icon_atlas_tile_count = (size_t)file_size / ICON_TILE_BYTES;
                ESP_LOGI(TAG, "loaded %ld-byte UI icon atlas (%u tiles)",
                         file_size, (unsigned)icon_atlas_tile_count);
            } else {
                free(candidate);
                ESP_LOGW(TAG, "UI icon atlas read failed; using fallback symbols");
            }
        }
        if (file != NULL) {
            fclose(file);
        }
    }
    {
        FILE *file = fopen(WEATHER_ATLAS_PATH, "rb");
        long file_size = -1;
        if (file == NULL) {
            ESP_LOGW(TAG, "cannot open %s: errno=%d (%s)",
                     WEATHER_ATLAS_PATH, errno, strerror(errno));
        } else if (fseek(file, 0, SEEK_END) != 0 ||
                   (file_size = ftell(file)) < 0 ||
                   fseek(file, 0, SEEK_SET) != 0) {
            ESP_LOGW(TAG, "cannot determine weather atlas size: errno=%d (%s)",
                     errno, strerror(errno));
        } else if (file_size != WEATHER_ATLAS_BYTES) {
            ESP_LOGW(TAG, "weather atlas is %ld bytes; expected %u",
                     file_size, WEATHER_ATLAS_BYTES);
        } else {
            uint8_t *candidate = heap_caps_malloc(
                WEATHER_ATLAS_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (candidate != NULL &&
                fread(candidate, 1, WEATHER_ATLAS_BYTES, file) ==
                    WEATHER_ATLAS_BYTES) {
                weather_atlas = candidate;
                ESP_LOGI(TAG, "loaded %u-byte weather picture atlas",
                         WEATHER_ATLAS_BYTES);
            } else {
                free(candidate);
                ESP_LOGW(TAG,
                         "weather atlas read failed; using fallback symbols");
            }
        }
        if (file != NULL) {
            fclose(file);
        }
    }
    sd_storage_release();
    ESP_LOGI(TAG, "SD power disabled after asset load attempt");
}

void app_main(void)
{
    initialize_cjson_allocator();
    initialize_i2c();
    ESP_ERROR_CHECK(ble_rtc_initialize());
    enable_display_power();
    initialize_display();
    initialize_display_bounds();
    initialize_glyph_indices();
    for (int index = 0; index < DISPLAY_BUFFER_COUNT; index++) {
        band_pixels[index] = heap_caps_malloc(
            BOARD_DISPLAY_WIDTH * DISPLAY_BAND_ROWS *
                sizeof(*band_pixels[index]),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        ESP_ERROR_CHECK(band_pixels[index] == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    }
    watch_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    launcher_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    settings_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    alarm_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    map_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    weather_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    messages_frame = heap_caps_malloc(DISPLAY_FRAME_BYTES,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(watch_frame == NULL || launcher_frame == NULL ||
                            settings_frame == NULL || alarm_frame == NULL ||
                            map_frame == NULL || weather_frame == NULL ||
                            messages_frame == NULL
                        ? ESP_ERR_NO_MEM : ESP_OK);

    bootstrap_task_handle = xTaskGetCurrentTaskHandle();
    BaseType_t task_result = xTaskCreate(ui_task, "ui", 6144, NULL, 4,
                                         &ui_task_handle);
    ESP_ERROR_CHECK(task_result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    load_icon_atlas_from_sd();
    watch_values_t cached_values;
    read_watch_values(&cached_values);
    draw_watch_launcher(watch_frame);
    compose_launcher_frame(launcher_frame, &cached_values);
    strcpy(displayed_launcher_battery, cached_values.battery);
    displayed_launcher_ble_enabled = cached_values.ble_enabled;
    displayed_launcher_wifi_state = cached_values.wifi_state;
    compose_settings_frame(settings_frame);
    compose_alarm_frame(alarm_frame);
    assets_refresh_pending = icon_atlas != NULL || weather_atlas != NULL;
    xTaskNotify(ui_task_handle, UI_EVENT_ASSETS_READY, eSetBits);
    ESP_ERROR_CHECK(ble_rtc_start());
}
