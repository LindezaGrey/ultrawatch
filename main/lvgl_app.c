#include "lvgl_app.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "twatch_display.h"
#include "twatch_board.h"
#include "twatch_brookesia_bridge.h"

static const char *TAG = "lvgl_app";
static lv_display_t *display;
static lv_color_t draw_buffer[410 * 16];
static lv_indev_t *touch_input;
static lv_obj_t *utc_label;
static lv_obj_t *menu_panel;
static lv_obj_t *menu_text;
static lv_obj_t *menu_screen;
static lv_obj_t *home_screen;
static lv_obj_t *diagnostics_screen;
static lv_obj_t *diagnostics_label;
static bool display_inverted = true;

/* The LilyGO Ultra port expands partial invalidation areas to the panel's
 * left edge; CO5300 updates can otherwise shift narrow redraws sideways. */
static void rounder_cb(lv_event_t *event)
{
    lv_area_t *area = (lv_area_t *)lv_event_get_param(event);
    if (area->x1 > 1) area->x1 = 0;
    if (!(area->x2 & 1)) area->x2++;
    if (area->y1 & 1) area->y1--;
    if (!(area->y2 & 1)) area->y2++;
}

static void flush_cb(lv_display_t *display_handle, const lv_area_t *area, uint8_t *px_map)
{
    const uint32_t width = (uint32_t)(area->x2 - area->x1 + 1);
    const uint32_t height = (uint32_t)(area->y2 - area->y1 + 1);
    /* CO5300 QSPI expects each RGB565 pixel in big-endian byte order.
     * Match LilyGO's Ultra LVGL helper before sending the draw buffer. */
    lv_draw_sw_rgb565_swap(px_map, width * height);
    if (!twatch_brookesia_bridge_draw((uint32_t)area->x1, (uint32_t)area->y1,
                                      (uint32_t)(area->x2 + 1), (uint32_t)(area->y2 + 1), px_map)) {
        ESP_LOGE(TAG, "Brookesia panel flush failed");
    }
    lv_display_flush_ready(display_handle);
}

static void touch_read_cb(lv_indev_t *input, lv_indev_data_t *data)
{
    (void)input;
    twatch_touch_sample_t sample;
    if (twatch_brookesia_bridge_read_touch(&sample) && sample.pressed) {
        data->point.x = sample.x;
        data->point.y = sample.y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void button_event_cb(lv_event_t *event)
{
    lv_obj_t *label = lv_event_get_user_data(event);
    static unsigned count;
    char text[32];
    lv_snprintf(text, sizeof(text), "Touches: %u", ++count);
    lv_label_set_text(label, text);
}

static void menu_toggle_cb(lv_event_t *event)
{
    (void)event;
    if (menu_screen != NULL) lv_screen_load(menu_screen);
}

static void menu_back_cb(lv_event_t *event)
{
    (void)event;
    if (home_screen != NULL) lv_screen_load(home_screen);
}

static void diagnostics_open_cb(lv_event_t *event)
{
    (void)event;
    if (diagnostics_screen != NULL) lv_screen_load(diagnostics_screen);
}

static void diagnostics_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (diagnostics_label == NULL || lv_screen_active() != diagnostics_screen) return;
    twatch_power_snapshot_t power = {0};
    twatch_rtc_snapshot_t rtc = {0};
    if (twatch_board_read_power(&power) != ESP_OK || twatch_board_read_rtc(&rtc) != ESP_OK) return;
    char text[160];
    lv_snprintf(text, sizeof(text), "Battery: %umV\nUSB: %s  Charge: %s\nRTC: %02x:%02x:%02x UTC",
                power.battery_mv, power.usb_present ? "yes" : "no",
                power.charging ? "yes" : "no", rtc.hours, rtc.minutes, rtc.seconds);
    lv_label_set_text(diagnostics_label, text);
}

static void menu_display_cb(lv_event_t *event)
{
    (void)event;
    display_inverted = !display_inverted;
    if (twatch_display_set_inverted(display_inverted) == ESP_OK) {
        lv_label_set_text(menu_text, display_inverted ?
                          "Display: inverted\nTouch: polling\nPower: read-only" :
                          "Display: normal\nTouch: polling\nPower: read-only");
    }
}

static uint8_t bcd_to_binary(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10) + (value & 0x0f));
}

static void rtc_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    twatch_rtc_snapshot_t rtc;
    if (twatch_board_read_rtc(&rtc) != ESP_OK) {
        char text[24];
        lv_snprintf(text, sizeof(text), "UTC --:--:--");
        lv_label_set_text(utc_label, text);
        return;
    }
    char text[24];
    lv_snprintf(text, sizeof(text), "UTC %02u:%02u:%02u",
                bcd_to_binary(rtc.hours & 0x3f), bcd_to_binary(rtc.minutes & 0x7f),
                bcd_to_binary(rtc.seconds & 0x7f));
    lv_label_set_text(utc_label, text);
}

esp_err_t lvgl_app_init(void)
{
    lv_init();
    display = lv_display_create(410, 502);
    if (display == NULL) return ESP_ERR_NO_MEM;
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(display, draw_buffer, NULL, sizeof(draw_buffer),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_add_event_cb(display, rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    lv_display_set_flush_cb(display, flush_cb);
    touch_input = lv_indev_create();
    if (touch_input == NULL) return ESP_ERR_NO_MEM;
    lv_indev_set_type(touch_input, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(touch_input, display);
    lv_indev_set_read_cb(touch_input, touch_read_cb);
    home_screen = lv_screen_active();
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "ESP-Brookesia\nT-Watch Ultra");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 86);
    utc_label = lv_label_create(lv_screen_active());
    lv_label_set_text(utc_label, "UTC --:--:--");
    lv_obj_set_style_text_font(utc_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(utc_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(utc_label, lv_color_hex(0xFF0000), 0);
    lv_obj_set_width(utc_label, 300);
    lv_obj_set_height(utc_label, 52);
    lv_obj_align(utc_label, LV_ALIGN_TOP_MID, 0, 190);
    lv_timer_create(rtc_timer_cb, 1000, NULL);
    rtc_timer_cb(NULL);
    lv_obj_t *touch_count = lv_label_create(lv_screen_active());
    lv_label_set_text(touch_count, "Touches: 0");
    lv_obj_set_style_text_font(touch_count, &lv_font_montserrat_22, 0);
    lv_obj_align(touch_count, LV_ALIGN_TOP_MID, 0, 270);
    lv_obj_t *button = lv_button_create(lv_screen_active());
    lv_obj_set_size(button, 180, 72);
    lv_obj_align(button, LV_ALIGN_TOP_MID, 0, 340);
    lv_obj_t *button_label = lv_label_create(button);
    lv_label_set_text(button_label, "Touch me");
    lv_obj_set_style_text_font(button_label, &lv_font_montserrat_24, 0);
    lv_obj_center(button_label);
    lv_obj_add_event_cb(button, button_event_cb, LV_EVENT_CLICKED, touch_count);
    lv_obj_t *menu_button = lv_button_create(lv_screen_active());
    lv_obj_set_size(menu_button, 140, 50);
    lv_obj_align(menu_button, LV_ALIGN_TOP_MID, 0, 425);
    lv_obj_t *menu_label = lv_label_create(menu_button);
    lv_label_set_text(menu_label, "Menu");
    lv_obj_set_style_text_font(menu_label, &lv_font_montserrat_22, 0);
    lv_obj_center(menu_label);
    lv_obj_add_event_cb(menu_button, menu_toggle_cb, LV_EVENT_CLICKED, NULL);
    menu_panel = lv_obj_create(lv_screen_active());
    lv_obj_set_size(menu_panel, 300, 110);
    lv_obj_align(menu_panel, LV_ALIGN_TOP_MID, 0, 285);
    menu_text = lv_label_create(menu_panel);
    lv_label_set_text(menu_text, "Display: inverted\nTouch: polling\nPower: read-only");
    lv_obj_set_style_text_font(menu_text, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_align(menu_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(menu_text);
    lv_obj_add_flag(menu_text, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(menu_text, menu_display_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(menu_panel, LV_OBJ_FLAG_HIDDEN);
    menu_screen = lv_obj_create(NULL);
    lv_obj_t *menu_title = lv_label_create(menu_screen);
    lv_label_set_text(menu_title, "Brookesia Menu");
    lv_obj_set_style_text_font(menu_title, &lv_font_montserrat_28, 0);
    lv_obj_align(menu_title, LV_ALIGN_TOP_MID, 0, 55);
    lv_obj_t *display_button = lv_button_create(menu_screen);
    lv_obj_set_size(display_button, 260, 78);
    lv_obj_align(display_button, LV_ALIGN_TOP_MID, 0, 145);
    lv_obj_t *display_label = lv_label_create(display_button);
    lv_label_set_text(display_label, "Toggle Display");
    lv_obj_set_style_text_font(display_label, &lv_font_montserrat_24, 0);
    lv_obj_center(display_label);
    lv_obj_add_event_cb(display_button, menu_display_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *diagnostics_button = lv_button_create(menu_screen);
    lv_obj_set_size(diagnostics_button, 260, 78);
    lv_obj_align(diagnostics_button, LV_ALIGN_TOP_MID, 0, 235);
    lv_obj_t *diagnostics_label = lv_label_create(diagnostics_button);
    lv_label_set_text(diagnostics_label, "Diagnostics");
    lv_obj_set_style_text_font(diagnostics_label, &lv_font_montserrat_24, 0);
    lv_obj_center(diagnostics_label);
    lv_obj_add_event_cb(diagnostics_button, diagnostics_open_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_button = lv_button_create(menu_screen);
    lv_obj_set_size(back_button, 260, 78);
    lv_obj_align(back_button, LV_ALIGN_TOP_MID, 0, 335);
    lv_obj_t *back_label = lv_label_create(back_button);
    lv_label_set_text(back_label, "Back");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_24, 0);
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_button, menu_back_cb, LV_EVENT_CLICKED, NULL);
    diagnostics_screen = lv_obj_create(NULL);
    lv_obj_t *diag_title = lv_label_create(diagnostics_screen);
    lv_label_set_text(diag_title, "Diagnostics");
    lv_obj_set_style_text_font(diag_title, &lv_font_montserrat_28, 0);
    lv_obj_align(diag_title, LV_ALIGN_TOP_MID, 0, 55);
    twatch_power_snapshot_t power = {0};
    twatch_board_read_power(&power);
    twatch_rtc_snapshot_t rtc = {0};
    twatch_board_read_rtc(&rtc);
    char diag_text[160];
    lv_snprintf(diag_text, sizeof(diag_text), "Battery: %umV\nUSB: %s  Charge: %s\nRTC: %02x:%02x:%02x UTC",
                power.battery_mv, power.usb_present ? "yes" : "no",
                power.charging ? "yes" : "no", rtc.hours, rtc.minutes, rtc.seconds);
    diagnostics_label = lv_label_create(diagnostics_screen);
    lv_label_set_text(diagnostics_label, diag_text);
    lv_obj_set_style_text_font(diagnostics_label, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_align(diagnostics_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(diagnostics_label, LV_ALIGN_TOP_MID, 0, 155);
    lv_timer_create(diagnostics_timer_cb, 1000, NULL);
    lv_obj_t *diag_back = lv_button_create(diagnostics_screen);
    lv_obj_set_size(diag_back, 260, 78);
    lv_obj_align(diag_back, LV_ALIGN_TOP_MID, 0, 335);
    lv_obj_t *diag_back_label = lv_label_create(diag_back);
    lv_label_set_text(diag_back_label, "Back");
    lv_obj_set_style_text_font(diag_back_label, &lv_font_montserrat_24, 0);
    lv_obj_center(diag_back_label);
    lv_obj_add_event_cb(diag_back, menu_back_cb, LV_EVENT_CLICKED, NULL);
    ESP_LOGI(TAG, "LVGL ready display=410x502 buffer_lines=16");
    return ESP_OK;
}

void lvgl_app_tick(void)
{
    static int64_t last_us;
    const int64_t now_us = esp_timer_get_time();
    if (last_us == 0) last_us = now_us;
    const int64_t elapsed_us = now_us - last_us;
    if (elapsed_us > 0) {
        lv_tick_inc((uint32_t)(elapsed_us / 1000));
        last_us += (elapsed_us / 1000) * 1000;
    }
    lv_timer_handler();
}
