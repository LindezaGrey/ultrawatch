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
static lv_obj_t *home_screen;
static lv_obj_t *menu_screen;

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

static void open_menu_cb(lv_event_t *event)
{
    (void)event;
    lv_screen_load(menu_screen);
}

static void back_home_cb(lv_event_t *event)
{
    (void)event;
    lv_screen_load(home_screen);
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
    home_screen = lv_obj_create(NULL);
    lv_obj_remove_style_all(home_screen);
    lv_obj_t *label = lv_label_create(home_screen);
    lv_label_set_text(label, "ESP-Brookesia\nT-Watch Ultra");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    lv_obj_t *touch_count = lv_label_create(home_screen);
    lv_label_set_text(touch_count, "Touches: 0");
    lv_obj_set_style_text_font(touch_count, &lv_font_montserrat_22, 0);
    lv_obj_align(touch_count, LV_ALIGN_CENTER, 0, 60);
    lv_obj_t *button = lv_button_create(home_screen);
    lv_obj_set_size(button, 180, 72);
    lv_obj_align(button, LV_ALIGN_CENTER, 0, 150);
    lv_obj_t *button_label = lv_label_create(button);
    lv_label_set_text(button_label, "Touch me");
    lv_obj_set_style_text_font(button_label, &lv_font_montserrat_24, 0);
    lv_obj_center(button_label);
    lv_obj_add_event_cb(button, button_event_cb, LV_EVENT_CLICKED, touch_count);
    lv_obj_t *menu_button = lv_button_create(home_screen);
    lv_obj_set_size(menu_button, 180, 60);
    lv_obj_align(menu_button, LV_ALIGN_CENTER, 0, 220);
    lv_obj_t *menu_label = lv_label_create(menu_button);
    lv_label_set_text(menu_label, "Menu");
    lv_obj_set_style_text_font(menu_label, &lv_font_montserrat_22, 0);
    lv_obj_center(menu_label);
    lv_obj_add_event_cb(menu_button, open_menu_cb, LV_EVENT_CLICKED, NULL);

    menu_screen = lv_obj_create(NULL);
    lv_obj_remove_style_all(menu_screen);
    lv_obj_t *menu_title = lv_label_create(menu_screen);
    lv_label_set_text(menu_title, "Brookesia Menu");
    lv_obj_set_style_text_font(menu_title, &lv_font_montserrat_28, 0);
    lv_obj_center(menu_title);
    lv_obj_t *back_button = lv_button_create(menu_screen);
    lv_obj_set_size(back_button, 180, 72);
    lv_obj_align(back_button, LV_ALIGN_CENTER, 0, 150);
    lv_obj_t *back_label = lv_label_create(back_button);
    lv_label_set_text(back_label, "Back");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_24, 0);
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_button, back_home_cb, LV_EVENT_CLICKED, NULL);
    lv_screen_load(home_screen);
    ESP_LOGI(TAG, "LVGL ready display=410x502 buffer_lines=16 shell=home-menu");
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
