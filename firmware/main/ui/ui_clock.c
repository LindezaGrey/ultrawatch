#include "ui_clock.h"

#include "lvgl.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "bsp_twatch_ultra.h"
#include "bsp_display.h"
#include "bsp_cst9217.h"
#include "bsp_axp2101.h"
#include "bsp_pcf85063.h"
#include "screenshot.h"
#include "power.h"
#include "powerlog.h"

static const char *TAG = "ui_clock";

#define UI_LONG_PRESS_MS    1000

#define UI_REFRESH_MS         1000

/* Touch -> display mapping (tunable during bring-up) */
#define UI_TOUCH_SWAP_XY      0
#define UI_TOUCH_MIRROR_X     0
#define UI_TOUCH_MIRROR_Y     0

#define UI_DISPLAY_SWAP_RGB565 1

/* Generated 96px Montserrat-Bold font (digits '0'-'9' + ':') for the G-Shock face */
extern lv_font_t lv_font_montserrat_96;

typedef enum {
    UI_MODE_CASIO_DARK,
    UI_MODE_RED_LOWPOWER,
    UI_MODE_COUNT,
} ui_mode_t;

static ui_mode_t s_mode = UI_MODE_CASIO_DARK;

/* Console-triggered mode requests are deferred to the LVGL tick (ui_timer_cb):
 * the REPL runs in its own task and must not call LVGL APIs while the render
 * loop in app_main is active (LVGL is not thread-safe). -1 = none, -2 = cycle,
 * >=0 = explicit mode index. */
static volatile int32_t s_mode_request = -1;

static lv_obj_t *s_time_label;
static lv_obj_t *s_weekday_label;
static lv_obj_t *s_date_label;
static lv_obj_t *s_battery_label;
static lv_obj_t *s_battery_bar;
static lv_obj_t *s_battery_bar_fill;

static uint8_t *s_disp_buf1;
static uint8_t *s_disp_buf2;
static uint32_t s_disp_buf_size;
static bool s_capturing;

static bool s_shot_requested;

/* Last rendered time fields; -1 forces the first render */
static int s_last_min = -1;
static int s_last_hour = -1;

static const char *s_weekdays[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };

/* TE (tearing-effect) sync for the CO5300. The panel pulses the TE pin once per
 * frame (0x35, v-blanking). We align the start of each LVGL frame flush to the
 * next TE edge so partial-band writes never land mid-panel-scan (which causes
 * diagonal tearing). A timeout keeps it safe if TE is unavailable. */
#define TE_WAIT_TIMEOUT_US      (50 * 1000)
static uint32_t s_te_calls, s_te_timeouts, s_te_max_us, s_te_last_us;
static int s_te_last_level;

static void wait_for_te_edge(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BSP_DISP_TE_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    int level = gpio_get_level(BSP_DISP_TE_PIN);
    uint32_t t0 = (uint32_t)esp_timer_get_time();
    while (gpio_get_level(BSP_DISP_TE_PIN) == level) {
        if ((uint32_t)esp_timer_get_time() - t0 > TE_WAIT_TIMEOUT_US) {
            break;
        }
    }
    uint32_t dt = (uint32_t)esp_timer_get_time() - t0;
    s_te_calls++;
    s_te_last_us = dt;
    s_te_last_level = level;
    if (dt > s_te_max_us) {
        s_te_max_us = dt;
    }
    if (dt >= TE_WAIT_TIMEOUT_US - 1000) {
        s_te_timeouts++;
    }
    if ((s_te_calls % 25) == 0) {
        ESP_LOGI(TAG, "TE wait: %lu frames, %lu timeouts, max %lu us, last %lu us, lvl %d",
                 (unsigned long)s_te_calls, (unsigned long)s_te_timeouts,
                 (unsigned long)s_te_max_us, (unsigned long)dt, s_te_last_level);
    }
}

void ui_get_te_stats(uint32_t *calls, uint32_t *timeouts, uint32_t *max_us, uint32_t *last_us)
{
    *calls = s_te_calls;
    *timeouts = s_te_timeouts;
    *max_us = s_te_max_us;
    *last_us = s_te_last_us;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel = bsp_display_get_panel();

    /* TE-sync EVERY band, not just the first of a frame. A full-screen redraw
     * spans several panel frames; bands that are not aligned to v-blanking land
     * mid-scan. With the 60MHz QSPI clock the band writes (~30MB/s) outrun the
     * panel scan (~26MB/s), so an aligned band stays ahead of the scan and never
     * collides with the rows it is currently reading (no diagonal scramble). */
    wait_for_te_edge();

#if UI_DISPLAY_SWAP_RGB565
    uint32_t n = lv_area_get_width(area) * lv_area_get_height(area);
    uint16_t *p = (uint16_t *)px_map;
    for (uint32_t i = 0; i < n; i++) {
        p[i] = (uint16_t)((p[i] >> 8) | (p[i] << 8));
    }
#endif

    bsp_display_xfer_begin();
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1, px_map);

    /* draw_bitmap only queues the SPI DMA transfer. The panel IO driver recycles
     * the color buffer only on on_color_trans_done; LVGL must not touch px_map
     * again (next render) until the DMA has read it, or the panel shows mixed
     * / diagonally-teared bands. Wait for the transfer, then release the buffer. */
    bsp_display_wait_flush_done();

    lv_display_flush_ready(disp);
}

static void capture_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    lv_display_flush_ready(disp);
}

static void ui_apply_mode(void)
{
    uint32_t fg, accent, bg;
    bool show_all;

    switch (s_mode) {
    case UI_MODE_CASIO_DARK:
        fg = 0xFFFFFF;
        accent = 0x00BFFF;
        bg = 0x000000;
        show_all = true;
        break;
    case UI_MODE_RED_LOWPOWER:
        fg = 0xFF0000;
        accent = 0xFF0000;
        bg = 0x000000;
        show_all = false;
        break;
    default:
        return;
    }

    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(bg), 0);
    lv_obj_set_style_text_color(s_time_label, lv_color_hex(fg), 0);
    lv_obj_set_style_text_color(s_weekday_label, lv_color_hex(fg), 0);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(fg), 0);
    lv_obj_set_style_text_color(s_battery_label, lv_color_hex(fg), 0);
    lv_obj_set_style_border_color(s_battery_bar, lv_color_hex(fg), 0);
    lv_obj_set_style_bg_color(s_battery_bar_fill, lv_color_hex(accent), 0);

    if (show_all) {
        lv_obj_remove_flag(s_weekday_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_date_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_battery_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_weekday_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_date_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_battery_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN);
    }

    /* Low-power red mode also dims the panel (fewer lit subpixels + lower pwm) */
    if (s_mode == UI_MODE_RED_LOWPOWER) {
        bsp_display_set_brightness(UI_RED_MODE_BRIGHTNESS_PCT);
        power_set_user_brightness(UI_RED_MODE_BRIGHTNESS_PCT);
    } else {
        bsp_display_set_brightness(UI_DAY_MODE_BRIGHTNESS_PCT);
        power_set_user_brightness(UI_DAY_MODE_BRIGHTNESS_PCT);
    }
    ESP_LOGI(TAG, "mode=%d fg=%06X accent=%06X", (int)s_mode, fg, accent);
}

static void ui_capture_screenshot(void)
{
    if (s_capturing) {
        return;
    }
    s_capturing = true;
    ESP_LOGI(TAG, "capture: start");

    lv_display_t *disp = lv_display_get_default();
    if (disp == NULL) {
        s_capturing = false;
        return;
    }

    lv_draw_buf_t *cap = lv_draw_buf_create(BSP_LCD_H_RES, BSP_LCD_V_RES,
                                            LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO);
    if (cap == NULL) {
        ESP_LOGE(TAG, "capture buffer alloc failed");
        s_capturing = false;
        return;
    }
    ESP_LOGI(TAG, "capture: buffer %ux%u", cap->header.w, cap->header.h);

    lv_display_set_flush_cb(disp, capture_flush_cb);
    lv_display_set_buffers(disp, cap->data, NULL, cap->data_size,
                           LV_DISPLAY_RENDER_MODE_DIRECT);
    ESP_LOGI(TAG, "capture: buffers swapped, refr_now...");
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(disp);
    ESP_LOGI(TAG, "capture: refr_now done");

    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    lv_display_set_buffers(disp, s_disp_buf1, s_disp_buf2, s_disp_buf_size,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_obj_invalidate(lv_screen_active());
    ESP_LOGI(TAG, "capture: restored, stride=%u", cap->header.stride);

    screenshot_queue(cap);
    s_capturing = false;
    ESP_LOGI(TAG, "capture: queued");
}

void ui_shot_trigger(void)
{
    ESP_LOGI(TAG, "shot triggered from console");
    s_shot_requested = true;
}

void ui_mode_cycle(void)
{
    s_mode = (ui_mode_t)((s_mode + 1) % UI_MODE_COUNT);
    ui_apply_mode();
}

void ui_mode_select(int index)
{
    if (index < 0 || index >= (int)UI_MODE_COUNT) {
        ESP_LOGW(TAG, "mode index %d out of range", index);
        return;
    }
    s_mode = (ui_mode_t)index;
    ui_apply_mode();
}

/* Safe from any task context: only flags the request for the LVGL tick. */
void ui_mode_cycle_request(void)
{
    s_mode_request = -2;
}

void ui_mode_select_request(int index)
{
    if (index < 0 || index >= (int)UI_MODE_COUNT) {
        ESP_LOGW(TAG, "mode index %d out of range", index);
        return;
    }
    s_mode_request = index;
}

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    static bool last_pressed = false;
    static uint32_t press_start_ms = 0;
    uint16_t tx = 0, ty = 0;
    bool pressed = bsp_touch_get_point(&tx, &ty);

#if UI_TOUCH_SWAP_XY
    lv_coord_t x = ty;
    lv_coord_t y = tx;
#else
    lv_coord_t x = tx;
    lv_coord_t y = ty;
#endif
#if UI_TOUCH_MIRROR_X
    x = BSP_LCD_H_RES - 1 - x;
#endif
#if UI_TOUCH_MIRROR_Y
    y = BSP_LCD_V_RES - 1 - y;
#endif

    if (pressed && !last_pressed) {
        press_start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        ui_mode_cycle();
        power_set_interactive();
        ESP_LOGI(TAG, "touch at (%d,%d) mode=%d", (int)x, (int)y, (int)s_mode);
    } else if (!pressed && last_pressed) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (now_ms - press_start_ms >= UI_LONG_PRESS_MS) {
            ESP_LOGI(TAG, "long press -> screenshot");
            ui_capture_screenshot();
        }
    }
    last_pressed = pressed;

    data->point.x = x;
    data->point.y = y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

static uint32_t lvgl_tick_get_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void ui_update_battery(void)
{
    uint8_t percent = bsp_axp2101_battery_percent();
    char text[32];

    if (percent == 0xFF) {
        lv_label_set_text(s_battery_label, "BAT --");
        lv_obj_set_width(s_battery_bar_fill, 0);
        return;
    }
    uint16_t mv = bsp_axp2101_battery_voltage_mv();
    snprintf(text, sizeof(text), "BAT %d%% %.2fV", percent, mv / 1000.0f);
    lv_label_set_text(s_battery_label, text);

    lv_obj_set_width(s_battery_bar_fill, (lv_coord_t)(lv_pct(percent)));
}

static void ui_timer_cb(lv_timer_t *timer)
{
    char text[32];

    int32_t mode_req = s_mode_request;
    if (mode_req != -1) {
        s_mode_request = -1;
        if (mode_req == -2) {
            ui_mode_cycle();
        } else {
            ui_mode_select((int)mode_req);
        }
    }

    if (s_shot_requested) {
        s_shot_requested = false;
        ui_capture_screenshot();
    }

    struct tm tm;
    if (bsp_rtc_get_time(&tm) != ESP_OK) {
        return;
    }

    /* Power-saver face: only re-render when the minute (or the day) changes */
    if (tm.tm_min != s_last_min || tm.tm_hour != s_last_hour) {
        s_last_min = tm.tm_min;
        s_last_hour = tm.tm_hour;

        snprintf(text, sizeof(text), "%02d:%02d", tm.tm_hour, tm.tm_min);
        lv_label_set_text(s_time_label, text);

        snprintf(text, sizeof(text), "%s", s_weekdays[(tm.tm_wday + 7) % 7]);
        lv_label_set_text(s_weekday_label, text);

        snprintf(text, sizeof(text), "%02d-%02d", tm.tm_mon + 1, tm.tm_mday);
        lv_label_set_text(s_date_label, text);

        ui_update_battery();
    }

    powerlog_tick();
}

static void ui_calibration_cb(lv_timer_t *timer)
{
    lv_obj_delete((lv_obj_t *)lv_timer_get_user_data(timer));
    lv_timer_del(timer);
    ESP_LOGI(TAG, "calibration screen removed");
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_label_set_text(label, "");
    return label;
}

esp_err_t ui_clock_init(void)
{
    lv_init();
    lv_tick_set_cb(lvgl_tick_get_cb);

    lv_display_t *disp = lv_display_create(BSP_LCD_H_RES, BSP_LCD_V_RES);
    if (!disp) {
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    /* Partial double buffering in internal DMA RAM. At the 60MHz QSPI clock the
     * band writes (~30MB/s) outrun the panel scan (~26MB/s), so each TE-synced
     * band stays ahead of the scan and cannot tear. (Full-screen PSRAM buffers
     * were tried: the SPI driver needs a full-frame internal bounce buffer for
     * PSRAM sources, which does not fit, and direct PSRAM DMA underflows.) */
    uint32_t buf_size = BSP_LCD_H_RES * 60 * sizeof(uint16_t);
    uint8_t *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!buf1 || !buf2) {
        return ESP_ERR_NO_MEM;
    }
    s_disp_buf1 = buf1;
    s_disp_buf2 = buf2;
    s_disp_buf_size = buf_size;
    lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    /* --- Casio G-Shock layout (screen center is (205, 251)) --- */
    s_time_label = make_label(scr, &lv_font_montserrat_96, lv_color_hex(0xFFFFFF));
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, -6);        /* y ~245 */

    s_weekday_label = make_label(scr, &lv_font_montserrat_28, lv_color_hex(0xFFFFFF));
    lv_obj_align(s_weekday_label, LV_ALIGN_CENTER, 0, -181);   /* y ~70 */

    s_date_label = make_label(scr, &lv_font_montserrat_28, lv_color_hex(0xFFFFFF));
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, -131);      /* y ~120 */

    s_battery_label = make_label(scr, &lv_font_montserrat_14, lv_color_hex(0xFFFFFF));
    lv_obj_align(s_battery_label, LV_ALIGN_CENTER, 0, 159);    /* y ~410 */

    /* Battery bar */
    s_battery_bar = lv_obj_create(scr);
    lv_obj_remove_style_all(s_battery_bar);
    lv_obj_set_size(s_battery_bar, 96, 12);
    lv_obj_set_style_border_width(s_battery_bar, 2, 0);
    lv_obj_set_style_border_color(s_battery_bar, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_pad_all(s_battery_bar, 1, 0);
    lv_obj_align(s_battery_bar, LV_ALIGN_CENTER, 0, 189);      /* y ~440 */

    s_battery_bar_fill = lv_obj_create(s_battery_bar);
    lv_obj_remove_style_all(s_battery_bar_fill);
    lv_obj_set_size(s_battery_bar_fill, 0, 8);
    lv_obj_set_style_bg_color(s_battery_bar_fill, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_bg_opa(s_battery_bar_fill, LV_OPA_COVER, 0);
    lv_obj_align(s_battery_bar_fill, LV_ALIGN_TOP_LEFT, 0, 0);

    ui_update_battery();

    lv_timer_create(ui_timer_cb, UI_REFRESH_MS, NULL);

    ui_apply_mode();

    screenshot_init();

    /* --- Color calibration screen: 4 quadrants (RED/GREEN/BLUE/WHITE), auto-removed --- */
    lv_obj_t *cal_screen = lv_obj_create(scr);
    lv_obj_remove_style_all(cal_screen);
    lv_obj_set_size(cal_screen, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_pos(cal_screen, 0, 0);

    static const uint32_t qcolors[4] = { 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF };
    static const char *qnames[4] = { "RED", "GREEN", "BLUE", "WHITE" };
    for (int i = 0; i < 4; i++) {
        lv_obj_t *q = lv_obj_create(cal_screen);
        lv_obj_remove_style_all(q);
        lv_obj_set_style_bg_color(q, lv_color_hex(qcolors[i]), 0);
        lv_obj_set_style_bg_opa(q, LV_OPA_COVER, 0);
        lv_obj_set_size(q, BSP_LCD_H_RES / 2, BSP_LCD_V_RES / 2);
        lv_obj_set_pos(q, (i % 2) * (BSP_LCD_H_RES / 2), (i / 2) * (BSP_LCD_V_RES / 2));

        lv_obj_t *lbl = lv_label_create(q);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(i == 3 ? 0x000000 : 0xFFFFFF), 0);
        lv_label_set_text(lbl, qnames[i]);
        lv_obj_center(lbl);
    }

    lv_timer_t *cal_timer = lv_timer_create(ui_calibration_cb, 3500, NULL);
    lv_timer_set_user_data(cal_timer, cal_screen);

    ESP_LOGI(TAG, "ui clock ready");
    return ESP_OK;
}
