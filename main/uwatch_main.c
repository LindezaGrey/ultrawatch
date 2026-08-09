/*
 * uwatch_main.c - UWatch application entry point.
 *
 * FreeRTOS (IDF) app_main: initializes the T-Watch Ultra board package,
 * then renders a simple analog watch face on the CO5300 AMOLED and animates
 * a second hand. Debug via JTAG/OpenOCD/GDB.
 *
 * Rendering notes: the CO5300 (SH8601) panel requires even x/y draw
 * boundaries, so all shapes are rounded to even coordinates and drawn in
 * even-sized bands.
 */
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "twatch_board.h"
#include "co5300.h"

static const char *TAG = "uwatch";

#define FACE_CX    (CO5300_RES_X / 2)
#define FACE_CY    (CO5300_RES_Y / 2)
#define FACE_R     200

/* Fill an even-rounded rectangle, drawn in 48-row even bands. */
static void fb_rect(int x0, int y0, int x1, int y1, uint16_t color)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= CO5300_RES_X) x1 = CO5300_RES_X - 1;
    if (y1 >= CO5300_RES_Y) y1 = CO5300_RES_Y - 1;
    int sx = x0 & ~1;                 /* even start */
    int sy = y0 & ~1;
    int ex = ((x1 + 1) | 1) + 1;      /* even exclusive end */
    int ey = ((y1 + 1) | 1) + 1;
    if (ex > CO5300_RES_X) ex = CO5300_RES_X;
    if (ey > CO5300_RES_Y) ey = CO5300_RES_Y;
    int w = ex - sx;
    if (w <= 0 || ey <= sy) return;

    static uint16_t band[CO5300_RES_X * 48];
    for (int i = 0; i < w * 48; i++) {
        band[i] = color;
    }
    for (int y = sy; y < ey; y += 48) {
        int h = (ey - y) < 48 ? (ey - y) : 48;
        co5300_draw_bitmap(sx, y, sx + w - 1, y + h - 1, band);
    }
}

/* Filled circle drawn as 2-row even bands (union span of the two rows). */
static void fb_fill_circle(int cx, int cy, int r, uint16_t color)
{
    static uint16_t row[CO5300_RES_X * 2];
    int y_top = cy - r, y_bot = cy + r;
    if (y_top < 0) y_top = 0;
    if (y_bot >= CO5300_RES_Y) y_bot = CO5300_RES_Y - 1;
    for (int y = y_top & ~1; y <= y_bot; y += 2) {
        int xmin = CO5300_RES_X, xmax = -1;
        for (int yy = y; yy <= y + 1; yy++) {
            if (yy < y_top || yy > y_bot) continue;
            int dy = yy - cy;
            int dx = (int)sqrtf((float)(r * r - dy * dy));
            int x0 = cx - dx, x1 = cx + dx;
            if (x0 < xmin) xmin = x0;
            if (x1 > xmax) xmax = x1;
        }
        if (xmax < xmin) continue;
        if (xmin < 0) xmin = 0;
        if (xmax >= CO5300_RES_X) xmax = CO5300_RES_X - 1;
        int sx = xmin & ~1;
        int ex = ((xmax + 1) | 1) + 1;
        if (ex > CO5300_RES_X) ex = CO5300_RES_X;
        int w = ex - sx;
        for (int i = 0; i < w * 2; i++) {
            row[i] = color;
        }
        co5300_draw_bitmap(sx, y, sx + w - 1, y + 1, row);
    }
}

static void fb_line(int x0, int y0, int x1, int y1, int thick, uint16_t color)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int half = thick / 2;
    for (;;) {
        fb_rect(x0 - half, y0 - half, x0 + half, y0 + half, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void fb_tick(int cx, int cy, int angle_deg, int r0, int r1, int thick, uint16_t color)
{
    float a = angle_deg * M_PI / 180.0f;
    int x0 = cx + (int)(r0 * sinf(a));
    int y0 = cy - (int)(r0 * cosf(a));
    int x1 = cx + (int)(r1 * sinf(a));
    int y1 = cy - (int)(r1 * cosf(a));
    fb_line(x0, y0, x1, y1, thick, color);
}

static void render_face(void)
{
    co5300_fill(CO5300_BLACK);
    fb_fill_circle(FACE_CX, FACE_CY, FACE_R, CO5300_WHITE);

    for (int i = 0; i < 12; i++) {
        fb_tick(FACE_CX, FACE_CY, i * 30, 178, 196, 2, CO5300_BLACK);
    }

    /* Fixed hands (10:09:30). */
    fb_tick(FACE_CX, FACE_CY, 10 * 30 + 15, 0, 95, 6, CO5300_BLACK);   /* hour */
    fb_tick(FACE_CX, FACE_CY, 9 * 6 + 30, 0, 155, 4, CO5300_RED);      /* minute */

    fb_fill_circle(FACE_CX, FACE_CY, 8, CO5300_RED);
    ESP_LOGI(TAG, "face rendered");
}

static void render_second_hand(int sec, uint16_t color)
{
    float a = (sec % 60) * 6.0f * M_PI / 180.0f;
    int x = FACE_CX + (int)(168 * sinf(a));
    int y = FACE_CY - (int)(168 * cosf(a));
    fb_line(FACE_CX, FACE_CY, x, y, 2, color);
}

static void uwatch_main_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "UWatch main task started");

    render_face();

    int sec = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        render_second_hand(sec, CO5300_WHITE);   /* erase previous */
        sec = (sec + 1) % 60;
        render_second_hand(sec, CO5300_RED);     /* draw new */
    }
}

void app_main(void)
{
    esp_err_t err = twatch_board_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "board init reported error 0x%x (%s), continuing", err, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "UWatch boot complete");

    xTaskCreatePinnedToCore(uwatch_main_task, "uwatch", 8192, NULL, 5, NULL, tskNO_AFFINITY);
}
