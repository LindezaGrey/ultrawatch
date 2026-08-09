/*
 * uwatch_main.c - UWatch application entry point.
 *
 * FreeRTOS (IDF) app_main: initializes the T-Watch Ultra board package,
 * then hands the display over to LVGL (esp_lvgl_adapter), which runs its
 * own FreeRTOS task. Debug via JTAG/OpenOCD/GDB.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "twatch_board.h"
#include "lvgl_app.h"

static const char *TAG = "uwatch";

static void uwatch_main_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "UWatch main task started");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_err_t err = twatch_board_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "board init reported error 0x%x (%s), continuing", err, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "UWatch boot complete");

    err = lvgl_app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl start failed: %s", esp_err_to_name(err));
    }

    xTaskCreatePinnedToCore(uwatch_main_task, "uwatch", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}
