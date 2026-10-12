/*
 * controllers/haptic.c - see haptic.h.
 */
#include "haptic.h"
#include "twatch_board.h"
#include "xl9555.h"
#include "drv2605.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HAPTIC_NVS_NS  "haptic"
static const char *TAG = "haptic";

/* Effect ids from the DRV2605 datasheet's Waveform Library Effects List
 * (section 11.2) - library 1 (ERM, see drv2605_init()). 47 ("Buzz 1 - 100%")
 * is the historical hardcoded default every call site used before this
 * picker existed; kept as an option, not the default, once a plain click
 * became selectable too. */
const haptic_pattern_t HAPTIC_PATTERNS[HAPTIC_PATTERN_COUNT] = {
    { "Strong Click", 1 },
    { "Sharp Click",  4 },
    { "Double Click", 10 },
    { "Triple Click", 12 },
    { "Soft Bump",    7 },
    { "Buzz",         47 },
};

static size_t s_pattern_idx;

void haptic_init(void)
{
    nvs_handle_t h;
    if (nvs_open(HAPTIC_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;
        if (nvs_get_u8(h, "pattern", &v) == ESP_OK && v < HAPTIC_PATTERN_COUNT) {
            s_pattern_idx = v;
        }
        nvs_close(h);
    }
}

size_t haptic_get_pattern_index(void)
{
    return s_pattern_idx;
}

void haptic_set_pattern_index(size_t idx)
{
    if (idx >= HAPTIC_PATTERN_COUNT || idx == s_pattern_idx) {
        return;
    }
    s_pattern_idx = idx;
    nvs_handle_t h;
    if (nvs_open(HAPTIC_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "pattern", (uint8_t)idx);
        nvs_commit(h);
        nvs_close(h);
    }
}

uint8_t haptic_get_wave_id(void)
{
    return HAPTIC_PATTERNS[s_pattern_idx].wave_id;
}

void haptic_play_test(uint8_t wave_id)
{
    xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
    vTaskDelay(pdMS_TO_TICKS(20));
    drv2605_play(twatch_haptic_dev, wave_id);
    vTaskDelay(pdMS_TO_TICKS(400));
    xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
}

static portMUX_TYPE s_test_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_test_running;

static void haptic_play_test_task(void *arg)
{
    haptic_play_test((uint8_t)(uintptr_t)arg);
    portENTER_CRITICAL(&s_test_mux);
    s_test_running = false;
    portEXIT_CRITICAL(&s_test_mux);
    vTaskDelete(NULL);
}

/* One preview at a time. Every run drives the shared HAPTIC_EN rail through
 * the XL9555, so overlapping runs cut each other off mid-effect and each one
 * costs another 2 kB of task stack - tapping through the picker quickly used to
 * start a task per tap. */
void haptic_play_test_async(uint8_t wave_id)
{
    bool start = false;
    portENTER_CRITICAL(&s_test_mux);
    if (!s_test_running) {
        s_test_running = true;
        start = true;
    }
    portEXIT_CRITICAL(&s_test_mux);
    if (!start) {
        ESP_LOGD(TAG, "preview running, ignoring wave %u", (unsigned)wave_id);
        return;
    }
    if (xTaskCreate(haptic_play_test_task, "haptic_test", 2048,
                    (void *)(uintptr_t)wave_id, 3, NULL) != pdPASS) {
        portENTER_CRITICAL(&s_test_mux);
        s_test_running = false;
        portEXIT_CRITICAL(&s_test_mux);
        ESP_LOGE(TAG, "test task allocation failed");
    }
}
