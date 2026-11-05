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
#include "freertos/semphr.h"

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
static SemaphoreHandle_t s_output_mutex;
static portMUX_TYPE s_output_init_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_output_users;

void haptic_init(void)
{
    if (!s_output_mutex) {
        SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
        if (!mutex) {
            ESP_LOGE(TAG, "failed to create output state mutex");
            return;
        }
        portENTER_CRITICAL(&s_output_init_mux);
        if (!s_output_mutex) {
            s_output_mutex = mutex;
            mutex = NULL;
        }
        portEXIT_CRITICAL(&s_output_init_mux);
        if (mutex) vSemaphoreDelete(mutex);
    }
    SemaphoreHandle_t mutex = s_output_mutex;
    xSemaphoreTake(mutex, portMAX_DELAY);
    nvs_handle_t h;
    if (nvs_open(HAPTIC_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;
        if (nvs_get_u8(h, "pattern", &v) == ESP_OK && v < HAPTIC_PATTERN_COUNT) {
            s_pattern_idx = v;
        }
        nvs_close(h);
    }
    xSemaphoreGive(mutex);
}

size_t haptic_get_pattern_index(void)
{
    SemaphoreHandle_t mutex = s_output_mutex;
    if (!mutex) return 0;
    xSemaphoreTake(mutex, portMAX_DELAY);
    size_t idx = s_pattern_idx;
    xSemaphoreGive(mutex);
    return idx;
}

void haptic_set_pattern_index(size_t idx)
{
    if (idx >= HAPTIC_PATTERN_COUNT) {
        return;
    }
    SemaphoreHandle_t mutex = s_output_mutex;
    if (!mutex) {
        ESP_LOGE(TAG, "pattern selection before haptic_init");
        return;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (idx == s_pattern_idx) {
        xSemaphoreGive(mutex);
        return;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(HAPTIC_NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, "pattern", (uint8_t)idx);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err == ESP_OK) {
        s_pattern_idx = idx;
    } else {
        ESP_LOGW(TAG, "failed to persist pattern: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(mutex);
}

uint8_t haptic_get_wave_id(void)
{
    return HAPTIC_PATTERNS[haptic_get_pattern_index()].wave_id;
}

esp_err_t haptic_output_start(void)
{
    SemaphoreHandle_t mutex = s_output_mutex;
    if (!mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(mutex, portMAX_DELAY);
    esp_err_t err = twatch_board_av_rail_acquire();
    if (err != ESP_OK) {
        xSemaphoreGive(mutex);
        return err;
    }

    if (s_output_users == 0) {
        err = xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            twatch_board_av_rail_release();
            xSemaphoreGive(mutex);
            return err;
        }
    }
    s_output_users++;
    xSemaphoreGive(mutex);
    return ESP_OK;
}

esp_err_t haptic_output_stop(void)
{
    SemaphoreHandle_t mutex = s_output_mutex;
    if (!mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (s_output_users == 0) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ESP_OK;
    if (s_output_users == 1) {
        err = xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
    }
    if (err == ESP_OK) {
        s_output_users--;
        err = twatch_board_av_rail_release();
        if (err != ESP_OK) s_output_users++;
    }
    xSemaphoreGive(mutex);
    return err;
}

esp_err_t haptic_calibrate(void)
{
    esp_err_t err = haptic_output_start();
    if (err != ESP_OK) return err;
    err = drv2605_auto_calibrate(twatch_haptic_dev);
    esp_err_t stop = haptic_output_stop();
    return err != ESP_OK ? err : stop;
}

esp_err_t haptic_play_wave(uint8_t wave_id)
{
    esp_err_t err = haptic_output_start();
    if (err != ESP_OK) return err;
    err = drv2605_play(twatch_haptic_dev, wave_id);
    if (err == ESP_OK) vTaskDelay(pdMS_TO_TICKS(400));
    esp_err_t stop = haptic_output_stop();
    return err != ESP_OK ? err : stop;
}

void haptic_play_test(uint8_t wave_id)
{
    esp_err_t err = haptic_play_wave(wave_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "haptic preview failed: %s", esp_err_to_name(err));
    }
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
