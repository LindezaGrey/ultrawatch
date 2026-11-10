/*
 * t3902.c - TDK T3902 PDM microphone.
 *
 * Captures audio over I2S0 RX in PDM mode; the ESP32-S3 hardware decimates
 * PDM to 16-bit PCM (I2S_PDM_DATA_FMT_PCM). The channel handle comes from
 * twatch_board, which also owns the shared I2S0 TX channel for the amp.
 */
#include "t3902.h"
#include "esp_log.h"
#include "driver/i2s_pdm.h"
#include "driver/i2s_common.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdatomic.h>

static const char *TAG = "t3902";

static i2s_chan_handle_t s_rx;
static atomic_bool s_rx_enabled = ATOMIC_VAR_INIT(false);

esp_err_t t3902_init(i2s_chan_handle_t rx_handle)
{
    if (!rx_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rx && s_rx != rx_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (atomic_load(&s_rx_enabled)) {
        return ESP_OK;   /* already configured and running */
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = T3902_PIN_CLK,
            .din = T3902_PIN_DAT,
            .invert_flags = { .clk_inv = false },
        },
    };

    esp_err_t err = ESP_OK;
    if (!s_rx) {
        err = i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pdm rx init failed: %s", esp_err_to_name(err));
            return err;
        }
        s_rx = rx_handle;
    }
    err = i2s_channel_enable(rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel enable failed: %s", esp_err_to_name(err));
        return err;
    }
    atomic_store(&s_rx_enabled, true);
    ESP_LOGI(TAG, "T3902 ready (%u Hz, PDM->PCM mono 16-bit)", AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t t3902_read(int16_t *samples, size_t n)
{
    if (!samples || n == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_rx || !atomic_load(&s_rx_enabled)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (n > SIZE_MAX / sizeof(*samples)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (AUDIO_SAMPLE_RATE == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t requested_bytes = n * sizeof(*samples);
    /* Bound timeout math before multiplying, then give capture 10x duration
     * plus one second for task scheduling and I2S DMA startup. */
    const uint64_t max_samples =
        (((uint64_t)UINT32_MAX - 1000) / 10 * AUDIO_SAMPLE_RATE) / 1000;
    if ((uint64_t)n > max_samples) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint64_t duration_ms = ((uint64_t)n * 1000 + AUDIO_SAMPLE_RATE - 1) / AUDIO_SAMPLE_RATE;
    uint64_t timeout_ms64 = duration_ms * 10 + 1000;
    if (timeout_ms64 > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t total_read = 0;
    int64_t deadline_us = esp_timer_get_time() + (int64_t)(timeout_ms64 * 1000);
    while (total_read < requested_bytes) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            ESP_LOGW(TAG, "capture timed out: %u/%u bytes",
                     (unsigned)total_read, (unsigned)requested_bytes);
            return ESP_ERR_TIMEOUT;
        }
        uint32_t remaining_ms = (uint32_t)((remaining_us + 999) / 1000);
        if (remaining_ms == 0) {
            remaining_ms = 1;
        }
        size_t read_now = 0;
        esp_err_t err = i2s_channel_read(s_rx, (uint8_t *)samples + total_read,
                                         requested_bytes - total_read,
                                         &read_now, remaining_ms);
        if (read_now > requested_bytes - total_read) {
            ESP_LOGE(TAG, "I2S returned an invalid byte count: %u",
                     (unsigned)read_now);
            return ESP_ERR_INVALID_SIZE;
        }
        total_read += read_now;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "capture failed after %u/%u bytes: %s",
                     (unsigned)total_read, (unsigned)requested_bytes,
                     esp_err_to_name(err));
            return err;
        }
        if (read_now == 0) {
            TickType_t delay = pdMS_TO_TICKS(1);
            vTaskDelay(delay ? delay : 1);
        }
    }
    return ESP_OK;
}
