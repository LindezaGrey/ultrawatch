/*
 * max98357a.c - Analog MAX98357A Class-D amp (I2S PCM output).
 *
 * Drives the amp over I2S0 TX in standard mode (mono, 16-bit). The channel
 * handle comes from twatch_board, which also owns the shared I2S0 RX channel
 * for the PDM microphone.
 */
#include "max98357a.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <stdint.h>
#include <stddef.h>

static const char *TAG = "max98357a";

static i2s_chan_handle_t s_tx;
static atomic_bool s_tx_enabled = ATOMIC_VAR_INIT(false);

esp_err_t max98357a_init(i2s_chan_handle_t tx_handle)
{
    if (!tx_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_tx && s_tx != tx_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (atomic_load(&s_tx_enabled)) {
        return ESP_OK;   /* already configured and running */
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        /* Mono: the MAX98357A samples the single left-aligned slot. */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MAX98357A_PIN_BCLK,
            .ws   = MAX98357A_PIN_WCLK,
            .dout = MAX98357A_PIN_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    esp_err_t err = ESP_OK;
    if (!s_tx) {
        err = i2s_channel_init_std_mode(tx_handle, &std_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "std mode init failed: %s", esp_err_to_name(err));
            return err;
        }
        s_tx = tx_handle;
    }
    err = i2s_channel_enable(tx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel enable failed: %s", esp_err_to_name(err));
        return err;
    }
    atomic_store(&s_tx_enabled, true);
    ESP_LOGI(TAG, "MAX98357A ready (%u Hz, mono 16-bit)", AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t max98357a_write(const int16_t *data, size_t n)
{
    if (!data || n == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_tx || !atomic_load(&s_tx_enabled)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (n > SIZE_MAX / sizeof(*data)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (AUDIO_SAMPLE_RATE == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t requested_bytes = n * sizeof(*data);
    /* Cap timeout math before multiplication, then use 10x stream duration
     * plus one second of scheduling/driver headroom. */
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

    size_t total_written = 0;
    int64_t deadline_us = esp_timer_get_time() + (int64_t)(timeout_ms64 * 1000);
    while (total_written < requested_bytes) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            ESP_LOGW(TAG, "write timed out: %u/%u bytes",
                     (unsigned)total_written, (unsigned)requested_bytes);
            return ESP_ERR_TIMEOUT;
        }
        uint32_t remaining_ms = (uint32_t)((remaining_us + 999) / 1000);
        if (remaining_ms == 0) {
            remaining_ms = 1;
        }
        size_t written_now = 0;
        esp_err_t err = i2s_channel_write(s_tx, (const uint8_t *)data + total_written,
                                          requested_bytes - total_written,
                                          &written_now, remaining_ms);
        if (written_now > requested_bytes - total_written ||
            written_now % sizeof(*data) != 0) {
            ESP_LOGE(TAG, "I2S returned an invalid byte count: %u",
                     (unsigned)written_now);
            return ESP_ERR_INVALID_SIZE;
        }
        total_written += written_now;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "write failed after %u/%u bytes: %s",
                     (unsigned)total_written, (unsigned)requested_bytes,
                     esp_err_to_name(err));
            return err;
        }
        if (written_now == 0) {
            TickType_t delay = pdMS_TO_TICKS(1);
            vTaskDelay(delay ? delay : 1);
        }
    }
    return ESP_OK;
}

esp_err_t max98357a_set_volume(float db)
{
    (void)db;
    /* The MAX98357A gain is fixed by a resistor (default ~20 dB); there is no
     * software volume register. Volume control is done by scaling the samples
     * on the host side. */
    return ESP_OK;
}
