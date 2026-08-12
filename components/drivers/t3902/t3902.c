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

static const char *TAG = "t3902";

static i2s_chan_handle_t s_rx;

esp_err_t t3902_init(i2s_chan_handle_t rx_handle)
{
    if (!rx_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rx) {
        return ESP_OK;   /* already configured */
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

    esp_err_t err = i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pdm rx init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2s_channel_enable(rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel enable failed: %s", esp_err_to_name(err));
        return err;
    }
    s_rx = rx_handle;
    ESP_LOGI(TAG, "T3902 ready (%u Hz, PDM->PCM mono 16-bit)", AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t t3902_read(int16_t *samples, size_t n)
{
    if (!s_rx || !samples) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Timeout scales with the requested length: n samples at AUDIO_SAMPLE_RATE
     * take n/rate seconds. Give generous headroom (10x + 1 s). */
    size_t read = 0;
    uint32_t timeout_ms = (uint32_t)(n * 1000 / AUDIO_SAMPLE_RATE) * 10 + 1000;
    esp_err_t err = i2s_channel_read(s_rx, samples, n * sizeof(int16_t), &read, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (read != n * sizeof(int16_t)) {
        ESP_LOGW(TAG, "short read: %u/%u bytes", (unsigned)read, (unsigned)(n * 2));
    }
    return ESP_OK;
}
