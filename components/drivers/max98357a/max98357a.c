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

static const char *TAG = "max98357a";

static i2s_chan_handle_t s_tx;

esp_err_t max98357a_init(i2s_chan_handle_t tx_handle)
{
    if (!tx_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_tx) {
        return ESP_OK;   /* already configured */
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

    esp_err_t err = i2s_channel_init_std_mode(tx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "std mode init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2s_channel_enable(tx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel enable failed: %s", esp_err_to_name(err));
        return err;
    }
    s_tx = tx_handle;
    ESP_LOGI(TAG, "MAX98357A ready (%u Hz, mono 16-bit)", AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t max98357a_write(const int16_t *data, size_t n)
{
    if (!s_tx || !data) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Timeout scales with the data length: n samples at AUDIO_SAMPLE_RATE take
     * n/rate seconds. Give generous headroom (10x + 1 s) so long clips don't
     * spuriously time out. */
    size_t written = 0;
    uint32_t timeout_ms = (uint32_t)(n * 1000 / AUDIO_SAMPLE_RATE) * 10 + 1000;
    esp_err_t err = i2s_channel_write(s_tx, data, n * sizeof(int16_t), &written, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (written != n * sizeof(int16_t)) {
        ESP_LOGW(TAG, "short write: %u/%u bytes", (unsigned)written, (unsigned)(n * 2));
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
