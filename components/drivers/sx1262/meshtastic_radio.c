#include "meshtastic_radio.h"
#include "sx1262.h"

/* Meshtastic's default public "LongFast" preset, EU_868 region.
 *
 * SF/BW/CR and preamble length verified against Meshtastic's own firmware
 * source (github.com/meshtastic/firmware, src/mesh/RadioInterface.h:
 * "uint16_t preambleLength = 16; // 8 is default, but we use longer to
 * increase the amount of sleep time when receiving"). Sync word 0x2B and
 * the SF11/BW250/CR4:5 preset table cross-checked against Meshtastic's
 * public documentation and the LongFast/EU_868 default channel (869.525
 * MHz - EU_868's slot-1 frequency after factory reset).
 */
static const sx1262_lora_params_t s_longfast_eu868 = {
    .freq_hz = 869525000UL,
    .sf = 0x0B,                 /* SF11 */
    .bw = 0x05,                  /* 250 kHz */
    .cr = 0x01,                  /* 4/5 */
    .low_data_rate_optimize = false,   /* symbol time 8.192ms, below the ~16.38ms LDRO threshold */
    .preamble_len = 16,
    .explicit_header = true,
    .payload_len_max = 0xFF,
    .crc_on = true,
    .invert_iq = false,
    .sync_word = 0x2B,
    .boosted_rx_gain = true,
};

esp_err_t meshtastic_radio_init(void)
{
    return sx1262_configure_lora(&s_longfast_eu868);
}

esp_err_t meshtastic_radio_start_rx(void)
{
    return sx1262_rx_start();
}

esp_err_t meshtastic_radio_stop_rx(void)
{
    return sx1262_rx_stop();
}

esp_err_t meshtastic_radio_recv(uint8_t *buf, size_t buf_cap, size_t *out_len,
                                 int16_t *rssi_dbm, int8_t *snr_db, bool *crc_ok,
                                 int timeout_ms)
{
    return sx1262_recv(buf, buf_cap, out_len, rssi_dbm, snr_db, crc_ok, timeout_ms);
}
