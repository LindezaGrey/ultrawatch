#include "meshtastic_radio.h"
#include "sx1262.h"

/* Meshtastic's "ShortSlow" preset, EU_868 region.
 *
 * SF/BW/CR verified against Meshtastic's own firmware source
 * (github.com/meshtastic/firmware, src/mesh/MeshRadio.h's
 * modemPresetToParams(): SHORT_SLOW/non-wideLora -> bw=250kHz, cr=5 (4/5),
 * sf=8 - the only difference from the previous LongFast preset is SF
 * (11 -> 8); BW/CR/preamble/sync word are all unchanged). Frequency stays
 * 869.525 MHz: EU_868's legal span for this radio is exactly one 250kHz-wide
 * channel slot (matching Meshtastic's own docs: "There is one frequency
 * slot defined with the standard radio preset LongFast"), and ShortSlow
 * uses the same 250kHz bandwidth, so it collapses to that same single slot
 * regardless of the preset's channel-hash name. Preamble length (16
 * symbols) is a fixed constant in RadioInterface.h, not preset-dependent:
 * "uint16_t preambleLength = 16; // 8 is default, but we use longer to
 * increase the amount of sleep time when receiving".
 *
 * IMPORTANT: switching modem preset changes the implicit name of the
 * default/unnamed channel (used to derive its PacketHeader.channel hash
 * byte - see meshtastic_crypto.h), from "LongFast" to "ShortSlow". Named
 * channels (e.g. "Mesh Hessen") are unaffected - their hash only depends on
 * their own explicit name + PSK, not the active preset. */
static const sx1262_lora_params_t s_shortslow_eu868 = {
    .freq_hz = 869525000UL,
    .sf = 0x08,                 /* SF8 */
    .bw = 0x05,                  /* 250 kHz */
    .cr = 0x01,                  /* 4/5 */
    .low_data_rate_optimize = false,   /* symbol time 1.024ms, well below the ~16.38ms LDRO threshold */
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
    return sx1262_configure_lora(&s_shortslow_eu868);
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
