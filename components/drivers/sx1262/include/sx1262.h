/*
 * sx1262.h - Semtech SX1262 LoRa transceiver (SPI, 868 MHz EU).
 *
 * Shares the board SPI bus (twatch_board). Chip-control pins: CS 36,
 * RESET 47, BUSY 48, DIO1/IRQ 14. Built on the Espressif SPI master driver.
 *
 * Chip-generic only: SF/BW/CR encoding, frequency, sync word etc. are all
 * plain SX1262 concepts here. Meshtastic-specific presets live in
 * meshtastic_radio.h, which calls sx1262_configure_lora() with its own
 * parameter values - this driver has no notion of "Meshtastic".
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SX1262_PIN_CS    36
#define SX1262_PIN_RESET 47
#define SX1262_PIN_BUSY  48
#define SX1262_PIN_IRQ   14

#define SX1262_FREQ_868  868000000UL
#define SX1262_FREQ_915  915000000UL
#define SX1262_FREQ_920  920000000UL

/* LoRa modem/packet configuration. Fields use the SX1262's own raw enum
 * values (see the datasheet's SetModulationParams/SetPacketParams tables),
 * not human units - callers build these from a preset, not by hand. `sync_word`
 * carries the 8-bit public/private value (for example 0x34/0x12). */
typedef struct {
    uint32_t freq_hz;
    uint8_t  sf;                    /* 5..12 */
    uint8_t  bw;                    /* SX1262 LoRa BW (0x04=125k, 0x05=250k, 0x06=500k) */
    uint8_t  cr;                    /* SX1262 CR enum: 0x01=4/5 .. 0x04=4/8 */
    bool     low_data_rate_optimize;
    uint16_t preamble_len;          /* symbols */
    bool     explicit_header;
    uint8_t  payload_len_max;       /* accept-filter ceiling, not per-packet length */
    bool     crc_on;
    bool     invert_iq;
    uint16_t sync_word;             /* e.g. 0x2B; driver does the nibble->register expansion */
    bool     boosted_rx_gain;       /* reg 0x08AC = 0x96 (boosted) vs 0x94 (power-saving) */
} sx1262_lora_params_t;

/* Bring up the chip: configure GPIO/IRQ, reset, request STDBY_RC, and verify
 * command/status and device-error access. Returns the first failure; no later
 * SX1262 API is valid unless initialization succeeds. */
esp_err_t sx1262_init(spi_device_handle_t spi);

/* Configure the chip for LoRa RX/TX with the given parameters (the full
 * datasheet SS14.3 sequence except SetRx - leaves the chip in STDBY_RC). */
esp_err_t sx1262_configure_lora(const sx1262_lora_params_t *params);

/* Retune the carrier frequency only (owns the Hz -> RfFreq register
 * formula); accepts only the SX1262's 150-960 MHz range and is valid in
 * standby, after successful initialization. Retuning invalidates LoRa setup;
 * call sx1262_configure_lora() again before starting RX. */
esp_err_t sx1262_set_frequency(uint32_t freq_hz);

/* Enter RX Continuous mode (stays in RX indefinitely across packets). Requires
 * successful init and LoRa configuration; rejects a second start while active. */
esp_err_t sx1262_rx_start(void);

/* Leave RX (SetStandby). */
esp_err_t sx1262_rx_stop(void);

/* Wait up to timeout_ms for one packet. On ESP_OK, out_len/rssi_dbm/
 * snr_db/crc_ok are all filled in - crc_ok is a separate out-param, not
 * folded into the return code, since a CRC-failed frame is still useful
 * raw-capture signal. Returns ESP_ERR_TIMEOUT if nothing arrived (a
 * host-clock-bounded wait: RX Continuous mode never fires a hardware
 * Timeout IRQ on its own). `timeout_ms` must be nonnegative. `buf_cap` must
 * cover the whole received frame; a too-small buffer returns
 * ESP_ERR_INVALID_SIZE rather than truncating. */
esp_err_t sx1262_recv(uint8_t *buf, size_t buf_cap, size_t *out_len,
                       int16_t *rssi_dbm, int8_t *snr_db, bool *crc_ok,
                       int timeout_ms);

/* Not implemented - RX-only for now. */
esp_err_t sx1262_send(const uint8_t *buf, size_t len, int timeout_ms);

/* Instantaneous RSSI (dBm) while listening - confirms the RX front-end is
 * alive (picking up real RF noise) independent of whether a valid packet
 * has ever actually been received. */
esp_err_t sx1262_get_rssi_inst(int16_t *rssi_dbm);

/* Sticky hardware error flags (calibration/XOSC/PLL faults etc, datasheet
 * Table 13-85) - a finer-grained health check than GetStatus's coarse mode
 * bits. `errors` must point to a writable 16-bit value. */
esp_err_t sx1262_get_device_errors(uint16_t *errors);
esp_err_t sx1262_clear_device_errors(void);

/* Shared-SPI power policy callback: true while the radio is in RX and its
 * rail must remain on to keep the bus electrically safe. */
bool sx1262_spi2_power_needed(void);

#ifdef __cplusplus
}
#endif
