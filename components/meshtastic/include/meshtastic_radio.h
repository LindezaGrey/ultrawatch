/*
 * meshtastic_radio.h - Meshtastic-specific LoRa preset over sx1262.h.
 *
 * Sibling file inside the sx1262 component (not a separate component):
 * keeps sx1262.c chip-generic while this file owns the Meshtastic-specific
 * ShortSlow/EU_868 parameter set.
 * Milestone 2 (protobuf/AES packet decode) gets its next function here.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the radio for the "ShortSlow" preset, EU_868 region. Requires a
 * successful sx1262_init() and leaves the chip in STDBY_RC (call
 * meshtastic_radio_start_rx() to actually start listening). Also re-run after
 * any ALDO3 power cycle, as the radio loses its configuration when unpowered. */
esp_err_t meshtastic_radio_init(void);

esp_err_t meshtastic_radio_start_rx(void);
esp_err_t meshtastic_radio_stop_rx(void);

/* See sx1262_recv() for the exact semantics (crc_ok is a separate out-param,
 * ESP_ERR_TIMEOUT means nothing arrived in timeout_ms). */
esp_err_t meshtastic_radio_recv(uint8_t *buf, size_t buf_cap, size_t *out_len,
                                 int16_t *rssi_dbm, int8_t *snr_db, bool *crc_ok,
                                 int timeout_ms);

#ifdef __cplusplus
}
#endif
