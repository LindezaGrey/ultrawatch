/*
 * sx1262.h - Semtech SX1262 LoRa transceiver (SPI, 868 MHz EU).
 *
 * Shares the board SPI bus (twatch_board). Chip-control pins: CS 36,
 * RESET 47, BUSY 48, DIO1/IRQ 14. Built on the Espressif SPI master driver.
 */
#pragma once

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

esp_err_t sx1262_init(spi_device_handle_t spi);
esp_err_t sx1262_set_frequency(uint32_t freq_hz);
esp_err_t sx1262_send(const uint8_t *buf, size_t len, int timeout_ms);
esp_err_t sx1262_recv(uint8_t *buf, size_t len, int timeout_ms);

#ifdef __cplusplus
}
#endif
