/*
 * st25r3916.h - ST ST25R3916 NFC/HF reader (SPI).
 *
 * Shares the board SPI bus (twatch_board). CS 4, IRQ 5. Note: no
 * integrated presence detection - enable the reader to scan for cards.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST25R3916_PIN_CS  4
#define ST25R3916_PIN_IRQ 5

typedef struct {
    uint8_t  uid[10];
    uint8_t  uid_len;
    bool     found;
} st25r3916_tag_t;

esp_err_t st25r3916_init(spi_device_handle_t spi);
esp_err_t st25r3916_poll(st25r3916_tag_t *tag, int timeout_ms);

#ifdef __cplusplus
}
#endif
