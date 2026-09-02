#pragma once

#include "esp_err.h"

/* Keep the shared SD/LoRa SPI bus and both device rails active. */
esp_err_t shared_spi2_acquire(void);
void shared_spi2_release(void);
