#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configure XL9555 pins used by the board and power up the display/touch */
esp_err_t bsp_xl9555_init(void);

esp_err_t bsp_xl9555_pin_mode(uint8_t pin, bool output);
esp_err_t bsp_xl9555_write_pin(uint8_t pin, bool level);
esp_err_t bsp_xl9555_read_pin(uint8_t pin, bool *level);

/* SD card-detect (BSP_SD_DET_PIN, active low): returns true if a card is present */
bool bsp_xl9555_sd_detect(void);

#ifdef __cplusplus
}
#endif
