#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
esp_err_t bhi260ap_core_probe(bool *product_id_ok, uint8_t *product_id);
esp_err_t bhi260ap_core_load_firmware(uint8_t *boot_status);
esp_err_t bhi260ap_core_list_virtual_sensors(uint8_t *present, size_t present_size, size_t *count);
esp_err_t bhi260ap_core_start_irq_probe(void);
