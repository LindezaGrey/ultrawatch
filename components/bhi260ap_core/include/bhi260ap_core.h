#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t bhi260ap_core_probe(bool *product_id_ok, uint8_t *product_id);
