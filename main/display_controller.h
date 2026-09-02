#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef enum {
    DISPLAY_REASON_BOOT,
    DISPLAY_REASON_WAKE,
    DISPLAY_REASON_DRAW,
} display_reason_t;

typedef enum {
    DISPLAY_STATE_INITIALIZING,
    DISPLAY_STATE_READY,
    DISPLAY_STATE_AWAITING_DRAW,
    DISPLAY_STATE_VISIBLE,
    DISPLAY_STATE_SLEEPING,
    DISPLAY_STATE_FAULT,
} display_state_t;

esp_err_t display_controller_init(void);
void display_controller_attach_lvgl(void);
void display_controller_request_visible(display_reason_t reason);
void display_controller_note_draw_complete(void);
void display_controller_set_brightness(uint8_t level);
esp_err_t display_controller_sleep(TickType_t timeout);
display_state_t display_controller_get_state(void);
