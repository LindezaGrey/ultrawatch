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

typedef enum {
    DISPLAY_RECOVERY_REINIT_COMMANDS,
    DISPLAY_RECOVERY_REINIT_RESET,
    DISPLAY_RECOVERY_RAIL_CYCLE,
} display_recovery_t;

typedef enum {
    DISPLAY_DIAG_OUTPUT_OFF,
    DISPLAY_DIAG_OUTPUT_ON,
    DISPLAY_DIAG_BRIGHTNESS_ZERO,
    DISPLAY_DIAG_BRIGHTNESS_RESTORE,
} display_diag_operation_t;

esp_err_t display_controller_init(void);
void display_controller_attach_lvgl(void);
void display_controller_request_visible(display_reason_t reason);
void display_controller_request_repaint(void);
void display_controller_note_draw_complete(void);
void display_controller_set_brightness(uint8_t level);
esp_err_t display_controller_sleep(TickType_t timeout);
/* Turn panel output off and blank it for an imminent PMIC power cut. Unlike
 * display_controller_sleep(), this deliberately omits SLPIN. */
esp_err_t display_controller_power_off(TickType_t timeout);
esp_err_t display_controller_recover(display_recovery_t recovery, TickType_t timeout);
esp_err_t display_controller_diag_operation(display_diag_operation_t operation, TickType_t timeout);
esp_err_t display_controller_diag_cycle(unsigned count, TickType_t timeout);
esp_err_t display_controller_diag_read_register(uint8_t command, uint8_t *data,
                                                size_t length, TickType_t timeout);
display_state_t display_controller_get_state(void);
