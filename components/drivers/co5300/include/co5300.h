/*
 * co5300.h - CO5300 2.06" QSPI AMOLED panel driver (410x502, RGB565).
 *
 * Driven via the official esp_lcd SH8601 driver component (SH8601 is
 * CO5300-register-compatible) on SPI3_HOST with esp_lcd's QSPI panel IO.
 *
 * Notes (verified on hardware):
 *   - The panel samples RGB565 BIG-endian; LVGL renders RGB565_SWAPPED so no
 *     swap is needed on the flush path.
 *   - The panel requires EVEN x/y draw boundaries; the LVGL area rounder
 *     ensures this.
 *   - The 22-column GRAM offset is handled internally via set_gap.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO5300_RES_X 410
#define CO5300_RES_Y 502

/* Dedicated QSPI pins (display only). */
#define CO5300_PIN_CS       41
#define CO5300_PIN_SCK      40
#define CO5300_PIN_DATA0    38
#define CO5300_PIN_DATA1    39
#define CO5300_PIN_DATA2    42
#define CO5300_PIN_DATA3    45
#define CO5300_PIN_RESET    37
#define CO5300_PIN_TE       6

/* Common RGB565 colors. */
#define CO5300_BLACK        0x0000
#define CO5300_WHITE        0xFFFF
#define CO5300_RED          0xF800
#define CO5300_GREEN        0x07E0
#define CO5300_BLUE         0x001F
#define CO5300_DARK_NAVY    0x0841
#define CO5300_GRAY         0x8410

/* Initializes the panel with output on unless leave_display_off is true. */
esp_err_t co5300_init(bool leave_display_off);
esp_err_t co5300_deinit(void);

/* Re-run the full panel bring-up (hardware reset + init command list + gap +
 * display on) on the existing panel handle, to recover a panel that stopped
 * reflecting what it is sent. Caller must hold the LVGL lock. */
esp_err_t co5300_reinit(bool with_hw_reset, bool leave_display_off);

/* Panel sleep (SLPIN) / wake (SLPOUT + DISPON + brightness). */
esp_err_t co5300_sleep(void);
esp_err_t co5300_wake(void);

/* Display output off (0x28) / on (0x29). DISPOFF blanks the panel regardless
 * of GRAM, so it is sent before SLPIN to guarantee no stale frame on wake. */
esp_err_t co5300_display_off(void);
esp_err_t co5300_display_on(void);

/* Clear the visible panel GRAM to black (before SLPIN) so no stale frame
 * flashes on wake. */
esp_err_t co5300_blank(void);

/* Set display brightness (DCS 0x51, 8-bit; 0x00..0xFF). */
esp_err_t co5300_set_brightness(uint8_t bri);

/* Access to the underlying esp_lcd SH8601 panel / panel IO handles (for
 * integration with LVGL via esp_lvgl_adapter). */
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
esp_lcd_panel_handle_t co5300_get_panel(void);
esp_lcd_panel_io_handle_t co5300_get_panel_io(void);

#ifdef __cplusplus
}
#endif
