/*
 * st25r3916.h - ST ST25R3916 NFC/HF reader (SPI), ISO14443-A tag detection.
 *
 * Shares the board SPI bus (twatch_board). CS 4, IRQ 5. No RESET line - Set
 * default is a direct command, not a pin. Reader (initiator) mode only:
 * REQA -> ATQA -> SEL_CLx anticollision/select cascade to recover a tag's
 * UID. No NDEF, no card emulation, no genuine multi-tag collision
 * resolution beyond what the chip's anticollision framing support gives -
 * see the driver's top-of-file comment for the primary-source references.
 *
 * The rail (DLDO1) is dedicated to this chip and owned by this driver, but
 * unlike this codebase's other on-demand-rail drivers (sd_log_mount()/
 * unmount(), m10q_power()), the chip is NOT power-cycled per attempt:
 * LilyGo's own reference firmware (LilyGoWatchUltra.cpp's initPMU())
 * enables DLDO1 once at boot and leaves it on permanently, same as every
 * other rail, and repeatedly power-cycling it here (this driver's first
 * version did, once per st25r3916_poll() call) left only ~5ms of settle
 * time before hammering the chip with a full bring-up sequence on every
 * single call - a real, confirmed-different-from-reference behavior.
 * st25r3916_open() brings the chip up once (rail on, full config); repeat
 * st25r3916_try() calls only redo the REQA/cascade exchange; st25r3916_close()
 * powers back down when the caller is done. */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST25R3916_PIN_CS  4
#define ST25R3916_PIN_IRQ 5

typedef struct {
    uint8_t  uid[10];   /* up to 10 bytes: ISO14443-A cascade level 3 (CT+CT+7) */
    uint8_t  uid_len;
    bool     found;
} st25r3916_tag_t;

/* Wire up the SPI device and the PMU handle (for on-demand DLDO1 power);
 * does not power the chip or talk to it - st25r3916_open() does that. Logs
 * and returns ESP_OK even on failure (matches every other board driver's
 * log-and-continue convention). */
esp_err_t st25r3916_init(spi_device_handle_t spi, i2c_master_dev_handle_t pmu);

/* Power the chip on and run its full bring-up (Set default, chip-identity
 * check, oscillator start, regulator calibration, analog tuning, field on).
 * Call once before one or more st25r3916_try() calls; call st25r3916_close()
 * when done. Returns the same error codes bring-up can fail with (chip
 * absent/unresponsive, oscillator never stabilizes, ...) - never
 * ESP_ERR_NOT_FOUND (that's a try()-only outcome). */
esp_err_t st25r3916_open(void);

/* One REQA + anticollision/select cascade attempt against whatever chip
 * state st25r3916_open() left behind - does NOT redo bring-up or touch the
 * rail. Returns ESP_OK with tag->found=true and tag->uid/uid_len filled in
 * when a tag answered; ESP_ERR_NOT_FOUND if nothing answered REQA within
 * timeout_ms; any other esp_err_t on a genuine SPI/protocol failure. Safe
 * to call repeatedly (e.g. in a polling loop) between one open()/close()
 * pair. */
esp_err_t st25r3916_try(st25r3916_tag_t *tag, int timeout_ms);

/* Power the chip back down. Safe to call even if st25r3916_open() failed. */
void st25r3916_close(void);

/* Diagnostic primitive: probe the IC identity register through the driver's
 * own permanent SPI device (the one wired up by st25r3916_init(), same one
 * st25r3916_open()/try() use) - deliberately does NOT create a temporary
 * spi_device_handle_t on the same CS pin. An earlier version did, and
 * ESP-IDF warned "GPIO 4 is conflict with others and be overwritten" every
 * time it ran: adding a second device on a CS pin already bound to an
 * existing one reprograms the GPIO matrix routing for that pin, and nothing
 * guarantees spi_bus_remove_device() restores it correctly afterward for the
 * device left behind. That is the leading suspect for an st25r3916_open()
 * session going permanently unresponsive (ic_identity reading 0xFF, not the
 * SD-clamp's 0x00) after this probe had run earlier in the same boot -
 * cleared only by a full reboot, which is consistent with corrupted GPIO
 * matrix state rather than a chip-side fault (a temporary device's own
 * reads were fine throughout).
 *
 * Touches no power rail either - lets a caller sweep whatever rail axes
 * matter (the NFC rail, the SD card's rail, LoRa's rail - any of which can
 * clamp this shared bus's MISO line through their own ESD protection
 * diodes when unpowered) without this call itself perturbing anything.
 * Fills out_id[2] with two consecutive reads and returns true when they
 * agree and carry the ST25R3916 type nibble.
 *
 * No SPI mode/clock parameters (unlike the version this replaced): the
 * permanent device's settings are fixed at board-init time
 * (twatch_board.c), so sweeping them is no longer possible through this
 * function - that question is already answered (see docs/nfc.md's SPI
 * timing section) and isn't worth the CS-sharing risk to re-ask. */
bool st25r3916_probe_identity(uint8_t out_id[2]);

/* Convenience wrapper: open(), one try(), close(). Equivalent to this
 * driver's original one-shot-per-call behavior - prefer st25r3916_open()/
 * st25r3916_try()/st25r3916_close() for anything that polls more than
 * once, since this pays the full bring-up cost (and a rail power-cycle)
 * on every call. */
esp_err_t st25r3916_poll(st25r3916_tag_t *tag, int timeout_ms);

#ifdef __cplusplus
}
#endif
