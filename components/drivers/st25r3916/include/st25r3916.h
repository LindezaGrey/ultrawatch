/*
 * st25r3916.h - ST ST25R3916 NFC/HF reader (SPI), ISO14443-A tag detection.
 *
 * Shares the board SPI bus (twatch_board). CS 4, IRQ 5. No RESET line - Set
 * default is a direct command, not a pin. Reader (initiator) mode only:
 * REQA -> ATQA -> SEL_CLx anticollision/select cascade to recover a tag's
 * UID, plus a Type 2 Tag (NTAG21x/MIFARE Ultralight family) memory read
 * (st25r3916_read_type2()) once a tag is selected - hand its output to
 * components/ndef to decode an NDEF message. No card emulation, no P2P, and no
 * genuine multi-tag collision resolution; multi-tag collisions are reported as
 * protocol errors. See the driver's top-of-file comment for source references.
 *
 * The rail (DLDO1) is dedicated to this chip and owned by this driver. It is
 * enabled during open() and remains on after a successful close(): the chip
 * has not proven reliable after power-cycling following bring-up, and LilyGo's
 * reference firmware leaves the rail on at boot. open() performs full config;
 * repeated try() calls only redo REQA/anticollision. close() stops the RF field
 * and releases shared SPI2 hazard rails, but leaves DLDO1 on. A failed initial
 * bring-up may power-cycle DLDO1 before returning, as no working session exists. */
#pragma once

#include <stddef.h>
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

/* Wire up the SPI device and PMU handle (used for DLDO1 session power).
 * Does not power or communicate with the chip; open() performs bring-up.
 * Returns errors if handles, mutex allocation, or IRQ GPIO setup fail. */
esp_err_t st25r3916_init(spi_device_handle_t spi, i2c_master_dev_handle_t pmu);

/* Power the chip on and run its full bring-up (Set default, chip-identity
 * check, oscillator start, regulator calibration, analog tuning, field on).
 * Call once before one or more st25r3916_try() calls; call st25r3916_close()
 * when done. Returns the same error codes bring-up can fail with (chip
 * absent/unresponsive, oscillator never stabilizes, ...) - never
 * ESP_ERR_NOT_FOUND (that's a try()-only outcome).
 *
 * open()/close() own the chip for their whole duration: a second task
 * calling open() while another session is running returns
 * ESP_ERR_INVALID_STATE immediately (no waiting), so a background scan and a
 * console command can never interleave bring-up, RF field state, or the
 * antenna-tuning registers. try()/tuning_measure()/field_reset()/
 * read_type2() answer the same way if called outside the caller's own
 * session. An external-field collision-avoidance veto returns
 * ESP_ERR_INVALID_STATE without cycling the initialized rail; on that veto,
 * open() releases ownership and callers should not call close(). */
esp_err_t st25r3916_open(void);

/* One REQA + anticollision/select cascade attempt against whatever chip
 * state st25r3916_open() left behind - does NOT redo bring-up or touch the
 * rail. Returns ESP_OK with tag->found=true and tag->uid/uid_len filled in
 * when a tag answered; ESP_ERR_NOT_FOUND if nothing answered REQA within
 * timeout_ms; any other esp_err_t on a genuine SPI/protocol failure. Safe
 * to call repeatedly (e.g. in a polling loop) between one open()/close()
 * pair. */
esp_err_t st25r3916_try(st25r3916_tag_t *tag, int timeout_ms);

/* Diagnostic AAT primitive, valid only between open()/close(): temporarily
 * apply one Antenna tuning control register pair, wait for the varicaps to
 * settle, then return the raw amplitude and phase measurements. The next
 * open() restores the normal board defaults. */
esp_err_t st25r3916_tuning_measure(uint8_t aat_a, uint8_t aat_b,
                                   uint8_t *amplitude, uint8_t *phase);

/* Briefly cycle the RF field so a selected tag returns to its idle state and
 * can answer another REQA. Preserves the current antenna-tuning registers. */
esp_err_t st25r3916_field_reset(void);

/* Reads Type 2 user memory (page 4 onward), bounded by `out_cap` and the
 * Capability Container's advertised size; `*out_len` reports bytes copied.
 * A read timeout/NAK after some data returns the partial data; a failed first
 * user-memory read propagates unless it indicates an absent/short response.
 * The CC must contain the NDEF magic byte (page 3 byte 0 = 0xE1); otherwise
 * ESP_ERR_NOT_FOUND is returned. Authenticated reads are not supported.
 *
 * Only valid while a tag is selected: call it after a successful
 * st25r3916_try(), before st25r3916_close(). The bytes returned are raw
 * Type 2 Tag TLV-area data - hand them to ndef_parse() (components/ndef)
 * to find and decode an NDEF message; this driver has no NDEF knowledge of
 * its own, only the chip-level read primitive. */
esp_err_t st25r3916_read_type2(uint8_t *out, size_t out_cap, size_t *out_len, int timeout_ms);

/* End the session owned by the calling task: stops the RF field and releases
 * the shared SPI2 rails. Safe to call even if st25r3916_open() failed, and a
 * no-op when this task has no session open (it never disturbs another
 * task's). */
void st25r3916_close(void);

/* True while some task owns the reader session or the exclusive probe guard. */
bool st25r3916_busy(void);

/* Reserve the reader against open() for a diagnostic that must manipulate
 * NFC/SPI2 rails outside normal driver policy (nfcprobe). Non-blocking. Only
 * the owning task can probe or release the reservation. */
esp_err_t st25r3916_probe_begin(void);
void st25r3916_probe_end(void);

/* Diagnostic primitive: called only by the task owning probe_begin(); probes
 * the IC identity through the permanent SPI device used by the reader API,
 * and does NOT create a temporary spi_device_handle_t on the same CS pin.
 * An earlier version did, and ESP-IDF warned about a GPIO 4 conflict each
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
 * Touches no power rail; the probe owner controls the rail state. Fills
 * out_id[2] with two consecutive reads and returns true when they agree and
 * carry the ST25R3916 type nibble.
 *
 * No SPI mode/clock parameters (unlike the version this replaced): the
 * permanent device's settings are fixed at board-init time
 * (twatch_board.c), so sweeping them is no longer possible through this
 * function - that question is already answered (see docs/nfc.md's SPI
 * timing section) and isn't worth the CS-sharing risk to re-ask. */
bool st25r3916_probe_identity(uint8_t out_id[2]);

/* Convenience wrapper: open(), one try(), close(). Use open()/try()/close()
 * for polling loops to avoid paying the full oscillator/configuration startup
 * on every attempt. */
esp_err_t st25r3916_poll(st25r3916_tag_t *tag, int timeout_ms);

#ifdef __cplusplus
}
#endif
