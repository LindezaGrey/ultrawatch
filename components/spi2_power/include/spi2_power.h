/*
 * spi2_power.h - shared power-sequencing policy for the T-Watch Ultra's
 * SPI2 bus (SD card, SX1262 LoRa, ST25R3916 NFC).
 *
 * SPI2's MOSI/MISO/SCK are wired to all three devices. A device that is
 * physically present on that bus but electrically unpowered does not
 * release it: its I/O pins clamp the shared MISO net through their own
 * ESD protection diodes toward its dead (0V) supply, so every read on the
 * bus - not just reads of that device - comes back 0x00. Confirmed on
 * hardware for the SD card (see docs/nfc.md); the SX1262 module has a
 * single VCC pin for its whole package with no separate always-on I/O
 * rail (unlike the ST25R3916, whose host interface is powered from the
 * always-on DC3V3 rail independent of its own DLDO1), so it is assumed
 * to behave the same way until measured.
 *
 * This module is the single place that owns each SPI2-adjacent rail's
 * on/off policy, replacing the ad hoc, single-purpose rail-holding logic
 * that used to be duplicated between the NFC and SD drivers. It cannot
 * live inside twatch_board (which depends on the NFC/SD/LoRa drivers,
 * so those drivers calling into it would be circular) or inside axp2101
 * (a chip-generic PMU driver with no business knowing about this board's
 * bus topology) - hence its own small component, depended on directly by
 * twatch_board and by each SPI2 driver.
 */
#pragma once

#include "axp2101.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*spi2_power_need_fn_t)(void);

typedef enum {
    /* Raised by ANY caller's spi2_power_hold() if need_fn() says so (NULL
     * need_fn = unconditionally). Lowered only on the LAST
     * spi2_power_release(), and only if need_fn() says no *at that time*
     * (re-evaluated fresh on every release, not cached from when the hold
     * started) - a card inserted mid-hold is picked up correctly.
     *
     * For rails that stay wired to the shared bus even when off and can
     * corrupt every OTHER device's reads: the SD card's ALDO1 (need_fn
     * checks whether a card is physically seated) and LoRa's ALDO3 (no
     * "removed" state for a soldered module - unconditional, NULL
     * need_fn, until measured otherwise). */
    SPI2_POWER_SHARED,

    /* Raised only when its own registrant explicitly asks for it, by
     * passing its rail as `owned_rail` to spi2_power_hold(). Never
     * auto-lowered by spi2_power_release() - some chips do not reliably
     * survive a rail power-cycle, so once on, this policy leaves a rail
     * on until something outside this module explicitly powers it down
     * (a documented, driver-owned exception - see st25r3916_open()'s
     * bring-up-failure path).
     *
     * For rails that don't affect bus integrity for anyone else: NFC's
     * DLDO1, whose host interface is powered from the always-on DC3V3
     * rail independent of DLDO1 itself (confirmed on the schematic and by
     * reading the chip's identity register with DLDO1 off - see
     * docs/nfc.md). */
    SPI2_POWER_OWNED,
} spi2_power_policy_t;

/* Must be called once, after the PMU is up, before any spi2_power_register()
 * or spi2_power_hold() call. */
esp_err_t spi2_power_init(i2c_master_dev_handle_t pmu);

/* Declare a rail's policy. Call once per rail, during board bring-up,
 * before anything can call spi2_power_hold(). `need_fn` is polled fresh on
 * every hold()/release() transition; pass NULL to mean "always needed"
 * (SPI2_POWER_SHARED) or "not applicable" (SPI2_POWER_OWNED, which never
 * reads need_fn at all). */
esp_err_t spi2_power_register(axp2101_rail_t rail, spi2_power_policy_t policy,
                               spi2_power_need_fn_t need_fn);

/* Take/release a SPI2 bus session. Session-scoped, not per-transaction:
 * call once around a whole open/mount/rx-session bracket (e.g. the whole
 * duration between st25r3916_open() and st25r3916_close()), not around
 * each individual SPI read. Refcounted and safe to call concurrently from
 * multiple tasks.
 *
 * On the first hold() (0 -> 1 holders): raises every SPI2_POWER_SHARED
 * rail whose need_fn() currently says yes, plus `owned_rail` if it names a
 * registered SPI2_POWER_OWNED rail (pass AXP2101_RAIL_MAX if the caller
 * has no owned rail of its own - SD and LoRa's sessions do not).
 *
 * On the last release() (1 -> 0 holders): re-evaluates every
 * SPI2_POWER_SHARED rail's need_fn() and lowers whichever now say no.
 * `owned_rail` is passed for symmetry with hold() but is never lowered
 * here - see SPI2_POWER_OWNED above. */
esp_err_t spi2_power_hold(axp2101_rail_t owned_rail);
void      spi2_power_release(axp2101_rail_t owned_rail);

#ifdef __cplusplus
}
#endif
