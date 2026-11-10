# A shared power abstraction for SPI2 (SD + LoRa + NFC), superseding ADR 0004's deferral

ADR 0004 deferred abstracting SPI2 - shared by the SD card (ALDO1), the SX1262 LoRa
radio (ALDO3) and the ST25R3916 NFC reader (DLDO1) - "until either driver has a real
implementation." Both now do, and along the way we diagnosed the actual problem this
bus has: an unpowered device does not release it. The SD card's DAT0 pin, tied to the
shared MISO net, clamps that net through its own ESD protection diodes toward its dead
0V supply whenever ALDO1 is off, and every read on SPI2 - not just reads of the SD
card - comes back `0x00` while that's true. This was the root cause of a multi-day
false trail diagnosing the ST25R3916 as broken (see `docs/nfc.md`), fixed narrowly at
first (`sd_log_unmount()` keeping ALDO1 up while a card is seated, `st25r3916_open()`
independently holding it as a session-scoped backstop) and then generalized here.

LoRa's rail (ALDO3) was previously left on permanently. As the radio gained an
on-demand power switch, the same question arose: can ALDO3 be safely cut while another
SPI2 client is active? Hardware measurements found no evidence that ALDO3 clamps the
bus, unlike ALDO1 with a seated SD card, but the software conservatively retains ALDO3
as a `SPI2_POWER_SHARED` rail. Its predicate now reports true only while the SX1262
receiver is active; a transition hold serializes enable/disable against NFC and SD
sessions, and the final release powers the radio rail down only when no other bus
client is holding it. `docs/nfc.md` records the measurements and policy.

## The abstraction

`components/spi2_power` (`spi2_power.h`/`.c`) is a new, small leaf component. It
can't live inside `twatch_board` - `twatch_board` already depends on `st25r3916` and
`sd_log` depends on `twatch_board`, so either driver calling into `twatch_board` would
be circular (confirmed by reading both components' `CMakeLists.txt`). It doesn't
belong inside `axp2101` either - that's a chip-generic PMU driver, portable to other
boards, with no business knowing this specific board's bus topology. So it's its own
component, depended on directly by `twatch_board`, `st25r3916`, `sd_log`, and `sx1262`.

Each rail is registered once, during board bring-up, with one of two policies:

- **`SPI2_POWER_SHARED`** - raised by any caller's `spi2_power_hold()` if a supplied
  `need_fn()` predicate says so (or unconditionally, if none is given), and lowered
  only on the *last* `spi2_power_release()`, re-evaluating that predicate fresh at
  that moment rather than trusting anything cached from when the hold started. This
  is for rails that stay wired to the shared bus even when off and can corrupt every
  other device's reads: ALDO1 (predicate: `twatch_sd_card_seated()`) and ALDO3
  (predicate: `sx1262_spi2_power_needed()`, true during RX).
- **`SPI2_POWER_OWNED`** - raised only when its own registrant explicitly names it in
  a `spi2_power_hold()` call, and never auto-lowered by `spi2_power_release()`. This
  is for DLDO1 (NFC): its host interface runs off the always-on DC3V3 rail
  independent of DLDO1 itself (confirmed on the schematic and by reading the chip's
  identity register with DLDO1 off), so it was never a bus hazard for anyone else -
  but the chip has proven unable to reliably come back after a rail power-cycle once
  bring-up has completed, so once raised it stays raised until something with a
  documented reason to power-cycle it (a failed bring-up, handled directly in
  `st25r3916_open()`'s own error paths, bypassing this module on purpose) says
  otherwise.

Handling DLDO1 through the same registration table and the same `hold()`/`release()`
entry points as ALDO1/ALDO3 - rather than leaving it as ad hoc direct
`axp2101_enable_rail()` calls scattered in `st25r3916.c`, which is what the code
looked like before this - was a deliberate design goal, not an accident of the
implementation: one module owns every SPI2-adjacent rail's on/off policy, expressed
as data (the policy enum) rather than as scattered special cases in caller code, even
though the policies themselves differ. LoRa's `need_fn` tracks RX state so the radio
rail stays up while listening but can be reconciled OFF on the last bus-session release.

## Considered options

- **Fold the abstraction into `axp2101`**: rejected. `axp2101` is meant to be a
  portable, chip-generic AXP2101 driver; "which rails share a specific board's SPI
  bus" is board topology, not PMU register semantics, and every consumer already
  depends on `axp2101` directly, so nothing was gained by the merge except muddying
  that boundary.
- **A generic "force" override on `spi2_power` for bypassing `SPI2_POWER_OWNED`'s
  never-auto-lower rule**: rejected. The one place that legitimately needs to power
  DLDO1 back down outside the normal lifecycle - `st25r3916_open()`'s bring-up-failure
  path, which today already does this - keeps a direct `axp2101_enable_rail()` call
  for that documented, exceptional reason, same as `sd_log_mount()`'s own
  retry-toggle recovery dance already bypasses this module for its own reason. A
  generic override function would invite it to be reached for casually elsewhere,
  which defeats the point of the policy being data instead of caller-side special
  cases.
- **Per-transaction locking** (hold/release around each individual SPI read) instead
  of session-scoped (hold/release around a whole open/mount/rx-session): rejected as
  unnecessary overhead - nothing in this codebase's SPI2 usage pattern needs finer
  granularity than "session," and per-transaction locking would mean a settle delay
  risk on every single read instead of once per session.
