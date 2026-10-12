# ST25R3916 — NFC Reader Notes

Notes from bringing up the ST **ST25R3916** NFC/HF reader on the T-Watch Ultra.

Every register address, bit position, direct-command opcode and timing figure
below was checked against **DS12484 Rev 8** (`Datasheets/ST25R3916_datasheet.pdf`),
section numbers given inline. Items marked *unverified* have not yet been
confirmed on hardware.

## Part & interface

| Item | Value |
|---|---|
| Part | ST ST25R3916 (13.56 MHz NFC initiator/target front end) |
| Interface | SPI **mode 1** (CPOL=0, CPHA=1), `SPI2_HOST`, shared with SD + SX1262 |
| CS | GPIO **4** |
| IRQ | GPIO **5** (input only; the driver polls, no ISR) |
| Crystal | 27.12 MHz (`Y3` on the schematic) |
| Power | AXP2101 **DLDO1** @ 3300 mV |
| Antenna | Differential — RFO1/RFO2, RFI1/RFI2, with AAT (ATT_P/ATT_S, AAT_A/AAT_B) |

The chip has no capacitive presence detection on this board, so card detection
requires the reader field to be on (see `docs/hardware.md`).

## SPI framing (§4.3.3, Table 11)

Every transaction starts with a mode byte:

| Operation | Mode byte |
|---|---|
| Register write | `00` + `A5..A0` |
| Register read | `01` + `A5..A0` → `0x40 \| addr` |
| FIFO load | `0x80` |
| FIFO read | `0x9F` |
| Direct command | `11` + `C5..C0` → the opcode itself, `0xC0`–`0xFF` |

Multi-byte register access auto-increments the address. Register space B is
reached by chaining direct command `0xFB` in front of the read/write **inside
the same CS-low window** — "access to register space-B remains active until the
rising edge of BSS".

Direct command execution starts on the **rising** edge of CS, not on the byte.

### SPI timing — this bus is the reason reads were unreliable

| Symbol | Parameter | Value (Table 124/125) |
|---|---|---|
| `T_SCLK` | min SCLK period | 100 ns (→ **10 MHz absolute max**) |
| `T_DOD` | data-out delay | 55 ns typ, **70 ns max** |
| `T_SSH` | CS high between transactions | 100 ns min |

In mode 1 the chip launches MISO on the SCLK rising edge and the master samples
on the falling edge. At 10 MHz that is only **50 ns** of margin against a 70 ns
max output delay — before adding ESP32-S3 GPIO-matrix routing delay and the bus
capacitance of three devices. 10 MHz is therefore out of spec for *reads* even
though it is nominally the max clock.

This drove a whole class of confusing symptoms, because it fails *marginally*:
stable registers (`ic_identity`) are read repeatedly and mostly land, while the
interrupt registers are **read-once-and-clear**, so one corrupted read destroys
the event permanently instead of being retried.

The NFC device now uses **2 MHz** with `input_delay_ns = 80`, configured
separately from the LoRa device it previously inherited its config from
(`components/twatch_board/twatch_board.c`).

## Power — AXP2101 DLDO1

Two register-map bugs in `components/drivers/axp2101/axp2101.c` meant the NFC
rail was never actually being controlled:

| | Was | Correct |
|---|---|---|
| Enable bit (reg `0x90`) | bit 6 | **bit 7** (bit 6 is CPUSLDO) |
| Voltage register | `0x92 + 6` = `0x98` | **`0x99`** (`0x98` is CPUSLDO) |

So every "enable/disable DLDO1" call toggled CPUSLDO, and DLDO1's 3300 mV
encoding was written into CPUSLDO's register — where it is a reserved value,
since CPUSLDO is 0.5–1.4 V in 50 mV steps — while the real NFC rail sat at its
EFUSE default. Both are fixed.

DLDO1 is configured but left off at boot. The first NFC scan raises it during
`st25r3916_open()`; after successful bring-up it remains on because repeated
power-cycling did not restore the chip reliably. Revisit that policy only with
a real re-enumeration test.

> **A retracted measurement.** An earlier note claimed the chip's own ADC
> measured VDD at ~2.28 V, below the 2.4 V minimum. That reading is not valid:
> Table 13 lists "Measure power supply" (`0xDF`) as requiring operation mode
> `en`, and it was being issued before the oscillator was enabled, where the
> command is rejected and A/D output register `0x25` still holds the previous
> conversion. Trust the PMU-side read of `0x99` instead.

## Bring-up sequence

```
DLDO1 on  →  settle  →  0xC0 Set default  →  check ic_identity (0x3F, type 0x28)
  →  pre-oscillator analog config
  →  op_control = en                  (0x02 bit 7)
  →  poll osc_ok  (Aux display 0x31 bit 4, NOT Main IRQ I_osc)
  →  0xD6 Adjust regulators           (toggle reg_s 1→0 first, §4.4.10)
  →  op_control = en | rx_en
  →  mode = 0x08  (initiator, ISO14443A)   ← writable only once osc_ok = 1
  →  bitrate = 0x00  (106 kbit/s both ways)
  →  NFC-A/106k analog config
  →  en_fd_c = 01  →  0xC8 NFC initial field ON  →  wait I_apon then I_cat
```

Two ordering constraints from the datasheet are load-bearing:

- **Mode definition (`0x03`) can only be written while `oscok = 1`** (Table 22
  footnote 1). Writing it before the oscillator is stable silently does nothing.
- **`Adjust regulators` requires toggling `reg_s` (0x2C bit 7) to 1 then back
  to 0** before issuing `0xD6`, and the command is rejected outright if `reg_s`
  is left at 1 (§4.4.10).

`sup3V` (IO config 2, `0x01` bit 7) **must** be set — it defaults to 0 = 5 V
supply mode, and this board runs the chip at 3.3 V. The driver writes
`IOCONF2_INIT_VALUE = 0xB8` (`sup3V | aat_en | miso_pd2 | miso_pd1`).

`osc_ok` is read from the **Auxiliary display register `0x31` bit 4**, not from
the Main interrupt register's `I_osc`. `I_osc` is a one-shot IRQ bit that is
destroyed by any read; `0x31` is a level.

## Interrupts (§4.3.1) — the part that is easy to get wrong

Four status registers: Main `0x1A`, Timer/NFC `0x1B`, Error/wake-up `0x1C`,
Passive target `0x1D`. The rules that matter:

- **Reading a register resets its content to 0.** Each register clears on its
  own read; reading `0x1A` does *not* clear `0x1B`–`0x1D`.
- **More than one bit can be set** between polls.
- Masking (`0x16`–`0x19`) only gates the IRQ *pin*. Masked events still set the
  status bit, and a read still returns and clears them. All four mask registers
  default to 0 (unmasked), so masking was never the reason status registers read
  back `0x00`.
- Status bits are also cleared by `Set default`, `Stop all activities` and
  **`Clear FIFO`** — so `Clear FIFO` must come *before* the transmit command,
  never between transmit and the wait.

The consequence: **you cannot wait for two events with two sequential
single-shot reads.** A REQA → ATQA exchange at 106 kbit/s finishes in a few
hundred microseconds, well inside a 2 ms poll interval, so `I_txe` and `I_rxe`
almost always arrive in the same read. Waiting for `I_txe` first returns
successfully *and silently discards `I_rxe`*, and the subsequent `I_rxe` wait
then blocks until timeout — which reads exactly like "the tag never answered".

The driver reads all four registers in one auto-incrementing 4-byte read and
accumulates into a sticky 32-bit mask that persists for the whole wait:

```
bits  0..7  Main (0x1A)      8..15 Timer/NFC (0x1B)
bits 16..23 Error (0x1C)    24..31 Passive target (0x1D)
```

## Field on

Use direct command **`0xC8` NFC initial field ON**, not a direct write to
`tx_en`. Writing `tx_en` by hand leaves the bit reading back as set while the
RFI amplitude measures 0 — i.e. nothing radiates. Table 21 says `tx_en` "is
automatically set by NFC Field ON commands"; `0xC8` performs the initial RF
collision avoidance and then switches the transmitter on, signalling `I_apon`
(`0x1D` bit 5) and, after the guard time, `I_cat` (`0x1B` bit 1). A detected
external field gives `I_cac` (`0x1B` bit 2) and the transmitter stays off.

`0xC8` requires operation mode `en`, and §4.4.5 requires the external field
detector to be enabled for it to work at all.

`en_fd_c<1:0>` (`0x02` bits 1:0) must be **01** in reader mode — "manually
enable external field detector with collision-avoidance threshold". `11`
(automatic) is for NFCIP-1 active communication and passive target modes. This
matters more than it looks: Table 21 notes that `en_fd_c != 0` with every other
bit of `op_control` clear puts the device into **Low power initial NFC mode**.

> **A retracted theory.** The tree previously carried the claim that `0xC8`
> "clears en/rx_en/tx_en wholesale", based on `op_control` reading back `0x03`
> straight after field-on, and worked around it by force-writing
> `en|rx_en|tx_en`. That does not hold up. `0x03` is exactly what the closing
> read-modify-write `reg_change_bits(op_control, 0x03, en_fd)` produces **when
> its read returns `0x00`** — `(0x00 & ~0x03) | 0x03`. The observation is
> explained by a corrupted read (see the SPI timing section) or a genuine
> power-on reset, since `op_control` resets to `0x00`. The force-write is gone.

## FIFO status (Table 66/67)

`0x1E` holds byte count bits 7:0. In `0x1F`, the count MSBs `fifo_b<9:8>` are
**bits 7:6** — bits 1:0 are `fifo_lb0` and `np_lb` (bits in the last byte /
missing parity), and reading them as count bits inflates the result by 256 or
512 on any frame with an incomplete last byte.

```c
n = ((size_t)((s2 >> 6) & 0x03) << 8) | s1;
```

## Timers — defaults are fine, deliberately left alone

- **No-response timer** (`0x10`/`0x11`) defaults to all-zero, which per Table 49
  means *the timer is not started* — the receive window stays open rather than
  closing early. The driver bounds the wait on the host clock instead, so this
  is left at its default on purpose.
- **Mask-receive timer** (`0x0F`) defaults to `0x08` ≈ 37.8 µs, enough to cover
  receiver transients after end of TX.

## Analog configuration

The datasheet does not specify board-level analog values; they depend on the
antenna and PCB. The values in `apply_analog_defaults_pre_osc()` and
`apply_analog_defaults_nfca_106()` were transcribed from RFAL's
`rfal_rfst25r3916_analogConfigTbl.h` (stm32duino/ST25R3916 — the same library
LilyGoLib's `examples/factory/app_nfc.cpp` wraps), narrowed to the NFC-A /
106 kbit / OOK reader path.

`single = 0` (differential driving) is correct here: the schematic shows both
RFO1 and RFO2 driving a 270 nH / 680 pF matching network into RFI1/RFI2.

The board has real AAT hardware: register A drives the `ATT_S` series tuning
elements and register B drives the `ATT_P` shunt element. A hardware sweep found
amplitude nearly flat at 36–38 and phase ranging only from raw 161–179, so the
nominal raw phase target of about 128 was outside the available tuning range.
Practical tag reads were the useful optimization criterion instead:

| A | B | Successful reads | Result |
|---:|---:|---:|---|
| `0x00` | `0x00` | 14/58 | weak |
| `0x82` | `0x82` | 37/70 | generic RFAL baseline |
| `0x62` | `0x00` | 109/109 | reliable |
| `0x82` | `0x00` | 109/109 | reliable |
| `0xA2` | `0x00` | 110/110 | reliable |

Register A therefore has a broad useful plateau, while reducing B to zero made
the decisive improvement. The board default is `A=0x82`, `B=0x00`: it retains
the existing midpoint for A and changes only the control demonstrated to
matter. A moving edge-of-range run at this setting produced 98/153 reads and
clean tag-present/lost transitions; exact physical distance depends heavily on
tag antenna and alignment.

## Deferred: easier antenna tuning and measurements

Deferred at the user's request on 2026-09-09. The manual tuning process was
cumbersome; the next iteration should automate measurements and make tag
position repeatable. The user has a **PN532 module** available. Flipper Zero
and a VNA are possible alternatives, not confirmed available equipment.
The current validated default remains `A=0x82`, `B=0x00`.

| Option | Useful for | Limits / prerequisites |
|---|---|---|
| Watch's ST25R3916 | Antenna amplitude and phase; receive RSSI; automatic tuning algorithms | Amplitude/phase are already exposed by `nfctune`. Receive RSSI needs driver support and validation. These are not interchangeable measurements. |
| PN532 in target/card-emulation mode | A programmable counterpart for repeated detection and response tests | Confirm the module, host interface and emulated protocol fit our NFC-A test. It is not a calibrated field-strength meter, and target mode does not imply full NTAG emulation. |
| Flipper Zero | Convenient card emulation and reader analysis | Its standard NFC application is not an antenna analyzer. Emulation coverage depends on card type. |
| VNA with a suitable fixture | Antenna impedance and matching measurements to investigate hardware limits | Requires access to the matching network and the correct differential measurement setup. Disable the reader transmitter before direct connection, following ST's procedure, to avoid damaging the VNA. |

Proposed first implementation, using a real passive tag:

1. Fix the watch and tag in a repeatable position with a non-metallic spacer,
   near the edge of reliable reception. Keep orientation and nearby objects
   unchanged during comparisons.
2. Automate a coarse A/B sweep, then refine around the best region. Use the
   same number of attempts per setting and reset the tag between attempts.
3. Log A/B values, successful reads / attempts, response times, amplitude,
   phase and, once implemented, receive RSSI. Keep raw results for comparison;
   do not automatically persist a new default before validation.
4. Re-test finalists at another distance and with the watch worn. Keep the
   current default as the baseline and favor a broad reliable region over
   one isolated winning point.
5. Optionally use the PN532 or Flipper as an additional automated target.
   Keep a passive tag as the final reference: powered emulators can behave
   differently from a tag that obtains all its energy from the watch's field.
   Do not operate both devices as active readers during this test.

The earlier sweep showed amplitude almost unchanged despite a large change
in successful reads. Maximizing amplitude alone is therefore not a sufficient
optimization criterion for this board. ST's automatic tuning algorithms are
worth evaluating, but their amplitude/phase targets need to suit the actual
matching network; blindly chasing the previously unreachable phase target
would not establish better practical range.

References for resuming this work:

- [ST25R3916 measurement capabilities](https://www.st.com/en/nfc/st25r3916.html).
- [ST AN5322: automatic antenna tuning and VNA connection procedure](https://www.st.com/resource/en/application_note/an5322-automatic-antenna-tuning-aat-for-st25r391616b-and-st25r392020b-devices-stmicroelectronics.pdf).
- [ST AN5276: antenna design and matching measurements](https://www.st.com/resource/en/application_note/an5276-antenna-design-for-st25r391616b-st25r391717b-st25r3918-st25r3919b-and-st25r392020b-devices-stmicroelectronics.pdf).
- [NXP PN532 datasheet](https://www.nxp.com/docs/en/nxp/data-sheets/PN532_C1.pdf).
- [Flipper Zero NFC documentation](https://docs.flipper.net/zero/nfc).

## Shared SPI bus

`SPI2_HOST` carries the SD card (20 MHz, mode 0, mounted on demand), the SX1262
(10 MHz, mode 0, DIO1-interrupt-driven RX) and the ST25R3916 (2 MHz, **mode 1**).
Nothing in the repo calls `spi_device_acquire_bus()`. This is legal: ESP-IDF
serializes transactions per host and reprograms CPOL/CPHA per device, and the
one hard requirement — space-B access and command chaining living in a single
CS-low window — is met by packing each such sequence into one
`spi_transaction_t`. Worth revisiting if reads turn out to still be flaky.

## Debugging

The console command `nfcpoll [seconds]` (`main/debug/debug_cmds.c`, registered in
`main/uwatch_main.c`) opens the reader, retries `st25r3916_try()` for the given
window (default 8 s) and closes. Console is USB-Serial-JTAG on `/dev/ttyACM0`.

`nfctune <A> <B> [seconds]` temporarily applies one AAT register pair after
bring-up, waits 3 ms for the varicaps, and reports raw amplitude and phase. If
`seconds` is non-zero it polls for the full interval, reports tag-present/lost
transitions and a successful-read ratio, briefly cycling the RF field after
each read so the selected tag can answer the next REQA. This allows a range
comparison without rebuilding. Values accept decimal or `0x` notation, and
normal `0x82/0x00` defaults are restored on the next reader open.

Reference for a healthy log: `ic_identity` type nibble `0x28` after each stage,
`op_control` with `en|rx_en|tx_en` set after field-on *without* any force-write,
an RFI amplitude and phase baseline at the configured `0x82/0x00` AAT point,
then `I_apon` → `I_cat` → `I_txe` → `I_rxe` and a 2-byte ATQA in the FIFO.


## Root cause: the SD card's rail and the shared SPI bus

**This was the bug.** SPI2 is shared by the SD card, the SX1262 and the
ST25R3916, and `sd_log_unmount()` cuts the SD card's rail (ALDO1) while
leaving its DAT0 pin tied to the shared MISO net. Under the on-demand SD
lifecycle the card is unmounted most of the time.

An unpowered device does not release a bus. Its I/O pins clamp the net through
their ESD protection diodes toward its own (now 0 V) supply. So with ALDO1
down, **every read on SPI2 returns `0x00`**. Writes are unaffected — MOSI is
driven by the host — which is exactly what made this so hard to see: the
ST25R3916 was being configured correctly the entire time and simply could not
be read back.

Measured with the identity register (`0x3F`) read through a temporary SPI
device, sweeping the SD rail against the NFC rail:

| SD rail | NFC rail (DLDO1) | mode | clock | `ic_identity` |
|---|---|---|---|---|
| off | off | 1 | 1 MHz | `00` |
| off | off | 1 | 10 MHz | `00` |
| off | on | 1 | 1 MHz | `00` |
| off | on | 1 | 10 MHz | `00` |
| **on** | off | 1 | 1 MHz | **`2a`** |
| **on** | off | 1 | 10 MHz | **`2a`** |
| **on** | on | 1 | 1 MHz | **`2a`** |
| **on** | on | 1 | 10 MHz | **`2a`** |

This single fact explains every symptom at once:

- `ic_identity` reading `0x00` — "chip absent/unpowered".
- All four interrupt status registers reading `0x00`, so no `I_txe`, `I_rxe`,
  `I_apon` or `I_cat` ever appeared.
- `op_control` reading back `0x03` after field-on. That is precisely
  `(0x00 & ~0x03) | 0x03` — `field_on()`'s own closing read-modify-write on
  `en_fd_c` operating on a read that returned zero. There was never a chip
  quirk to work around.
- The intermittency: it worked whenever the SD card happened to be mounted.

`st25r3916_open()` now brackets its whole session with
`spi2_power_hold(AXP2101_DLDO1)`/`spi2_power_release(AXP2101_DLDO1)` — see
"The `spi2_power` abstraction" below — which raises ALDO1 (if a card is
seated) for the duration and restores DLDO1 the way NFC's own hardware
constraints require.

### ALDO1 is a shared SPI2 rail, not the SD card's private one

This is bus-wide, not an NFC problem: reads of the SX1262 are corrupted the
same way whenever ALDO1 is down — including LoRa RX serviced from the DIO1
interrupt during light sleep, when the card is unmounted by definition.

So `sd_log_unmount()` now cuts ALDO1 **only when the socket is empty**. With a
card seated the rail stays up after the filesystem is unmounted and across
light sleep; with an empty socket there is nothing to clamp the bus, so the
rail is cut and the idle current saved. Card presence comes from
`twatch_sd_card_seated()` (XL9555 P10, active low), which fails safe — an I2C
error reports "seated", because wrongly believing the socket is empty is what
silently corrupts the whole bus.

Every SPI2 session (SD's own mount/unmount, NFC's open/close, and LoRa's
future on-demand power-down) now goes through the same `spi2_power` module for
this, rather than each driver managing ALDO1 directly — see below.

**What this costs.** A seated card's idle draw is now permanent rather than
paid only while the filesystem is mounted. That is the price of keeping SPI2
readable, and it is deliberate. The delta has **not been measured on this
board** — an idle SD card is typically a fraction of a milliamp, but treat
that as an expectation to verify, not a figure. If it turns out to matter, the
way out is an SD-detect interrupt plus a properly refcounted bus hold, so the
rail can drop while the bus is genuinely idle — *not* going back to cutting it
blind, which is what caused all of this.

**Known limitation:** there is no SD-detect interrupt. A card inserted while
ALDO1 is down can clamp the bus until the next SPI2 session evaluates the
fresh card-present predicate and raises the rail. SD mount, NFC open and other
`spi2_power_hold()` users perform that evaluation; a normal wake deliberately
does not mount the filesystem.

### The `spi2_power` abstraction

`components/spi2_power` is the single place that owns every SPI2-adjacent
rail's on/off policy, replacing the ad hoc, single-purpose logic that used to
be duplicated between `st25r3916.c` and `sd_log.c`. Each rail is registered
once (in `twatch_board.c`) with one of two policies:

- **`SPI2_POWER_SHARED`** — raised by any caller's `spi2_power_hold()` if a
  supplied predicate currently says so (or unconditionally, with no
  predicate), lowered only on the *last* `spi2_power_release()`, re-evaluating
  the predicate fresh at that moment. ALDO1 (predicate:
  `twatch_sd_card_seated()`) and ALDO3 (no predicate — see below) are
  registered this way.
- **`SPI2_POWER_OWNED`** — raised only when its own registrant explicitly asks
  (by naming its rail in `spi2_power_hold()`), never auto-lowered by
  `spi2_power_release()`. DLDO1 is registered this way: it was never a bus
  hazard for anyone else (see "The NFC rail does not gate register access"
  below), but the chip doesn't reliably survive a rail power-cycle once
  bring-up has completed, so once raised it stays raised. The one documented
  exception is `st25r3916_open()`'s own bring-up-*failure* path, which still
  powers DLDO1 back down directly — bypassing this module on purpose, the
  same way `sd_log_mount()`'s retry-toggle recovery dance always has.

Full design rationale: `docs/adr/0005-spi2-bus-power-abstraction.md`.

### Measured: does LoRa's rail clamp the bus the same way SD's does?

LoRa's rail (ALDO3) is about to stop being permanently on, which raises the
same question SD already answered: does cutting it corrupt SPI2 reads for
everyone else? The schematic gives circumstantial evidence for "yes" — the
SX1262 module (HPB16B3) has one VCC pin for the whole package, no separate
always-on I/O rail like the ST25R3916 has — so `spi2_power`'s design (above)
registers ALDO3 as `SPI2_POWER_SHARED` on that assumption.

**Measured on hardware, it doesn't hold.** `nfcprobe` (extended to sweep SD ×
LoRa × NFC × SPI settings) reads the identity register through every
combination:

```
SD=off LORA=off NFC=off  mode=1  1000000 Hz: ic_identity=00 00
SD=off LORA=off NFC=on   mode=1  1000000 Hz: ic_identity=00 00
SD=off LORA=on  NFC=off  mode=1  1000000 Hz: ic_identity=00 00
SD=off LORA=on  NFC=on   mode=1  1000000 Hz: ic_identity=00 00
SD=on  LORA=off NFC=off  mode=1  1000000 Hz: ic_identity=2a 2a  <== OK
SD=on  LORA=off NFC=on   mode=1  1000000 Hz: ic_identity=2a 2a  <== OK
SD=on  LORA=on  NFC=off  mode=1  1000000 Hz: ic_identity=2a 2a  <== OK
SD=on  LORA=on  NFC=on   mode=1  1000000 Hz: ic_identity=2a 2a  <== OK
```

(mode 1 rows shown; mode 0 behaves as already documented above — see "On the
SPI clock" — independent of LoRa's rail state.) Whether ALDO3 is on or off
makes **no difference** to whether reads succeed, as long as ALDO1 is on. SD
is still the deciding factor; LoRa isn't.

This doesn't retroactively make the `SPI2_POWER_SHARED` registration for
ALDO3 wrong — it costs nothing today (ALDO3 is still permanently on, so the
registration has no observable effect yet) and one measurement session isn't
grounds to declare the hazard categorically absent on every unit. But it is a
real data point against the worst-case assumption, and whoever implements
LoRa's actual on-demand power-down should treat it as exactly that — evidence
to build on, not a settled fact — and is free to relax the registration once
they've verified further under their own power-down policy's actual timing
(this sweep used a 150 ms settle per state; a longer, sustained rail-off is
untested).

One more thing this run showed: the SX1262's RF configuration (TCXO,
frequency, modulation) survived the probe's brief ALDO3 toggling well enough
that `mesh_log` resumed decoding Meshtastic packets immediately afterward,
with no reboot needed — likely because the rail was never off long enough to
fully discharge the module's supply during a 150 ms-per-state sweep. That is
not a guarantee for every toggle pattern; don't take one clean run as proof
the chip retains its configuration across an arbitrary, sustained power-down.

### The NFC rail does not gate register access

Note the second column above: the identity register reads fine with **DLDO1
off**. This chip's host interface is powered from VDD_IO, which the schematic
takes from the always-on DC3V3 rail, not from DLDO1.

So "the chip answers SPI" was never evidence that its analog supply was up,
and any earlier conclusion resting on that is worth re-checking. It also means
DLDO1 gates only the analog/RF side, which is where it matters — but a
register-level probe cannot tell you whether that rail is healthy.

### Confirmed working

```
op_control after field_on(): 0xc9 (expected 0xc9: en|rx_en|tx_en|en_fd_c=01)
RFI amplitude reading: 36 (0-255, higher = more signal)
tag found: ATQA=4400 UID=1D E2 A9 40 1A 10 80 (7 bytes)
```

Repeatable, ~180 ms from `open()` to UID. Two things worth reading off that:

- `op_control` is `0xc9` = `en | rx_en | tx_en | en_fd_c=01`, with no
  force-write. Table 21 was right: `tx_en` is set automatically by the NFC
  field-on command.
- A 7-byte UID means two anticollision cascade levels completed, so the sticky
  interrupt mask is delivering both `I_txe` and `I_rxe` correctly.

The earlier "RFI amplitude 15-16/255, so the field must be weak" reading was
taken through a clamped bus and meant nothing. The real figure is ~35.

### On the SPI clock

The sweep shows mode 1 at 10 MHz reading correctly once the bus is not
clamped, so 10 MHz is not *the* bug. It is still out of spec for reads by the
timing analysis above, and the sweep gives direct evidence of marginality:
**mode 0** at 10 MHz — which should never work — returned `ff ff` in one rail
state and a spuriously correct `2a 2a` in another. That is aliasing across the
sampling edge, exactly what too little `T_DOD` margin looks like. The driver
runs at 2 MHz with `input_delay_ns = 80` for margin.
