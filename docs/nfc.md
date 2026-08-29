# ST25R3916 — NFC Reader Notes

Notes from bringing up the ST **ST25R3916** NFC/HF reader on the T-Watch Ultra.

Every register address, bit position, direct-command opcode and timing figure
below was checked against **DS12484 Rev 8** (`Datasheets/ST25R3916_datasheet.pdf`),
section numbers given inline. Items marked *unverified* have not yet been
confirmed on hardware.

> ## Current status (2026-08-29): the chip is not responding at all
>
> On hardware, the ST25R3916 currently answers **neither SPI nor I2C**, in any
> configuration. This is a more fundamental problem than the driver bugs
> documented below, and it needs a measurement, not more code. See
> [Current hardware state](#current-hardware-state) at the end.
>
> Everything else in this document is datasheet-verified and the driver fixes
> are real, but they are **unverified against a responding chip**.

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

DLDO1 is now enabled at boot and **left on**, matching LilyGO's `initPMU()`.
Power-cycling the rail per poll was tried and the chip did not reliably come
back. Revisit only with a real re-enumeration test.

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

**Open item (unverified):** `REG_ANT_TUNE_A`/`REG_ANT_TUNE_B` = `0x82` are
RFAL's generic defaults, not values calibrated for this board's antenna, and the
board has real AAT hardware (ATT_P/ATT_S). An RFI amplitude reading of 15–16/255
with the field on is low. This is the leading remaining explanation for weak
range — but it cannot explain interrupt registers reading `0x00`, so it was not
the place to start.

## Shared SPI bus

`SPI2_HOST` carries the SD card (20 MHz, mode 0, mounted on demand), the SX1262
(10 MHz, mode 0, DIO1-interrupt-driven RX) and the ST25R3916 (2 MHz, **mode 1**).
Nothing in the repo calls `spi_device_acquire_bus()`. This is legal: ESP-IDF
serializes transactions per host and reprograms CPOL/CPHA per device, and the
one hard requirement — space-B access and command chaining living in a single
CS-low window — is met by packing each such sequence into one
`spi_transaction_t`. Worth revisiting if reads turn out to still be flaky.

## Debugging

The console command `nfcpoll [seconds]` (`main/debug_cmds.c`, registered in
`main/uwatch_main.c`) opens the reader, retries `st25r3916_try()` for the given
window (default 8 s) and closes. Console is USB-Serial-JTAG on `/dev/ttyACM0`.

Reference for a healthy log: `ic_identity` type nibble `0x28` after each stage,
`op_control` with `en|rx_en|tx_en` set after field-on *without* any force-write,
then `I_apon` → `I_cat` → `I_txe` → `I_rxe` and a 2-byte ATQA in the FIFO.


## Current hardware state

As of 2026-08-29 the chip does not answer at all. What was measured, using the
`nfcprobe` and `i2cscan` console commands added for this:

**SPI sweep** — a temporary device is created per combination, `Set default`
(`0xC0`) issued, then the IC identity register (`0x3F`) read twice:

| SPI mode | Clock | `input_delay_ns` | `ic_identity` |
|---|---|---|---|
| 1 | 1 / 2 / 4 MHz | 0 and 80 | `00` |
| 1 | 6 / 10 MHz | 0 | `00` |
| 0 | 2 / 10 MHz | 0 | `00` |

**Rail sweep** — each of the four PMU states this board's firmware has ever
been in, entered cold (rails dropped, 200–1000 ms off, 100 ms settle), each
crossed with the SPI settings above. Reg `0x90` reads back exactly as
commanded in every case, so the PMU writes are landing:

| Rail state | `0x90` | `ic_identity` |
|---|---|---|
| both off | `0x1e` | `00` |
| DLDO1 on @ 3.3 V (corrected mapping) | `0x9e` | `00` |
| CPUSLDO on @ `0x98`=`0x1C` (what the pre-fix code actually toggled) | `0x5e` | `00` |
| both on | `0xde` | `00` |

**I2C scan** — the ST25R3916 selects SPI or I2C from its `I2C_EN` pin and
answers at `0x50` in I2C mode. Full 7-bit scan finds
`0x1a 0x20 0x28 0x34 0x51 0x5a` — touch, XL9555, BHI260AP, AXP2101, PCF85063A,
DRV2605. **No `0x50`.**

Ruled out by the above:

- SPI mode, clock rate and input delay — all silent, including mode 1 @ 10 MHz,
  the exact configuration under which the chip previously reported
  `ic_identity = 0x28`.
- Which PMU rail is enabled, including reproducing the pre-fix CPUSLDO state
  byte for byte.
- The chip having quietly fallen back to I2C.
- A bus fault: the SX1262 shares MOSI/MISO/SCK on SPI2 and enumerates fine
  (`SX1262 found, status=0xa2`) on every boot.
- A chip-select conflict: GPIO4 is defined exactly once in the tree, as
  `TWATCH_PIN_NFC_CS`.

What is left, and what would settle it:

1. **The chip's VDD is not actually reaching 3.3 V.** The PMU reports DLDO1
   enabled and programmed to 3300 mV, but a register bit reading back as set
   only proves the request latched — not that the LDO is sourcing current.
   *Test: meter on the ST25R3916's VDD pin / the DLDO1 net, with the rail
   commanded on.*
2. **VDD_IO / VDD power-up ordering.** VDD_IO appears to come from the
   always-on DC3V3 rail while VDD comes from DLDO1, which the firmware used to
   enable long after boot. If the part latched into a bad state from that
   ordering, only a full board power-cycle clears it — an ESP32 reset does not,
   since it never drops DC3V3. Now that DLDO1 is enabled during
   `axp2101_set_default_power()`, both rails come up together at boot.
   *Test: full power-off (`pwroff`, then the power button) rather than a reset,
   then `nfcprobe`.*
3. **The part is damaged.** Consistent with the evidence but not yet
   distinguishable from (1) and (2), and only worth concluding after both.

Note the ordering of events: the chip's last known-good `ic_identity = 0x28`
predates the AXP2101 register-map corrections. The tree already recorded it
"going unresponsive partway through this sequence (ic_identity reading 0x00)"
before those corrections were made, so the silence is not obviously caused by
them — the sweep above shows it persists with the pre-fix rail state restored.
