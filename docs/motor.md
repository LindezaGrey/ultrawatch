# DRV2605 — Haptic Motor Notes

Notes from bringing up the TI **DRV2605** haptic driver on the T-Watch Ultra.
Verified on hardware unless marked otherwise.

## Part & interface

| Item | Value |
|---|---|
| Part | TI DRV2605 (haptic driver for ERM/LRA actuators) |
| I2C address | `0x5A` |
| Enable pin | XL9555 **GPIO6** (M_EN, active high) |
| Actuator | ERM (eccentric rotating mass) motor, library 1 |
| Power | AXP2101 BLDO2 (speaker rail, 3.3 V) |

The driver is controlled over the board's shared I2C bus. The **M_EN** line on
the XL9555 must be asserted *before* any motor activity: it powers the
actuator stage, and the DRV2605 will not drive the motor (or calibrate)
without it.

## Registers used

| Reg | Name | Value / meaning |
|---|---|---|
| `0x01` | MODE | `0x00` internal trigger, `0x07` auto-calibration |
| `0x03` | LIBRARY | `1` (ERM library 1) |
| `0x04`–`0x0B` | Waveform sequencer | one wave per slot |
| `0x0C` | GO | bit 0 = go |
| `0x16` | RATEDV | rated voltage |
| `0x17` | CLAMPV | overdrive clamp |
| `0x18` | AUTOCALCOMP | drive compensation (calibration result) |
| `0x19` | AUTOCALEMP | back-EMF sample (calibration result) |
| `0x1D` | DATA | real-time amplitude |

## Driver API (`components/drivers/drv2605`)

| Function | Purpose |
|---|---|
| `drv2605_init(dev)` | Exit standby, internal trigger, ERM library 1; read-back check; restores NVS calibration if present |
| `drv2605_set_waveform(dev, slot, wave)` | Program one sequencer slot |
| `drv2605_go(dev)` | Start the waveform sequence |
| `drv2605_play(dev, wave)` | Set slot 0 and go (single buzz) |
| `drv2605_auto_calibrate(dev)` | Run the on-chip auto-calibration and save to NVS |
| `drv2605_calibrate_restore(dev)` | Reload the stored calibration from NVS |

## Calibration persistence (important)

The DRV2605 has **no non-volatile storage**: calibration results
(`AUTOCALCOMP` / `AUTOCALEMP`) are volatile RAM and lost on power-down. The
chip must be calibrated after every power-on, which takes ~1 s and buzzes the
motor — undesirable on every boot.

So the host persists the calibration:

- `drv2605_auto_calibrate()` runs the chip's auto-calibration and stores the
  resulting `AUTOCALCOMP`/`AUTOCALEMP` in NVS (`drv2605` namespace, keys
  `cal_comp` / `cal_emf`).
- `drv2605_init()` restores them on boot if present (verified: `comp=0x04`,
  `emf=0x9C`).
- Auto-calibration is **not** run automatically at init because it needs M_EN
  (the motor rail), which the board keeps off during init. Run it once with
  the `motor cal` console command (or a future UI path that asserts M_EN
  first).

## Console commands

| Command | Effect |
|---|---|
| `motor` | Play waveform 47 (single buzz, M_EN toggled around use) |
| `motor cal` | Run on-chip auto-calibration (~1 s motor buzz) and save to NVS |
| `motor calrestore` | Reload the stored calibration from NVS |

## Hardware notes

- M_EN lives on XL9555 GPIO6 (`TWATCH_XL_GPIO_HAPTIC_EN`); matches the
  arduino-esp32 variant (`DRV_EN=6`).
- The motor shares the speaker rail (BLDO2); nothing else on that rail is in
  use yet.
- The driver sits on I2C at `0x5A`; a read-back of the MODE register is used
  as a presence check at init (`DRV2605 initialized (mode=0x00, ERM, library
  1)`).
