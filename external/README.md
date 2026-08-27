# BHI260AP firmware package

This folder documents the official Bosch BHI260AP firmware **v1.1.8.0** (see
"How to obtain" below) — the licensed download, not currently present in this
folder or checked in (login-gated, see `.gitignore`).

## Currently deployed

`assets/bhi260/BHI260AP.fw` (also gitignored, per-machine local file — see
`bhi260ap_init()` in `components/drivers/bhi260ap/bhi260ap.c`) is presently
the **public, no-login** build instead: `BHI2xy_SensorAPI`'s
`firmware/bhi260ap/BHI260AP.fw`, SensorAPI v1.6.0 (2020-05-13), 103676 B,
md5 `381bfee9f6d8f3b672b24453922b9df9` (see "Related public resources"
below for the source). Confirmed working on hardware (2026-08-27 boot log):

```
bhi260ap: loaded firmware: 103676 bytes
bhi260ap: product id: 0x89 (expected 0x89)
bhi260ap: RAM firmware booted, kernel version 5991
bhi260ap: BHI260AP ready (persisted steps 46986)
```

Step counter, GAMERV rotation vector, activity recognition and the gestures
(wrist-tilt/wake/glance/pickup) all come from standard virtual sensors this
build exposes — no need for the licensed v1.1.8.0 package unless something
specifically requires it (e.g. PDR / pedestrian-position output, see
"Findings" below, unconfirmed either way since that build's binary is
encrypted). If the public firmware ever goes missing locally (it's
gitignored, so a fresh clone/machine won't have it), re-fetch it:

```bash
curl -sL -o assets/bhi260/BHI260AP.fw \
  https://raw.githubusercontent.com/boschsensortec/BHI2xy_SensorAPI/master/firmware/bhi260ap/BHI260AP.fw
```

## Contents

- `bhi260ap_fw_v1-1-8-0.zip` (dated 2021-11-16) contains 10 firmware images:
- `bst-bhi260_bhi360-an002.pdf` — Bosch app note
  "BSX sensor fusion for smart sensor systems" (BST-BHI260_BHI360-AN002-01,
  Nov 2023). Explains the BSX on-chip sensor-fusion features (quaternion /
  Euler orientation, virtual sensors, calibration) available on the BHI260.

| File | Purpose |
|---|---|
| `Bosch_APP30_SHUTTLE_BHI260.fw` | Standard BHI260 firmware (RAM-loadable) |
| `Bosch_APP30_SHUTTLE_BHI260-flash.fw` | Same, for flashing to BHI260 flash |
| `Bosch_APP30_SHUTTLE_BHI260_aux_BMM150.fw` / `-flash.fw` | + magnetometer support |
| `Bosch_APP30_SHUTTLE_BHI260_BME68x.fw` / `-flash.fw` | + gas/humidity (BME68x) support |
| `Bosch_APP30_SHUTTLE_BHI260_BMP390.fw` / `-flash.fw` | + pressure (BMP390) support |
| `Bosch_APP30_SHUTTLE_BHI260_turbo.fw` / `-flash.fw` | Turbo clock variant |

## Findings

- The firmware images are **encrypted/proprietary** (no readable feature
  strings). The only readable marker is `CalibBSX`, indicating the build
  contains **BSX sensor-fusion / calibration** support.
- This is a **different, newer build** than the stock firmware bundled in the
  public Sensor API repo. Comparison:
  - Public repo (`BHI2xy_SensorAPI`, SensorAPI v1.6.0, 2020-05-13):
    `BHI260AP.fw` = 103676 B, md5 `381bfee9f6d8f3b672b24453922b9df9`
  - This package (v1.1.8.0, 2021-11-16):
    `Bosch_APP30_SHUTTLE_BHI260.fw` = 106156 B, md5 `4b77f95b70209e7c608879c9f5fde902`
- For the **T-Watch Ultra** the BHI260AP is wired with **only accel + gyro**
  (I2C 0x28, INT = GPIO8; no magnetometer/pressure/humidity on board), so the
  correct image is the plain **`Bosch_APP30_SHUTTLE_BHI260.fw`** (or `-flash`
  variant).
- `-flash.fw` variants persist to the BHI260's internal flash (auto-boot);
  non-flash variants must be uploaded over I2C on every boot.
- Whether this build exposes the **PDR / pedestrian-position output** cannot be
  confirmed from the encrypted binary — it must be verified at runtime via the
  FSC host protocol (sensor ID 13 / position output).

## How to obtain

1. Create an account / log in at https://www.bosch-sensortec.com
2. Open the BHI260AP firmware download page (login required):
   https://www.bosch-sensortec.com/en/software-tools/double-opt-in-forms/firmware-bhi260ap.html
3. Accept the clickthrough license and download the firmware zip
   (`bhi260ap_fw_v1-1-8-0.zip`).
4. Place the zip in this folder (as done).

The related **SDK for BHI260AP** (SDK v1.1.18.0) is also login-gated:
https://www.bosch-sensortec.com/en/software-tools/double-opt-in-forms/sdk-bhi260ap-1-1-18.html

## Related public resources (no login, BSD-3)

- Sensor API (host-side driver + stock firmware):
  https://github.com/BoschSensortec/BHI2xy_SensorAPI

## BHI260AP documentation (Bosch)

- Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bhi260ap-ds000.pdf
- SDK Quick Start Guide (AN000)
- Programmer's Manual (AN002)
- Application note "BSX sensor fusion for smart sensor systems"
  (BHI260/BHI360 AN002) — a copy is in this folder:
  `bst-bhi260_bhi360-an002.pdf`
- Application note "Low power pedestrian position tracking" / PDR

(Exact AN/PDF links are available from the BHI260AP product page under "Documents".)
