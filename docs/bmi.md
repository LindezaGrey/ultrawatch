# BHI260AP — Findings & Integration Notes

Notes from bringing up the Bosch **BHI260AP** smart sensor hub on the
T-Watch Ultra. Verified on hardware unless marked otherwise.

## Part & interface

| Item | Value |
|---|---|
| Part | Bosch BHI260AP (6-axis IMU + programmable Fuser2 MCU) |
| I2C address | `0x28` |
| INT pin | ESP32 **GPIO8** (`TWATCH_PIN_IMU_INT`) |
| Power rail | AXP2101 **ALDO4** (sensor) |
| Firmware | `/assets/bhi260/BHI260AP.fw`, 106,156 bytes (SPIFFS) |
| Product ID | `0x89` (expected `0x89`) |
| Kernel version | 5991 |

## INT line polarity (important)

The BHI260AP host interrupt on this board is **active-LOW**: it idles high and
pulses low on an event. Verified with the `imon` debug command (GPIO8 level
monitor). The default-firmware description in the datasheet says active-high,
but the loaded LilyGO image inverts it — so:

- Awake-time ISR: `GPIO_INTR_NEGEDGE`.
- Light-sleep wake: `GPIO_INTR_LOW_LEVEL` (during sleep the FIFO is not
  drained, so the line holds low until wake).

## Power consumption (official, Bosch datasheet BST-BHI260AP-DS000, 1.8 V)

Per-sensor typical current (Fuser2 Long Run):

| Virtual sensor | ODR | Current |
|---|---|---|
| Game Rotation Vector (acc+gyro) | 25 Hz | 1.068 mA |
| Rotation Vector (9-DoF w/ mag) | 25 Hz | 1.126 mA |
| Geomagnetic Rotation Vector (acc+mag) | 25 Hz | 0.213 mA |
| Step counter / step detector | — | 0.098 mA |
| Activity recognition | — | 0.098 mA |
| Wake-up gesture | — | 0.261 mA |
| Glance / pickup gesture | — | 0.094 mA |
| Tilt / significant motion | — | 0.037 mA |

System states:

| State | Current |
|---|---|
| Gyro+accel suspend, Fuser2 deep sleep | 7.3 µA |
| Accel LP @25 Hz, gyro suspend, deep sleep | 12 µA |
| Accel full, gyro suspend, deep sleep | 184 µA |
| Accel+gyro full, deep sleep | 929 µA |
| Standby (product page) | 8 µA |

Keeping **ALDO4 powered during light sleep** costs roughly **0.1–0.3 mA**
(a wake-up gesture sensor) versus ~0 µA with the rail off, but avoids the
firmware re-upload and the sensor-freeze problem after a power-cycle.

## Enabled virtual sensors

| Sensor | Sample rate | Purpose |
|---|---|---|
| STC (step counter) | 1 Hz | Steps (persistent) |
| ACC | 12.5 Hz | Acceleration (mg) |
| GYRO | 12.5 Hz | Angular rate (dps) |
| GAMERV | 5 Hz | Game rotation vector (6-DoF quaternion) |
| AR | 5 Hz | Activity recognition |
| WRIST_TILT / WAKE / GLANCE / PICKUP | 1 Hz | Gestures |
| TILT_DETECTOR | 1 Hz | Tilt detector |

## Orientation & rotation (no magnetometer)

The BHI260AP is a **6-DoF IMU** (accel+gyro); the full 9-DoF Rotation Vector /
Orientation fusion sensors require an **external magnetometer**, which the
T-Watch Ultra does not have. Without it they report all-zero with accuracy 0
(see [arduino/nicla-sense-me-fw#102](https://github.com/arduino/nicla-sense-me-fw/issues/102)
for the same family behavior on a different sensor).

So the driver uses:

- **GAMERV (game rotation vector)** — 6-DoF accel+gyro fusion, works without a
  magnetometer; drives the `RV` row (quaternion x/y/z/w).
- **Pitch/roll computed from the accelerometer** (`atan2` on the gravity
  vector); heading is reported as `0` (magnetometer-only).

Gestures are natively wake-type; the `*_WU` sensor variants (ACC_WU, STC_WU,
…) also exist and could replace the non-wake streams during AP-suspend.

## AP-suspend wake architecture

The chip supports an **Application-Processor Suspend mode**: writing the
`AP_SUSPENDED` bit (`BHY2_HIF_CTRL_AP_SUSPENDED`) tells it to stop the
high-rate non-wakeup streams and run only the wake-up sensors, which assert
the host interrupt. This is the mechanism used for wrist-raise wake.

Sequence on sleep (`power_mgmt_enter_sleep`):

1. `co5300_blank()` + `co5300_sleep()` — clear GRAM, panel SLPIN.
2. `bhi260ap_ap_suspend()` — flush FIFO (`bhy2_flush_fifo(0xFF)`), then set the
   AP-suspend bit. The FIFO flush prevents a stale event from immediately
   re-waking the host.
3. **ALDO4 stays on**; the other peripheral rails are powered off.
4. GPIO8 armed as `LOW_LEVEL` light-sleep wake (disabled in night mode).

On wake (`power_mgmt_exit_sleep`):

1. `bhi260ap_ap_resume()` — clear the AP-suspend bit.
2. `co5300_wake()` + brightness, then `lvgl_force_redraw()` (the SPI flush path
   does not auto-refresh on resume, and the GRAM was blanked).

### Wake path wiring

- GPIO8 ISR (`GPIO_INTR_NEGEDGE`) → `pm_wake_task` → `esp_lv_adapter_request_wake()`.
  This mirrors the proven button path; the adapter's `_from_isr()` wake API
  crashed in this codebase, so the task-notify route is used instead.
- The adapter must receive `request_wake()` to leave PAUSE auto-sleep; a bare
  `esp_sleep_enable_gpio_wakeup()` on GPIO8 only wakes the CPU, which is why
  gesture wake needed its own ISR.

### Two gotchas found while debugging

1. **Wake-up FIFO watermark defaults to 0** (never asserts the interrupt).
   `bhy2_set_fifo_wmark_wkup(4, ...)` is required for gesture wake. This was
   the root cause of "wrist-raise doesn't wake".
2. **Do not re-init while suspended.** The sensor task's stale-data watchdog
   would deinit + re-init the chip during AP-suspend (no data flows by
   design), which leaves AP-suspend and floods the wake task with gesture
   events. The task now skips polling and stale-checking while
   `bhi260ap_is_suspended()`.

## Gestures

| Sensor | Data | Meaning |
|---|---|---|
| WAKE_GESTURE | event | Device picked up / raised |
| GLANCE_GESTURE | event | Quick glance at the watch |
| PICKUP_GESTURE | event | Lifted from a surface |
| WRIST_TILT_GESTURE | event | Raised wrist (flick) |
| TILT_DETECTOR | event | Device tilted (general) |

Activity classes: still, walking, running, cycling (on-bicycle), in vehicle,
tilting.

## Debug

- Console `bhi` — dump all sensor values.
- Console `suspend` / `resume` — toggle AP-suspend manually.
- Console `imon` — watch GPIO8 level for ~30 s (INT polarity / gesture checks).

## Step counter persistence

The on-chip counter resets on every RAM-firmware upload (each boot), so the
driver keeps a **NVS-stored base offset** (`bhi260ap`/`step_base`). The
reported total is `base + (chip - chip_at_last_fold)`.

- Every ~100 chip steps the delta is folded into the base and committed to NVS.
- Steps are also folded + saved on AP-suspend (each sleep).
- A chip-counter rollback (firmware re-upload / rail power-cycle) is detected
  and folded so the total stays monotonic.
- At most `STEP_FOLD_STEP` (100) steps can be lost on sudden power loss
  between folds. `nvs_flash_init()` runs in `app_main`.

## Limitations / future

- Heading is not available (no magnetometer on the board). Adding a BMM/QMC
  magnetometer to the BHI260AP's secondary I2C would enable 9-DoF RV/ORI.
- True sleep-wake from IMU relies on ALDO4 staying powered; battery-life impact
  is the ~0.1–0.3 mA gesture sensor.
