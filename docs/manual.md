# UWatch — User Manual

UWatch is a smartwatch firmware for the LilyGO T-Watch Ultra (ESP32-S3).
This manual describes the current user-facing behaviour.

## Screens

The **clock (watch face)** is the main screen shown after boot. From it you
swipe in a direction to open a menu, and swipe back the opposite way to return.

| Screen | Enter from clock | Return |
|---|---|---|
| Clock (watch face) | — (main) | — |
| Power Management | swipe **down** | swipe **up** |
| BHI260AP status | swipe **left** | swipe **right** |
| GPS | swipe **right** | swipe **left** |

```
          swipe up
             ^
             |
 swipe left <- Clock -> swipe right
             |
             v
          swipe down
```

- **Power Management** — battery level (%), battery voltage (V), charging
  state, battery temperature (°C), a charging enable switch, and a charge
  current selector (100 mA / 400 mA presets, 100 mA default).
- **BHI260AP status** — IMU status (ready / not ready), step counter, accel,
  gyro, orientation (pitch/roll from the accelerometer), rotation quaternion
  (6-DoF game rotation vector), activity, and last gesture.
- **GPS** — satellite **skyplot** (position by azimuth/elevation, color by
  signal strength; filled dot = used in fix) plus position, altitude, speed
  (km/h), course, satellite count and estimated accuracy (HDOP-based).

## Watch face GNSS indicator

The clock shows a small **satellite icon** in the top centre whose colour
reflects the GNSS receiver state:

| Colour | Meaning |
|---|---|
| Grey | GNSS off (normal idle state) |
| Red | GNSS on, acquiring / no fix yet |
| Green | 3D fix obtained |

It turns red while the boot-time position check runs, green once a 3D fix
locks (briefly, before the receiver is powered back off), and returns to grey
when idle.

## Distance tracking

The GPS screen has a **Start**/**Stop** button (bottom centre) plus a stats
line showing the tracked distance, step count and average step length. Tracking
can also be controlled from the console (`track`, `track stop`).

While a tracking session is active:

- The watch face shows a **solid red dot** in the top-right corner; the rest of
  the screen keeps working normally (you can swipe between screens).
- The watch stays awake so the BHI260AP step counter keeps running.
- Every **50 steps** the GNSS receiver is powered on once, a **3D fix** is
  awaited (VRTC backup makes this a warm/hot start), the position is recorded,
  and the receiver is powered back off.
- The haversine distance between consecutive fixes is accumulated; the
  last-known position is updated on every fix.
- **Lifetime totals** (distance and steps) are persisted in NVS across reboots;
  the average step length = distance / steps.

No 3D fix indoors means no distance is accumulated for that interval (the step
threshold simply re-triggers later).

## GPS power management

The GNSS receiver (u-blox MIA-M10Q) is the most power-hungry peripheral
(~8–14 mA when acquiring, ~5 mA tracking), so it is **powered off by default**
and only switched on while the **GPS screen is open**. Leaving the screen (or
the 5 s menu timeout) powers it back off. GNSS power transitions run on a
background task, so the UI never stalls during the seconds-long receiver
probe/config.

**On power-up** the watch runs a one-shot GNSS position check in the
background: it powers the receiver, waits for a **3D fix** (up to 120 s; the
VRTC backup normally makes it a warm/hot start in a few seconds), updates the
persisted last-known position if the new fix is **more than 50 m** away from
the stored one, then powers the receiver back off.

Despite being unpowered, the receiver's always-on **VRTC backup rail** keeps its
RTC and ephemeris alive (~28 µA), so the next power-up is a **warm/hot start**
(~1–5 s to a fix) instead of a ~25 s cold start.

If the watch auto-sleeps while the GPS screen is open, the GNSS rail is cut
gracefully and re-powered (with a fresh config) on wake.

The skyplot and fix data update once per second while the screen is open.

## Auto-return

Any menu returns to the clock automatically after **5 seconds** without a
touch. Touching the screen resets the timer.

## Wake / sleep

- The watch auto-sleeps after ~5 s idle (display off, low power).
- **Wake:** touch the screen, or press the **PWR** or **BOOT** button.
- While charging / on USB power, the watch stays awake (no auto-sleep).
- **Night mode:** between 23:00 and 07:00 the display shows a dim red watch
  face, and touch is disabled as a wake source (PWR/BOOT still wake).

## Time source

The displayed time is read from the on-board RTC (PCF85063A) every second, so
it never drifts from the ESP32's internal clock.

On the first GNSS fix the RTC is set from GPS UTC (converted to local time via
the configured timezone). While a fix is present, the receiver's 1PPS output
(GPIO13) is used to measure the RTC's drift and write a correction to the
PCF85063A **OFFSET** register (step 4.34 ppm/LSB, ±64 ppm). The offset is
persisted in NVS and re-applied on every boot, so the RTC keeps GPS accuracy
even in the blind. `rtccal` re-runs the calibration manually (needs a fix).

## Architecture: background tasks + cached sensors

Sensors and slow I2C peripherals are handled by dedicated background tasks so
the UI never blocks on the I2C bus:

- **BHI260AP** (accel/gyro/RV/steps/activity) — own `bhi260` task polls the FIFO;
  the UI reads cached RAM values.
- **M10Q GNSS** — own UART RX task parses fixes; the UI reads cached RAM values.
- **AXP2101 PMU + PCF85063A RTC** — a low-priority `sensor_cache` task polls
  them once per second into a RAM struct; the watch face and power screen read
  this snapshot instead of doing I2C reads. `cachedump` shows the cache.
- **Audio** — MAX98357A amp on **I2S1** (STD TX) and T3902 PDM mic on **I2S0**
  (PDM RX), 16 kHz mono 16-bit. Two separate I2S controllers are used because
  the mode register is per-controller (STD and PDM cannot share one). The amp
  rail (BLDO2) is on by default; the MAX98357A has no software volume (fixed
  gain), so sample scaling is done in software.
- **BLE** — NimBLE peripheral advertising as **UWatch** (service `0xDEAD`)
  for wireless debugging. Connects with any BLE client (e.g. `bleak` on a PC).

## BLE debug bridge

The watch advertises as **UWatch** with a GATT service `0xDEAD`:

| Characteristic | UUID | Purpose |
|---|---|---|
| CMD | `0xDE01` | write-only: accepts any console command |
| RESP | `0xDE02` | read + notify: command output is echoed here |
| TELEM | `0xDE03` | read + notify: steps + GNSS + tracking snapshot, ~1/s |

To read telemetry with Python (`pip install bleak`):

```python
from bleak import BleakClient
import asyncio
async def main():
    c = BleakClient("10:51:DB:40:4F:16")
    await c.connect()
    c.start_notify("0000de03-0000-1000-8000-00805f9b34fb",
                   lambda u, d: print(d.decode()))
    await asyncio.sleep(5)
asyncio.run(main())
```

This lets you watch the **step counter, GNSS state and tracking totals**
wirelessly while walking. Note: the display draw buffer is smaller than
default to leave internal DMA memory for the Bluetooth controller.

## Debug

While connected over USB-Serial-JTAG, the following console commands are
available (type them and press Enter):

| Command | Effect |
|---|---|
| `shot` | Save the current screen as a PNG to the SD card (`/sdcard/shot/`); falls back to streaming raw RGB565 over USB if no card. |
| `sdin` | Print the contents of `/sdcard/log/uwatch.log`. |
| `sdls` | List screenshot files on the SD card. |
| `bhi` | Dump all BHI260AP sensor values. |
| `gnss` | Dump GNSS state, fix (position/speed/sats/accuracy) and per-satellite azimuth/elevation/SNR. |
| `gpscheck` | Re-run the one-shot GNSS position check (background, like on power-up). |
| `rtccal` | Re-run the PPS-based RTC drift calibration now (needs a GNSS fix; ~120 s window). |
| `lpk` | Show the persisted last-known position (degrees). |
| `track` | Start a step-gated tracking session (shows a red dot on the watch face). |
| `track stop` | Stop tracking and persist the session totals. |
| `trackstat` | Show tracking state, distance, steps and average step length. |
| `cachedump` | Dump the cached PMU/RTC telemetry (battery, charge, temperature, time). |
| `suspend` / `resume` | Manually toggle the BHI260AP AP-suspend mode (debug). |
| `imon` | Watch the BHI260AP INT line (GPIO8) for 30 s (debug). |
| `motor` | Play a single haptic buzz. |
| `motor cal` | Run the DRV2605 on-chip auto-calibration (~1 s buzz) and save it to NVS. |
| `motor calrestore` | Reload the stored haptic calibration from NVS. |
| `crashinfo` | Show whether a crash core dump is pending in flash. |
| `crashsave` | Force the pending core dump to be decoded to the SD card now. |
| `crashls` | List saved crash reports / core dumps on the SD card. |
| `crashread <file>` | Print a saved crash report from the SD card. |
| `panictest` | Deliberately crash (null deref) to exercise the core dump feature. |
| `tone [hz] [ms] [amp]` | Play a sine tone on the speaker (default 440 Hz / 500 ms / near-max volume). |
| `sweep [f0] [f1] [ms]` | Play a log-frequency sweep (default 200→8000 Hz, 3 s) while the mic records it; reports the recorded peak. |
| `rec [ms]` | Record mono 16 kHz audio from the microphone (default 2 s, max 10 s); reports the peak amplitude. |
| `playrec` | Play the last recording back through the speaker (loops 3×). |

## Data storage (SD card)

When an SD card is present it is mounted at `/sdcard`:

- **Logs** — all log output is buffered in RAM and flushed to
  `/sdcard/log/uwatch.log` every 2 s.
- **Screenshots** — `shot` saves PNG files to `/sdcard/shot/`.
- **Crash dumps** — if the watch crashes, the ESP32 core dump is stored to a
  flash partition and, on the next boot with an SD card present, decoded to
  `/sdcard/log/crash/`:
  - `report_<timestamp>.txt` — human-readable crash report (panic reason,
    crashed task, exception PC/cause/vaddr, register dump, backtrace).
  - `core_<timestamp>.elf` — the raw core dump, analyzable offline with
    `espcoredump.py info_corefile -m build/UWatch.elf -c core_<timestamp>.elf`.
  - The flash copy is erased only after a successful save, so the dump survives
    if no card is present on the first boot after the crash.

## Notes

- A long press on **PWR** powers the device off (AXP2101 PEK behaviour).
- The BHI260AP firmware is uploaded to RAM on every boot from the SPIFFS
  assets partition.
- The RTC (PCF85063A) is synced from GPS UTC on the first fix and disciplined
  via the GNSS 1PPS (GPIO13) into the OFFSET register (see "Time source").
