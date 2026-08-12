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

- The display is blanked (battery saving) and the watch stays awake so the
  BHI260AP step counter keeps running.
- Every **50 steps** the GNSS receiver is powered on once, a **3D fix** is
  awaited (VRTC backup makes this a warm/hot start), the position is recorded,
  and the receiver is powered back off.
- The haversine distance between consecutive fixes is accumulated; the
  last-known position is updated on every fix.
- **Lifetime totals** (distance and steps) are persisted in NVS across reboots;
  the average step length = distance / steps.
- Pressing **PWR** or **BOOT** while tracking restores the display so the Stop
  button can be reached.

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

## Architecture: background tasks + cached sensors

Sensors and slow I2C peripherals are handled by dedicated background tasks so
the UI never blocks on the I2C bus:

- **BHI260AP** (accel/gyro/RV/steps/activity) — own `bhi260` task polls the FIFO;
  the UI reads cached RAM values.
- **M10Q GNSS** — own UART RX task parses fixes; the UI reads cached RAM values.
- **AXP2101 PMU + PCF85063A RTC** — a low-priority `sensor_cache` task polls
  them once per second into a RAM struct; the watch face and power screen read
  this snapshot instead of doing I2C reads. `cachedump` shows the cache.

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
| `lpk` | Show the persisted last-known position (degrees). |
| `track` | Start a step-gated tracking session (blanks the display). |
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
- **Open point (future):** the GNSS receiver's PPS output (GPIO13) could be
  used to discipline the RTC (PCF85063A) to UTC with ~30 ns accuracy whenever
  a GNSS fix is available. Not implemented yet.
