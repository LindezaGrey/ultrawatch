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

## GPS power management

The GNSS receiver (u-blox MIA-M10Q) is the most power-hungry peripheral
(~8–14 mA when acquiring, ~5 mA tracking), so it is **powered off by default**
and only switched on while the **GPS screen is open**. Leaving the screen (or
the 5 s menu timeout) powers it back off. GNSS power transitions run on a
background task, so the UI never stalls during the seconds-long receiver
probe/config.

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
| `suspend` / `resume` | Manually toggle the BHI260AP AP-suspend mode (debug). |
| `imon` | Watch the BHI260AP INT line (GPIO8) for 30 s (debug). |
| `motor` | Play a single haptic buzz. |
| `motor cal` | Run the DRV2605 on-chip auto-calibration (~1 s buzz) and save it to NVS. |
| `motor calrestore` | Reload the stored haptic calibration from NVS. |

## Data storage (SD card)

When an SD card is present it is mounted at `/sdcard`:

- **Logs** — all log output is buffered in RAM and flushed to
  `/sdcard/log/uwatch.log` every 2 s.
- **Screenshots** — `shot` saves PNG files to `/sdcard/shot/`.

## Notes

- A long press on **PWR** powers the device off (AXP2101 PEK behaviour).
- The BHI260AP firmware is uploaded to RAM on every boot from the SPIFFS
  assets partition.
- **Open point (future):** the GNSS receiver's PPS output (GPIO13) could be
  used to discipline the RTC (PCF85063A) to UTC with ~30 ns accuracy whenever
  a GNSS fix is available. Not implemented yet.
