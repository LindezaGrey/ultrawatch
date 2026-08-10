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
  current slider (0–500 mA).
- **BHI260AP status** — IMU status (ready / not ready) and the step counter.
- **GPS** — placeholder (GNSS receiver not integrated yet).

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

## Data storage (SD card)

When an SD card is present it is mounted at `/sdcard`:

- **Logs** — all log output is buffered in RAM and flushed to
  `/sdcard/log/uwatch.log` every 2 s.
- **Screenshots** — `shot` saves PNG files to `/sdcard/shot/`.

## Notes

- A long press on **PWR** powers the device off (AXP2101 PEK behaviour).
- The BHI260AP firmware is uploaded to RAM on every boot from the SPIFFS
  assets partition.
