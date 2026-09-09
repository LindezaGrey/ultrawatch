# UWatch — User Manual

UWatch is a smartwatch firmware for the LilyGO T-Watch Ultra (ESP32-S3).
This manual describes the current user-facing behaviour.

## Screens

The **clock (watch face)** is the main screen shown after boot.

| Screen | Navigation |
|---|---|
| Activity | Swipe left from Watch |
| GPS | Swipe left from Activity |
| Mesh | Swipe left from GPS; left again returns to Watch |
| Apps | Swipe up from Watch; swipe down returns |
| Alarms/Timer, NFC, Wi-Fi, Bluetooth | Tap in Apps; swipe right returns to Apps |
| Settings | Swipe down from Watch; swipe up returns |
| Display settings | Hold Watch, or open Settings → Display |
| Node overview | Swipe up from Mesh |

The horizontal ring wraps in both directions: Watch ↔ Activity ↔ GPS ↔ Mesh ↔
Watch. Swipe right or use a Back button on subpages to return to their parent.

- **NFC** — tap **Start** to scan an NFC-A tag. The reader is active only for
  the explicit scan, so simply visiting the screen has no radio activity.
- **Settings** — display, power, peripheral, sound, information, and vibration
  preferences. It sits outside the horizontal ring to avoid accidental changes.
- **BHI260AP status** — an IMU diagnostic screen reached through debug tools;
  it is not part of normal swipe navigation.
- **GPS** — satellite **skyplot** (position by azimuth/elevation, color by
  signal strength; filled dot = used in fix) plus position, altitude, speed
  (km/h), course, satellite count and estimated accuracy (HDOP-based).

## Watch face

The clock shows the local time as **HH:MM:SS** (large), with **UTC HH:MM**
below it and the date above. A **satellite icon** in the top centre reflects
the GNSS receiver state:

| Colour | Meaning |
|---|---|
| Grey | GNSS off (normal idle state) |
| Red | GNSS on, acquiring / no fix yet |
| Green | 3D fix obtained |

It turns red while the boot-time position check runs, green once a 3D fix
locks (briefly, before the receiver is powered back off), and returns to grey
when idle.

## Distance tracking

Tracking is currently disabled by `TRACKING_ENABLED=0`. The following describes
the background path when enabled.

The GPS screen has a **Start**/**Stop** button (bottom centre) plus a stats
line showing the tracked distance, step count and average step length. Tracking
can also be controlled from the console (`track`, `track stop`).

While a tracking session is active:

- The watch face shows a **solid red dot** in the top-right corner; the rest of
  the screen keeps working normally (you can swipe between screens).
- The display may sleep while the BHI260AP step counter continues running.
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

The GPS switch (or `gnsson` / `gpscheck` console command) starts a receiver
session. Power-on/configuration runs in the background. Once ready, the
receiver gets **60 seconds with weak or missing reception**. Five distinct
satellite reports smooth the signal estimate: an average of at least four
healthy, code-locked satellites at 30 dBHz extends acquisition; it falls back
to weak reception below four at 25 dBHz. A fresh valid position renews the
budget. Even with strong signals, no-fix acquisition is capped at **180 seconds**.
These initial thresholds need verification on the watch outdoors and indoors.

On timeout the worker saves GPS as off. The page reports **Poor signal**,
**No data**, or **No fix - GPS off**. Turn the switch on again, or run
`gnsson` / `gpscheck`,
to retry. Screen navigation, screen wake and GPX logging do not restart GPS.
An active GPX session remains open and resumes adding points after GPS is
manually re-enabled and has a fix.

This is an indoor/poor-reception heuristic, not a definite indoor classification:
a blocked antenna or weak reception outdoors can also trigger it. Reading the
same cached fix repeatedly does not extend the budget. The monotonic clock
continues during display sleep. The VRTC backup supply is retained when GPS
powers off.

The GPS enabled preference survives reboot. A boot with GPS enabled uses the
same acquisition limit. After an automatic cutoff, it stays disabled across
reboots until explicitly enabled again.

## Auto-return

Read-only pages and app lists return to Watch after **30 seconds** without
interaction by default. Settings, alarm editing, timer entry and ringing are
exempt. Holding a touch keeps the inactivity timer fresh.

The return timer includes normal sleep: a quick wake resumes the page; a wake
after the deadline shows Watch. The timer never wakes the display by itself.
Unsaved editor state survives normal display sleep, but not a reboot/deep sleep.

## Wake / sleep

- The display sleeps after about **10 seconds** idle by default. Existing saved
  timeout values are retained.
- Display settings provide separate **Sleep** (5/10/15/20/30/60 s) and
  **Return home** (15/30/60/120 s) controls. Both apply immediately and persist.
- **Wake:** touch, wrist gesture, PWR, BOOT or an armed RTC alarm. Wrist and
  touch wake are disabled during night mode.
- USB prevents sleep by default; this can be disabled in Settings.
- GPS acquisition continues without keeping the display lit. Visible NFC scans
  and ringing may keep it awake. Tracking remains disabled by the existing
  build setting; its background path no longer requires a lit display.
- **Night mode:** 23:00–07:00 uses the dim red watch face.

## Time source

The displayed time is read from the on-board RTC (PCF85063A) every second, so
it never drifts from the ESP32's internal clock.

On the first GNSS fix the RTC is set directly from GPS UTC. Local display,
alarms, night mode, and local-date logs use the POSIX timezone selected by
`CONFIG_UWATCH_TIMEZONE` (CET/CEST by default). While a fix is present, the
receiver's 1PPS output (GPIO13) is used to measure the RTC's drift and write a
correction to the PCF85063A **OFFSET** register (step 4.34 ppm/LSB, ±64 ppm).
The offset is persisted in NVS and re-applied on every boot, so the RTC keeps
GPS accuracy even in the blind. `rtccal` re-runs the calibration manually
(needs a fix).

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
  rail (BLDO2) is normally off and is enabled only during playback; the
  MAX98357A has no software volume (fixed gain), so sample scaling is done in
  software.
- **BLE scanner** — NimBLE is initialized on demand from the Bluetooth screen,
  guarded by a free-DMA-memory check.

## BLE debug bridge (currently disabled)

The firmware contains an experimental GATT debug bridge, but it is not started
because its controller allocation conflicts with the display DMA buffer. The
following UUIDs describe that dormant implementation; the watch does **not**
currently advertise them:

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

If the bridge is re-enabled after its DMA conflict is resolved, it can expose
the **step counter, GNSS state and tracking totals** wirelessly.

## Debug

While connected over USB-Serial-JTAG, the following console commands are
available (type them and press Enter):

| Command | Effect |
|---|---|
| `shot` | Save the current screen as a PNG to the SD card (`/sdcard/shot/`); falls back to streaming raw RGB565 over USB if no card. |
| `sdls` | List screenshot files on the SD card. |
| `bhi` | Dump all BHI260AP sensor values. |
| `gnss` | Dump GNSS state, fix (position/speed/sats/accuracy) and per-satellite azimuth/elevation/SNR. |
| `gpscheck` | Start/restart GPS acquisition; signal-aware auto-off (60 s weak reception, 180 s acquisition cap). |
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

The filesystem is mounted on demand only for an active read or write and is
unmounted immediately afterward; waking the watch does not mount it. A seated
card's ALDO1 rail remains powered even while unmounted because an unpowered
card clamps the SPI2 MISO line used by LoRa and NFC. During an I/O session the
filesystem is mounted at `/sdcard`:

- **Screenshots** — `shot` saves PNG files to `/sdcard/shot/`.
- **Daily telemetry** — steps, activity and battery samples are appended to
  `/sdcard/log/steps.csv`, `activity.csv`, and `battery.csv`.
- **GPX tracks** — active GPX sessions write timestamped files under
  `/sdcard/log/gpx/`.
- **Mesh and system logs** — received mesh text is appended to
  `/sdcard/log/mesh.txt`; captured firmware logs go to `syslog.txt`.
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

GPS positions, satellite reports and HDOP expire after three seconds. The
skyplot retains the strongest 24 satellites, while reception statistics use
all reports. The screen distinguishes 2D/3D; unavailable altitude/course/HDOP
show as missing. GPX skips stale fixes and fixes without valid UTC, timestamps
points with receiver UTC, and omits altitude for 2D fixes. `gnssver` reads the
version cached at power-on without interrupting an active receiver session.

Mesh messages use a Latin Unicode font at 22 px, with an 18 px monochrome
emoji fallback. Node names and preset buttons use the compact Unicode font.
German umlauts (ÄÖÜäöü), ß and common emoji are included. This is bitmap glyph
support: complex joined emoji, skin-tone combinations and country flags are
not shaped into a single symbol. Existing message rows still clip long text.

Daily activity uses the local calendar date. Startup reconciles the saved date
before exposing daily counters: same-day boots restore steps and durations;
later boots queue the previous recorded day and start today. Powered-off days
and powered-off time are not invented as activity. While running, housekeeping
checks the date about once per second; rollover occurs on the first check
that observes the new date, so activity near the boundary has sampling latency.
Backward date changes are deferred rather than repeatedly resetting totals.

Steps, lifetime baseline and accumulated activity milliseconds are stored in
one NVS checkpoint each minute, at rollover, before orderly shutdown/deep sleep,
and before sensor reinitialization. Sudden power loss can lose the interval
since the last successful checkpoint. Old firmware's lost activity durations
cannot be recovered; existing step counters are migrated.

`/sdcard/log/activity_totals.csv` contains minute samples with date, time,
daily/lifetime steps and accumulated **still, walking, running, cycling,
vehicle, tilting and unknown seconds**. `/sdcard/log/daily/YYYYMMDD.csv` contains
one completed-day summary with the same columns (time is blank, since a summary
may be recovered after a missed midnight). Existing CSV formats are retained.

Completed summaries remain in the NVS checkpoint until the SD file has been
written and synchronized successfully. Retries overwrite that date's file,
so a restart between writing and acknowledging cannot duplicate summaries.
The queue holds 16 recorded days without SD access; when full, rollover is
blocked and daily counters are unavailable until summaries can be drained.
Minute samples missed without a card are not replayed; accumulated totals are
retained. Summary seconds are whole seconds; checkpoints retain milliseconds.

Settings → Info distinguishes SD **Inserted - idle**, **Active**, **No card**
and a recorded **SD error**. Capacity, used space and free space show the last
successful measurement this boot, labeled “Last known”; they remain visible
while unmounted or absent. No measurement is displayed until a filesystem
session has completed. Replacing the card updates these values after its next
successful session. Viewing Info does not mount the card. The main SD icon is
green for inserted/idle, red for mounted, grey for absent and orange for a
recorded mount/filesystem error.
