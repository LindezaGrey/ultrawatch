# T-Watch Ultra sensors over BLE

Runs a small Watch/Launcher/Settings window manager in Cascadia Code inside the
validated safe-area contour. It exposes the clock, power management, Wi-Fi,
BHI260AP orientation, and MIA-M10Q GPS data over Bluetooth Low Energy.

- Device name: `UltraWatch`
- Service: `7a1e0001-7a1e-4b6c-8d9e-001122334455`
- RTC: `7a1e0002-7a1e-4b6c-8d9e-001122334455`, read/write/notify each minute,
  payload `YYYY-MM-DDTHH:MM:SS`
- Battery/power: `7a1e0003-7a1e-4b6c-8d9e-001122334455`, read/notify every five
  seconds, payload `percentage,millivolts,direction,vbus,present`
- Charge configuration: `7a1e0004-7a1e-4b6c-8d9e-001122334455`, read/write.
  Reads return `charge_milliamps,input_milliamps,charge_millivolts,enabled`;
  writes accept `charge_milliamps,input_milliamps,enabled`.
- IMU orientation: `7a1e0005-7a1e-4b6c-8d9e-001122334455`, read/notify at
  25 Hz. Its 10-byte little-endian payload contains signed 16-bit
  `x,y,z,w,accuracy` values. Divide every value by 16384; accuracy is in
  radians. The game rotation vector uses accelerometer and gyroscope fusion,
  so inclination is absolute while heading is relative and may drift.
  The browser converts the BHI coordinate system (`Z` out of the display) to
  the Three.js scene coordinate system (`Y` up), then applies a fixed 90°
  clockwise preview offset before rotating the model.
- Screen brightness: `7a1e0006-7a1e-4b6c-8d9e-001122334455`, read/write. The
  payload is one unsigned byte from 0 through 100 percent. The firmware maps
  it to the AMOLED controller's 8-bit DCS brightness command; 0% leaves the
  display and BLE active but renders the AMOLED pixels black.
- CST9217 touch: `7a1e0007-7a1e-4b6c-8d9e-001122334455`, read/notify on touch
  transitions and coordinate changes. Its 6-byte little-endian payload is
  `pressed,event,x,y`: one byte each for `pressed` and `event`, followed by
  unsigned 16-bit X and Y coordinates. Event values are `0` idle, `1` down,
  `2` move, and `3` up. Coordinates use the physical 410 x 502 display space.
  The browser renders contacts on a Three.js watch face and pulses a ring for
  every down event.
- MIA-M10Q GPS: `7a1e0008-7a1e-4b6c-8d9e-001122334455`, read/notify once per
  second. Its 30-byte little-endian payload is `ready` (u8), `fix_valid` (u8),
  `fix_type` (u8), `satellites` (u8), `latitude_e7` (i32), `longitude_e7`
  (i32), `altitude_msl_mm` (i32), `horizontal_accuracy_mm` (u32),
  `ground_speed_mm_s` (u32), `heading_e5` (i32), and `rf_agc` (u16).
- DRV2605 haptics: `7a1e0009-7a1e-4b6c-8d9e-001122334455`, read/write. Write
  two bytes, `effect,repeats`, to play a ROM Library A ERM effect. Effect IDs
  are 1 through 123 and repeats are limited to 1 through 4. Write `0,0` to
  stop immediately. The 6-byte status payload is `ready,device_id,playing,`
  `last_effect,last_repeats,faults`; fault bits report over-current,
  over-temperature, feedback timeout, and diagnostic failure.
- Sensor power: `7a1e000a-7a1e-4b6c-8d9e-001122334455`, read/write/notify.
  Reads return two bytes, `requested,ready`; writes accept the one-byte
  `requested` mask. Bit 0 controls IMU, bit 1 GPS, and bit 2 touch. The
  requested mask is stored in NVS and restored after restart. A watch without
  a stored preference starts with touch enabled and IMU/GPS disabled so
  automatic light sleep is effective by default.
- ESP32 power: `7a1e000b-7a1e-4b6c-8d9e-001122334455`, read/write. The
  three-byte little-endian payload is `maximum_cpu_mhz` (u16), followed by
  `automatic_light_sleep` (u8). Supported maximum frequencies are 80, 160,
  and 240 MHz; the idle frequency is the ESP32-S3 crystal frequency of 40 MHz.
  Settings are stored in NVS. The first boot defaults to 160 MHz with automatic
  light sleep enabled.
- Wi-Fi status: `7a1e000e-7a1e-4b6c-8d9e-001122334455`, read/write/notify. A
  one-byte write uses `0` for Stop and `1` for Start or Retry. The 12-byte value
  is `requested,state,error,rssi,ipv4[4],active_profile_u32_le`. Unknown RSSI is
  `127`; no active profile is `0xffffffff`.
- Wi-Fi profiles: `7a1e000f-7a1e-4b6c-8d9e-001122334455`, write/indicate. Each
  20-byte frame is `operation,request_id,result_or_flags,data_length,`
  `profile_index_u32_le,data_offset_u16_le,data[10]`. Operations are Count `1`,
  Get `2`, Put `3`, Delete `4`, and Move `5`. The browser waits for each
  indication before it sends the next frame.

Power direction values are `0` standby, `1` charging, `2` discharging, and `3`
reserved/unknown. `vbus` and `present` are `0` or `1`.

Charge-current writes are restricted to documented AXP2101 steps from 25 mA
through the board-recommended 500 mA maximum. Input-current writes accept 100,
500, 900, 1000, 1500, or 2000 mA. The target charge voltage is intentionally
read-only. Settings are read back from the PMIC after every write and are not
persisted across a PMIC power-on reset.

The RTC stores local calendar time only; the payload has no timezone or UTC
offset. Its hardware year range is represented here as 2000 through 2099. This
minimal iteration does not require BLE pairing or an encrypted connection.

The display path runs first at startup: after RTC and panel setup, the UI task
renders the Watch face with `HH:MM`, the PCF85063A weekday/date, battery charge,
BLE state, and launcher control before it reads SD media or starts the remaining
sensor state. It sleeps until the next minute boundary instead of repainting
seconds. NVS, PMIC measurement, haptics, BLE, and enabled sensor initialization
follow. This keeps SD, IMU firmware loading, and GPS probing outside the
first-Watch-frame path.

The window manager has Watch, Launcher, Settings, Alarm, Map, Weather, and Black
states. Watch is the boot/default app. The launcher clock, settings, alarm, map,
and weather bubbles open their corresponding screens; the other app bubbles are visual
placeholders and deliberately inert. Settings changes AMOLED brightness continuously while
dragging and uses the same public advertising setter as the physical side
button. After the 10-second inactivity interval, the active app resets to Watch
and the display becomes completely black. The waking finger is consumed through
its release, so it cannot also activate a Watch control.

After 10 seconds without a CST9217 touch interrupt, the firmware writes black
pixels across the complete 410 x 502 framebuffer and stops display refreshes.
It does not send the AMOLED sleep/display-off commands and does not switch off
the panel power rail. A touch interrupt first restores the complete calibrated
frame and blue contour, then redraws the clock, charge, and BLE state and starts
a new 10-second inactivity period. Other events, including the side button and
BLE writes, do not wake the black screen. Touch must remain enabled in the
sensor controls for touch-to-wake to be available.

The physical side button is the AXP2101 PWRON key, not a direct ESP32 GPIO.
Its events arrive through the PMIC interrupt line on GPIO 7. A completed button
click toggles BLE advertising on the AXP2101 PWRON release event and redraws the
display as `BLE AN` or `BLE AUS`; the separate long-press event suppresses that
toggle so the AXP2101's hardware power-key behavior is retained. Disabling
advertising also terminates an established connection. With advertising
disabled, disconnect and advertising-complete events do not restart advertising.

## Wi-Fi profiles

Wi-Fi uses station mode and is off after each restart. Select **Connect** in the
web page, then use the Wi-Fi section to add networks and start Wi-Fi. The watch
loads profiles in their saved order and stops when one connection succeeds. If
one complete pass fails, use **Retry** to start a new pass.

Profiles are UTF-8 JSON in `/ultrawatch/config.txt` on the SD card:

```json
{
  "version": 1,
  "openweathermap_api_key": "",
  "networks": [
    {
      "ssid": "Example",
      "password": "example-password"
    }
  ]
}
```

An SSID is 1 through 32 UTF-8 bytes. A password is empty for an open network,
8 through 63 UTF-8 bytes, or one 64-character hexadecimal key. Duplicate SSIDs
and unsupported file versions are rejected. Updates use `config.tmp` and
`config.bak`; a valid backup is recovered if the main file is missing or
invalid. The firmware does not format the card.

This development interface stores passwords as plain text and transfers them
through an unauthenticated BLE connection. Do not use production credentials.
Wi-Fi uses RAM-only ESP-IDF station configuration and modem power saving. A
Stop command disconnects, stops, and deinitializes the Wi-Fi driver.
The on-watch Settings screen has a Wi-Fi switch. Opening Weather also starts
Wi-Fi when it is off; it does not restart an active connection.

## Weather

The Weather bubble opens a seven-day on-device forecast. Weather requires a
current GPS fix or a position that the watch saved from an earlier fix. The
watch stores a consumed live position in NVS. It uses that position until GPS
supplies a newer fix.

Add an OpenWeather One Call API 4.0 key to
`/ultrawatch/config.txt` on the SD card:

```json
"openweathermap_api_key": "YOUR_KEY"
```

Do not commit a real key. One Call API 4.0 requires its separate One Call by
Call subscription. When Wi-Fi is connected, the watch requests metric daily
data for the selected latitude and longitude. It shows the first seven daily
records. With Wi-Fi off, it uses
`/ultrawatch/weather/cache.json` only when that cache matches the selected
position. A failed or missing cache does not restart the watch.

## Storage inventory

NVS application data:

- `sensor_control/enabled`: `u8`, default `0x04`.
- `system_power/max_mhz`: `u16`, default `160`.
- `system_power/light_sleep`: `u8`, default `1`.
- `alarm/cfg_minimal`: alarm hour, minute, and enabled state. Default is
  `07:00`, disabled.
- `theme/main_rgb`: `u32`, default `0x1863FF`.
- `gps/last_pos`: versioned latitude and longitude from the last live fix that
  Weather consumed. No value exists until a valid fix is available.

NVS system data:

- ESP-IDF PHY calibration data can exist because PHY calibration storage is
  enabled.
- Wi-Fi credentials and station configuration do not use NVS.
- Current NVS recovery can erase the full NVS partition when it is full or has
  an incompatible version. Application preferences then return to defaults.

SD data:

- UI atlas.
- Offline map package.
- Plain-text `/ultrawatch/config.txt` Wi-Fi profiles.
- `/ultrawatch/config.txt` OpenWeather API key and
  `/ultrawatch/weather/cache.json` weather cache.

Hardware registers:

- PCF85063A calendar and active alarm registers.
- AXP2101 charge settings. These settings do not survive a PMIC power-on reset.

Volatile state:

- Display brightness.
- BLE advertising state and active BLE connection.
- Wi-Fi state.
- GPS lease.
- Sensor-ready state and current runtime errors.

Disabling IMU switches off AXP2101 ALDO4 after stopping its data path;
disabling GPS stops its UART/parser and switches off BLDO1; disabling touch
holds the CST9217 in reset. Re-enabling a sensor initializes it again. The web
page distinguishes the persisted requested state from the live ready state.

ESP-IDF dynamic frequency scaling, tickless idle, and Bluetooth modem sleep are
enabled. When no task or peripheral holds a power-management lock, the CPU can
scale down to 40 MHz and enter automatic light sleep without losing BLE or
application state. The CST9217 active-low interrupt on GPIO12 wakes the chip and
drives touch reads, replacing the former 20 ms polling loop. An enabled GPS
holds an ESP-IDF no-light-sleep lock so UART responses are not lost; disabling
unused sensors therefore remains important for runtime.

The BHI260AP is initialized with Bosch's BHI2xy SensorAPI v1.6.0 and its stock
RAM firmware. The vendored driver, firmware image, and BSD-3-Clause license are
under `firmware/main/vendor/bhy2/`. Firmware loading failures are logged without
restarting the watch, so RTC and power remain usable instead of causing a boot
loop.

The GPS module is powered from AXP2101 BLDO1 at 3.3 V and connected to UART1
with ESP32-S3 TX on GPIO 43 and RX on GPIO 44. When enabled, the firmware probes
38400, 115200, and 9600 baud for a u-blox `MIA-` identity, switches to 115200,
enables the documented constellations, and requests UBX-NAV-PVT output at 1 Hz.
The receiver configuration uses the RAM layer, so it is applied again after
each watch restart. RTC synchronization, MGA aiding, GNSS software backup, and
TTFF history are intentionally outside this first live-position iteration.

The haptic driver uses the shared I2C bus at address `0x5A`. XL9555 port 0 bit
6 asserts its `M_EN` pin during board startup. Firmware accepts both DRV2605
and DRV2605L device IDs, selects internal-trigger mode and ERM Library A, and
limits BLE commands to finite ROM effects. The browser provides several named
presets plus explicit read and stop controls.

The firmware embeds only the Cascadia Code glyphs derived from the static
Watch, German date, and Settings strings. They are generated from
`CascadiaCode-Regular.otf` with
`firmware/tools/generate_cascadia_font.py`; the font license is included beside
the generated header in `firmware/main/CASCADIA_CODE_LICENSE.txt`.

For example, regenerate the header from a Cascadia Code installation on macOS
(the Python environment must provide Pillow):

```sh
python3 firmware/tools/generate_cascadia_font.py \
  "$HOME/Library/Fonts/CascadiaCode-Regular.otf" \
  firmware/main/cascadia_code_72.h
```

## SD-card UI assets

Copy the contents of `firmware/sdcard/` to the root of a FAT32 card. The watch
expects `/ultrawatch/ui/icons.rgb565`, an exact 115,200-byte, row-major atlas of
twenty-five 48 x 48 RGB565 tiles in display byte order. The tile order is
launcher, clock, settings, activity, heart, sleep, wellness, weather, music,
messages, rings, BLE off, BLE on, and four battery states from empty to full.
The next four tiles are Map, zoom in, zoom out, and map center. The final four
tiles are Wi-Fi off, connecting, connected, and error. Wellness stays in the
atlas for compatibility, but the launcher replaces it with Map.
The ready-to-copy atlas, original generated/chroma-key PNGs,
alpha-normalized source PNGs, 48 x 48 tiles, and QA contact sheet are retained
under `firmware/`.
Firmware also accepts the previous 50,688-byte 11-tile, 78,336-byte 17-tile,
82,944-byte 18-tile, and 96,768-byte 21-tile atlases. Missing status, map,
map-control, and Wi-Fi symbols use procedural fallbacks.

The BLE, battery, and Wi-Fi variants are deterministic. Regenerate their
transparent source PNGs before rebuilding the atlas:

```sh
python3 firmware/tools/create_status_icons.py firmware/assets/ui/sources
```

To reproduce the atlas after generating new flat-background source art, first
remove each chroma key with the ImageGen helper, then run:

```sh
python3 firmware/tools/build_ui_assets.py \
  firmware/assets/ui/sources \
  firmware/sdcard/ultrawatch/ui/icons.rgb565 \
  firmware/assets/ui/contact-sheet.png \
  --tile-dir firmware/assets/ui/tiles
```

The build tool rejects non-transparent source corners, normalizes and preblends
each icon on black, writes big-endian RGB565 bytes, checks the required atlas
size, and produces the labeled contact sheet. Firmware renders the first Watch
frame before SD initialization, never formats media, caches a valid atlas in
internal RAM, unmounts the card, and disables AXP2101 ALDO1. Missing or invalid
media uses procedural launcher, clock, and settings symbols without rebooting.

The Weather app uses a separate 294,912-byte atlas at
`/ultrawatch/weather/images.rgb565`. It contains nine 128 x 128 pictures in the
OpenWeather order clear, few clouds, scattered clouds, broken clouds, shower
rain, rain, thunderstorm, snow, and mist. Build it with Pillow:

```sh
python3 firmware/tools/build_weather_assets.py \
  firmware/assets/weather/sources \
  firmware/sdcard/ultrawatch/weather/images.rgb565 \
  firmware/assets/weather/contact-sheet.png \
  --tile-dir firmware/assets/weather/tiles
```

The watch maps OpenWeather condition IDs to these nine documented icon groups.
If the weather atlas is absent or invalid, the Weather app uses its procedural
fallback symbols.

## Offline on-device map

The Map bubble at the left side of the launcher opens an offline, north-up map.
It uses the MIA-M10Q position but does not rotate with movement heading. Use the
four arrow controls at the upper-left to move the view by half a map tile in a
fixed direction.
A drag applies one pan when the finger is released, so slow rendering cannot
replay intermediate drag positions. Use the large zoom controls to change the
available archive zoom, and use the target control to resume live following.
The center launcher control returns to the app drawer. The app does not
calculate routes or instructions. An interactive attribution view is
intentionally not included in this iteration.

### Minimal map build

The map build uses the pinned basemap.de Hessen GeoPackage for map geometry and
the official BKG GN250 dataset for place names. The current Hessen GeoPackage
has an empty `name_punkt_bdlm` table, so it cannot supply city labels. The
pipeline does not download WMS or WMTS tiles. Source files and generated map
data stay under the ignored `local-map-data/` directory.

Download the pinned Hessen Basis-DLM GeoPackage with resume support:

```sh
mkdir -p local-map-data
curl --fail --location --continue-at - \
  --output local-map-data/basisviews_bdlm_HE_EPSG4326_2026-08-17.gpkg \
  https://basemap.de/dienste/opendata/basisviews/basisviews_bdlm_HE_EPSG4326_2026-08-17.gpkg

curl --fail --location --continue-at - \
  --output local-map-data/gn250.utm32s.shape.zip \
  https://daten.gdz.bkg.bund.de/produkte/sonstige/gn250/aktuell/gn250.utm32s.shape.zip

unzip -o local-map-data/gn250.utm32s.shape.zip \
  'gn250/GN250_p.*' -d local-map-data
```

The official QGIS image is currently published for AMD64 only. On an
Apple Silicon Mac, pull and run it through Docker's AMD64 emulation. Use the
pinned QGIS version instead of the moving `stable` tag:

```sh
docker pull --platform linux/amd64 qgis/qgis:3.44.12
```

The repository contains the versioned `firmware/maps/hessen-debug.qgs`
project. Its data-source path is relative to the project. Its minimal headless
render command is:

```sh
docker run --rm --platform linux/amd64 \
  -e QT_QPA_PLATFORM=offscreen \
  -v "$PWD:/work" -w /work \
  qgis/qgis:3.44.12 \
  qgis_process run native:tilesxyzmbtiles \
  --PROJECT_PATH=/work/firmware/maps/hessen-debug.qgs -- \
  'EXTENT=7.609011,10.388817,49.359391,51.749253 [EPSG:4326]' \
  ZOOM_MIN=6 ZOOM_MAX=15 DPI=96 TILE_FORMAT=1 QUALITY=75 \
  METATILESIZE=4 \
  OUTPUT_FILE=/work/local-map-data/hessen-raster.mbtiles
```

The checked-in QGIS project defines the source layers, draw order, and a
minimal dark debug style. Regenerate it after a GeoPackage filename or layer
change with:

```sh
docker run --rm --platform linux/amd64 \
  -e QT_QPA_PLATFORM=offscreen \
  -v "$PWD:/work" -w /work \
  qgis/qgis:3.44.12 \
  python3 firmware/tools/generate_hessen_qgis_project.py \
  local-map-data/basisviews_bdlm_HE_EPSG4326_2026-08-17.gpkg \
  firmware/maps/hessen-debug.qgs \
  --labels local-map-data/gn250/GN250_p.shp
```

The project shows GN250 municipality names at overview scales. At detailed
scales, it also shows locality names. Label collision handling and population
filters keep the overview readable.

The production pipeline will pin the QGIS image by digest and render separate
resumable zoom stages. Attribution is outside the current renderer-debug
iteration.

Convert the rendered raster MBTiles to the current watch package:

```sh
python3 firmware/tools/build_offline_map.py \
  local-map-data/hessen-raster.mbtiles \
  firmware/sdcard/ultrawatch/maps \
  --jpeg-quality 75 \
  --attribution DEBUG
```

The temporary `DEBUG` value satisfies the current package-format validator. It
is not production attribution. The tool accepts 256 x 256 PNG, JPEG, and WebP
raster tiles, converts TMS rows to XYZ, and rejects vector archives. It
produces `map.uwi`, `map.000`, optional additional `map.NNN` segments, and
`map.sha256`. ESP-IDF on this target has a signed 32-bit file seek, so segment
files split at that platform boundary before the larger FAT32 file limit.

Copy the generated files to `/ultrawatch/maps/` on the FAT32 card. Map data is
not included in this repository. The SD card and GPS are active only while Map
is open. The temporary GPS request does not change the saved BLE sensor mask.
Map keeps the screen active while it is open. Closing Map or an alarm unmounts
the card, disables ALDO1, and restores the previous GPS state. The normal
10-second screen timeout resumes after Map closes.

Build with ESP-IDF 5.3:

```sh
docker run --rm -v "$PWD/firmware:/project" -w /project espressif/idf:release-v5.3 idf.py -B build -D SDKCONFIG=build/sdkconfig build
```

Pushing any Git tag runs `.github/workflows/release-firmware.yml`. The workflow
uses the same ESP-IDF 5.3 Docker image, creates a complete
`ultrawatch-factory.bin`, and publishes it together with an ESP Web Tools
manifest and SHA-256 checksum on that tag's GitHub Release.
The tag can point to a commit on any branch.

The committed defaults configure BLE and power management. If an existing
generated build configuration predates those defaults, configure a fresh build
directory so the new defaults are applied.

Flash on macOS (replace the serial port if it differs):

```sh
python3 -m venv .venv && .venv/bin/pip install esptool==4.12.0
(cd firmware/build && ../../.venv/bin/esptool.py --chip esp32s3 -p /dev/cu.usbmodem101 --before default_reset --after hard_reset write_flash @flash_args)
```

Serve the minimal Web Bluetooth client from localhost, then open it in a
Chromium-based browser. The IMU demo imports Three.js 0.185.1 from jsDelivr,
and the GPS preview uses Leaflet 1.9.4 from unpkg with OpenStreetMap tiles, so
the visual previews also need internet access. BLE values remain available if
the map tiles cannot be loaded:

```sh
python3 -m http.server 8000 --directory web
```

Open <http://localhost:8000>, select **Connect**, and choose `UltraWatch`.

The **Firmware** page uses ESP Web Tools over Web Serial. Download
`ultrawatch-factory.bin` from a GitHub Release, select it in the page, connect
the watch over USB, and choose **Flash selected firmware**. The browser flasher
expects the complete factory image and writes it at offset `0x0`. Web Serial
requires a supported Chromium browser and a secure context (HTTPS or
localhost).
