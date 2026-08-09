# T-Watch Ultra sensors over BLE

Displays the PCF85063A clock and AXP2101 charge level in Cascadia Code inside
the validated blue safe-area contour, and exposes the clock, power management,
the BHI260AP orientation, and MIA-M10Q GPS data over Bluetooth Low Energy.

- Device name: `UltraWatch`
- Service: `7a1e0001-7a1e-4b6c-8d9e-001122334455`
- RTC: `7a1e0002-7a1e-4b6c-8d9e-001122334455`, read/write/notify each second,
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

The BHI260AP is initialized with Bosch's BHI2xy SensorAPI v1.6.0 and its stock
RAM firmware. The vendored driver, firmware image, and BSD-3-Clause license are
under `firmware/main/vendor/bhy2/`. Firmware loading failures are logged without
restarting the watch, so RTC and power remain usable instead of causing a boot
loop.

The GPS module is powered from AXP2101 BLDO1 at 3.3 V and connected to UART1
with ESP32-S3 TX on GPIO 43 and RX on GPIO 44. At boot the firmware probes
38400, 115200, and 9600 baud for a u-blox `MIA-` identity, switches to 115200,
enables the documented constellations, and requests UBX-NAV-PVT output at 1 Hz.
The receiver configuration uses the RAM layer, so it is applied again after
each watch restart. RTC synchronization, MGA aiding, GNSS software backup, and
TTFF history are intentionally outside this first live-position iteration.

The firmware embeds only the Cascadia Code glyphs required by the clock and
percentage display. They are generated from `CascadiaCode-Regular.otf` with
`firmware/tools/generate_cascadia_font.py`; the font license is included beside
the generated header in `firmware/main/CASCADIA_CODE_LICENSE.txt`.

For example, regenerate the header from a Cascadia Code installation on macOS
(the Python environment must provide Pillow):

```sh
python3 firmware/tools/generate_cascadia_font.py \
  "$HOME/Library/Fonts/CascadiaCode-Regular.otf" \
  firmware/main/cascadia_code_72.h
```

Build with ESP-IDF 5.3:

```sh
docker run --rm -v "$PWD/firmware:/project" -w /project espressif/idf:release-v5.3 idf.py -B build -D SDKCONFIG=build/sdkconfig build
```

Keeping `sdkconfig` inside the build directory ensures that the committed BLE
defaults are used even if an older generated `firmware/sdkconfig` exists.

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
