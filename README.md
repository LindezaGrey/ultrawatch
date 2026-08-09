# T-Watch Ultra sensors over BLE

Displays the PCF85063A clock and AXP2101 charge level in Cascadia Code inside
the validated blue safe-area contour, and exposes the clock, power management,
and BHI260AP orientation over Bluetooth Low Energy.

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
Chromium-based browser. The IMU demo imports Three.js 0.185.1 as an ES module
from jsDelivr, so the browser also needs internet access:

```sh
python3 -m http.server 8000 --directory web
```

Open <http://localhost:8000>, select **Connect**, and choose `UltraWatch`.
