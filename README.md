# T-Watch Ultra RTC over BLE

Displays the PCF85063A clock inside the validated blue safe-area contour and
exposes RTC plus AXP2101 battery/power data over Bluetooth Low Energy.

- Device name: `UltraWatch`
- Service: `7a1e0001-7a1e-4b6c-8d9e-001122334455`
- RTC: `7a1e0002-7a1e-4b6c-8d9e-001122334455`, read/write/notify each second,
  payload `YYYY-MM-DDTHH:MM:SS`
- Battery/power: `7a1e0003-7a1e-4b6c-8d9e-001122334455`, read/notify every five
  seconds, payload `percentage,millivolts,direction,vbus,present`

Power direction values are `0` standby, `1` charging, `2` discharging, and `3`
reserved/unknown. `vbus` and `present` are `0` or `1`.

The RTC stores local calendar time only; the payload has no timezone or UTC
offset. Its hardware year range is represented here as 2000 through 2099. This
minimal iteration does not require BLE pairing or an encrypted connection.

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
Chromium-based browser:

```sh
python3 -m http.server 8000 --directory web
```

Open <http://localhost:8000>, select **Connect**, and choose `UltraWatch`.
