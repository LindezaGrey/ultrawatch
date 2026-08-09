# AGENT.md

## Project

ESP32-S3 firmware built on FreeRTOS via Espressif ESP-IDF. Debugging is done over JTAG (OpenOCD + GDB), not via serial monitor printf.

## Environment

- Chip: ESP32-S3 (Xtensa LX7, dual-core)
- RTOS: FreeRTOS (bundled with ESP-IDF)
- Build system: ESP-IDF (cmake + ninja), entry point `idf.py`
- Toolchain: `xtensa-esp32s3-elf-gcc` (provided by ESP-IDF docker image)
- ESP-IDF runs in Docker: image `espressif/idf:v6.0.2` (tag pinned in `Dockerfile` / `docker-compose.yml`)
- JTAG debugger: OpenOCD (target `esp32s3-builtin` for the USB-JTAG on DevKitC, or `esp32s3.cfg` with an external JTAG adapter)

### Docker setup

The ESP-IDF toolchain, `idf.py`, `openocd` and the Xtensa GDB all run inside the container. The project directory is mounted as the container workdir so the build directory is shared with the host. A `Dockerfile` (`FROM espressif/idf:v6.0.2`) and `docker-compose.yml` (device + port passthrough) are provided.

```bash
# Preferred: use the compose service
docker compose up -d                        # starts long-running esp-idf container
docker compose exec esp-idf idf.py set-target esp32s3
docker compose exec esp-idf idf.py build
docker compose exec esp-idf idf.py -p /dev/ttyACM0 flash

# Equivalent one-shot container:
docker run --rm -it \
  -v $PWD:/project \
  -w /project \
  -v $HOME/.espressif:/root/.espressif \
  -u $(id -u):$(id -g) \
  --device=/dev/ttyACM0 \
  --privileged \
  -p 3333:3333 \
  -p 5039:5039 \
  espressif/idf:v6.0.2 esp-idf/export.sh && idf.py set-target esp32s3 && idf.py build
```

Notes:
- `--device=/dev/ttyACM0` passes the ESP32-S3 USB/JTAG port into the container. Use `--privileged` and/or the correct serial device path if your board enumerates elsewhere.
- `-p 3333:3333` exposes the OpenOCD GDB port and `-p 5039:5039` exposes the ESP-IDF monitor's port so GDB can run on the host if preferred.
- Mounting `$HOME/.espressif` keeps the ESP-IDF tools (and downloads) cached across container runs.
- All drivers are our own components built on Espressif's native APIs (`driver/*`, `esp_lcd`, `sdspi_host`, ...); no Arduino. The `twatch_board` component owns pins, buses and the AXP2101 power tree.

## Hardware quirks & recovery (learned the hard way)

- **The T-Watch Ultra runs on battery**: unplugging/replugging USB does NOT reset the chip (battery keeps it powered). Use a full power cycle (hold PWR ~6 s to power off, replug, tap PWR ~1 s to power on) or the RST button.
- **Stale `idf_monitor` processes block the serial port**: a `docker run -it ... monitor` left running (e.g. after a `script`/`timeout` wrapper was killed) keeps `idf_monitor.py` attached to `/dev/ttyACM0`, causing "port busy", dropped connections and intermittent USB. Always `docker ps -a` + `pgrep -af idf_monitor` and stop/kill strays before flashing. Prefer `docker compose exec` for monitor so it is tied to the compose container.
- **Enter download mode for reliable flashing**: the builtin USB-Serial/JTAG is unstable for long writes (16 MB factory image) unless the chip is held in download mode. Official sequence: hold **BOOT**, click **RST**, release BOOT; USB port then stays fixed. Verify with `esptool.py --chip esp32s3 -p /dev/ttyACM0 --before=no-reset-no-sync chip-id`. After flashing, press **RST** once to exit download mode.
- **Never kill OpenOCD without `reset run`**: killing OpenOCD while the chip is `reset halt`-ed leaves the CPU halted; the USB-Serial/JTAG then won't respond to esptool ("readiness to read but returned no data"). Exit with `reset run` / `shutdown` cleanly.
- **Recover a bricked/boot-looping device with LilyGO factory firmware**:
  `esptool.py --chip esp32s3 -p /dev/ttyACM0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m 0x0 <factory.watch.ultra.sx1262.20260424.bin>`
  (full 16 MB image; write address 0, not 0x1000). Factory bin: https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/master/firmware/factory.watch.ultra.sx1262.20260424.bin
- **Host-side esptool works** (installed at `/usr/bin/esptool.py`, v5.3.x) and is used for recovery/recovery writes; the ESP-IDF toolchain still runs in Docker.
- **Boot firmware must never `ESP_ERROR_CHECK` a peripheral init** — a failed sensor/PMU probe then becomes an infinite reboot loop (which destabilizes USB and flashing). Peripheral failures log-and-continue (see `twatch_board_init`).

## Build / Flash / Debug workflow

```bash
# Shortcut: run all idf.py commands through docker (see Docker setup above)
idf.py set-target esp32s3
idf.py build

# Flash + monitor (serial, fallback only)
idf.py -p /dev/ttyACM0 flash monitor

# Rebuild + flash + start JTAG debug session (main workflow)
idf.py -p /dev/ttyACM0 flash
openocd -f board/esp32s3-builtin.cfg &   # inside the container
xtensa-esp32s3-elf-gdb build/UWatch.elf -x .gdbinit
```

## JTAG debug (primary workflow)

- On-chip USB-JTAG (no external probe needed): the ESP32-S3 exposes JTAG over its USB port. OpenOCD config: `board/esp32s3-builtin.cfg`. Run OpenOCD inside the container with the USB device passed through.
- Restart the chip with the USB-JTAG mode selected before attaching: `idf.py -p /dev/ttyACM0 run` or momentarily reset into download mode.
- Use a `.gdbinit` in the repo root to attach:

```
target remote :3333
monitor reset halt
maintenance packet Qqemu.PhyMemMode:1
set remotetimeout 15
load
```

- If GDB runs on the host, it must reach OpenOCD's port `3333` (already published by docker) and an Xtensa GDB must be available on the host; otherwise run GDB inside the container too.
- FreeRTOS-aware debugging: `xtensa-esp32s3-elf-gdb` from ESP-IDF has FreeRTOS task awareness built in via `task` commands (`task list`, `task current`).
- Common GDB commands: `continue`, `break`, `b <file>:<line>`, `p *pxCurrentTCB`, `thread`, `task list`.

## Conventions

- Target app: `UWatch` (name may be revised to lower-case `uwatch` in build config).
- Keep FreeRTOS tasks small; use `esp_log` (`ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`) rather than raw `printf`.
- Do not rely on serial monitor for debugging; the canonical debug flow is JTAG/GDB.
- Config and menuconfig settings: `idf.py menuconfig`.

## Verification

- Build must pass: `idf.py build` (exit 0) inside the container.
- After changes, re-flash and attach GDB to verify no new warnings/crashes.
- If JTAG attach fails, confirm the device is in download/JTAG mode, the USB device is passed into the container, and OpenOCD is listening on `:3333`.
