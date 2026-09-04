# UWatch agent guide

## Project and environment

- ESP32-S3 firmware using ESP-IDF v6.0.2, FreeRTOS, CMake/Ninja, and native Espressif APIs (not Arduino).
- Board: LilyGO T-Watch Ultra LoRa. Main USB interface is normally `/dev/ttyACM0`.
- Native ESP-IDF at `~/esp/esp-idf` is the day-to-day environment. Docker is only for clean, reproducible builds; avoid it for normal USB/JTAG work.
- Load the toolchain in every new shell:

  ```bash
  source ~/esp/esp-idf/export.sh
  ```

- `twatch_board` owns pins, buses, and the AXP2101 power tree. Keep peripheral drivers as independent components.

Read `README.md` for the standard setup, `docs/hardware.md` for board details, and `docs/display-power-design.md` for display/power design.

## Safe hardware operating rules

Hardware-changing operations require the user's explicit confirmation: flashing, erasing, resetting, running OpenOCD, GDB writes, or serial writes.

- Serial MCP is log observation only unless the user explicitly requests an input/write action.
- Only one program may own the serial interface. Close Serial MCP, `idf.py monitor`, and any other serial client before flashing or resetting.
- Do not start a second flasher while one is active. A full flash can take longer than an agent command timeout: run one normal flash process and inspect its log rather than splitting images or retrying over it.
- Before a hardware operation, check that the intended board is present, that no stale `idf_monitor`, OpenOCD, or flasher owns the port, and that the requested action names the right device.
- USB unplug/replug does not necessarily reset this battery-powered watch. Use the physical reset/power procedure when recovery requires it.
- For reliable long writes, enter download mode: hold **BOOT**, press **RST**, release **BOOT**. After flashing, press **RST** once to boot normally.
- Exit OpenOCD cleanly with `reset run` and `shutdown`; do not leave the CPU halted.
- Failed peripheral initialization must log and continue; do not turn a missing peripheral into a boot loop with `ESP_ERROR_CHECK`.

## Build, flash, and debug

```bash
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3       # first setup only
idf.py build
```

After explicit approval to deploy:

```bash
idf.py -p /dev/ttyACM0 flash
```

Serial output is a fallback. JTAG is the primary debugging path:

```bash
openocd -f board/esp32s3-builtin.cfg
xtensa-esp32s3-elf-gdb build/UWatch.elf -x .gdbinit
```

Use GDB/FreeRTOS task inspection for crashes, breakpoints, and task state. If JTAG attach fails, first verify the board is in a suitable USB/JTAG state and OpenOCD is listening on `:3333`.

## Development loop

1. Implement the smallest focused change.
2. Run relevant host/unit tests. For screen changes, use the simulator first.
3. Build the firmware.
4. Review the diff and build result.
5. Only after the user explicitly authorizes it, flash the board.
6. Use JTAG to investigate unexpected hardware behavior; reopen the serial log viewer only after flash completion.

## Simulator and UI

The SDL2 simulator in `sim/` renders shared `main/screens/*.c` sources without hardware. Use it before device flashing for UI work:

```bash
cd sim
mkdir -p build && cd build
cmake .. && make -j$(nproc)
./uwatch_sim
```

For headless screenshots, use the simulator's framebuffer capture—not desktop screenshot tools:

```bash
SDL_VIDEODRIVER=offscreen UWATCH_SIM_SCREEN=settings \
  UWATCH_SIM_SHOT=/tmp/settings.ppm ./uwatch_sim
```

The AMOLED panel is 410×502 at roughly 315 PPI. Verify screenshots at 1:1 scale; favor legible fonts, generous touch targets, aligned spacing, and the safe-area guidance in `docs/application.md`.

## Code and verification conventions

- Keep the POC code concise and simple. Avoid unnecessary abstractions, defensive scaffolding, and comments that merely restate code.
- Keep FreeRTOS tasks small. Use `ESP_LOGI`, `ESP_LOGW`, and `ESP_LOGE` rather than raw `printf`.
- Run `idf.py build` after firmware changes. Run the relevant simulator or host tests for UI/component work.
- `idf.py menuconfig` owns configuration changes.
- Issues use the GitHub CLI; see `docs/agents/issue-tracker.md`. Architectural context is in `CONTEXT.md` and `docs/adr/`.
