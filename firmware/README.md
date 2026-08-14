# ultrawatch firmware

ESP32-S3 firmware for the T-Watch Ultra (LilyGO) smartwatch. Time display (Casio
G-Shock style) with light-sleep power management, SD-card power logging, and a
console over USB-Serial-JTAG.

Target: **ESP32-S3**, ESP-IDF **6.0.2**, LVGL **9.5.0**, `esp_lcd_co5300` **2.1.0**.

---

## Table of contents

- [Hardware overview](#hardware-overview)
- [Build & flash](#build--flash)
- [Source layout](#source-layout)
- [Board wiring / pinout](#board-wiring--pinout)
- [Boot sequence](#boot-sequence)
- [Display subsystem](#display-subsystem)
- [Power management (light sleep)](#power-management-light-sleep)
- [SD card](#sd-card)
- [Power logger](#power-logger)
- [Console commands](#console-commands)
- [Screenshots](#screenshots)
- [Key decisions log](#key-decisions-log)
- [Field test results](#field-test-results)
- [Known issues / open questions](#known-issues--open-questions)

---

## Hardware overview

- SoC: ESP32-S3, 240 MHz, 16 MB flash, **8 MB QSPI PSRAM** (APS6404L, quad, not octal).
- Display: 2.06" 410x502 AMOLED, **CO5300** driver, **QSPI** (SPI3).
- PMU: **AXP2101** (power rails, battery ADC, ALDO1/ALDO2).
- GPIO expander: **XL9555** (I2C) — display/touch enable, SD card-detect.
- Touch: **CST9217** (I2C), interrupt on GPIO12.
- RTC: **PCF85063A** (I2C), 24h mode.
- SD card: SPI2 (shared with NFC + LoRa on this board).
- Console: USB-Serial-JTAG (native USB), `/dev/ttyACM0` on the host.

## Build & flash

```sh
source /home/peter/esp/esp-idf/export.sh
idf.py -p /dev/ttyACM0 build flash monitor
```

- The serial port must be free; kill any stray `openocd` before flashing.
- Firmware lives in `firmware/`; build output is `twatch_ultra_firmware.elf`.
- Partition table: custom (`partitions.csv`) — single `factory` app partition
  (5 MB), NVS + phy_init. See [partitions.csv](partitions.csv).
- Key `sdkconfig.defaults` settings:
  - `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`
  - `CONFIG_ESP_MAIN_TASK_STACK_SIZE=16384` (LVGL object creation + first full render)
  - `CONFIG_FREERTOS_HZ=1000` — see [decisions](#key-decisions-log)
  - `CONFIG_SPIRAM_MEMTEST=n` (skip 16 MB memtest that hangs this board)
  - `CONFIG_LOG_DEFAULT_LEVEL_INFO` — DEBUG console flood breaks host capture
  - LVGL 16-bit color, snapshot enabled, CLIB malloc (so LVGL allocations land in PSRAM)

## Source layout

```
firmware/
  CMakeLists.txt               project() = twatch_ultra_firmware
  partitions.csv               nvs / phy_init / factory (5 MB)
  sdkconfig.defaults           default config knobs
  main/
    app_main.c                 boot init order, main loop (lv_timer_handler + power tick)
    cmd_time.c                 console commands (settime/time/shot/mode/brightness/diag/sd/cat/sleep)
    power.c / power.h          light-sleep engine (timer + EXT1 touch wake)
    powerlog.c / powerlog.h    5-minute power log to /sdcard/power.log
    screenshot.c / screenshot.h  screenshot capture + UART dump (debugging)
    bsp/
      bsp_twatch_ultra.h       all pins, I2C addresses, board constants
      bsp_i2c.c/.h             shared I2C bus (400 kHz)
      bsp_axp2101.c/.h         PMU: rails, battery ADC
      bsp_xl9555.c/.h          GPIO expander, SD detect
      bsp_display.c/.h         CO5300 QSPI panel init, TE/SPI diagnostics, brightness
      bsp_cst9217.c/.h         touch controller
      bsp_pcf85063.c/.h        RTC
      bsp_sdcard.c/.h          SD power-cycle, mount, card info
    ui/
      ui_clock.c/.h            LVGL clock face, modes, flush callback, TE sync
      fonts/lv_font_montserrat_96.c  96 px digit font (generated)
```

## Board wiring / pinout

All constants live in `main/bsp/bsp_twatch_ultra.h`. Verified against the
official T-Watch Ultra schematic and LilyGoLib.

### I2C bus (400 kHz, GPIO2=SCL, GPIO3=SDA)
| Device | Address |
|---|---|
| AXP2101 (PMU) | 0x34 |
| XL9555 (expander) | 0x20 |
| CST9217 (touch) | 0x5A |
| PCF85063A (RTC) | 0x51 |

### Display QSPI (SPI3)
| Signal | Pin |
|---|---|
| SCK | 40 |
| D0 | 38 |
| D1 | 39 |
| D2 | 42 |
| D3 | 45 |
| CS | 41 |
| RST | 37 |
| TE (tearing effect) | 6 |

### SD card (SPI2, shared with NFC + LoRa)
| Signal | Pin |
|---|---|
| CS | 21 |
| SCK | 35 |
| MOSI | 34 |
| MISO | 33 |
| Power | AXP2101 ALDO1 (SD_VDD, 3300 mV) |
| Card-detect | XL9555 P1.10 (pin 10), active low |
| NFC CS (deasserted) | 4 |
| LoRa CS (deasserted) | 36 |
| LoRa RST (deasserted) | 47 |

### XL9555 outputs
| Pin | Function |
|---|---|
| 6 | DRV_EN |
| 7 | DISP_EN |
| 8 | TOUCH_RST |

### Other
- Touch interrupt: GPIO12 (also used as EXT1 light-sleep wake source, any-low).

## Boot sequence

`app_main.c`, ordered, each step logged as `OK` / `FAILED`:

1. `nvs_flash_init()` (fatal on failure)
2. `bsp_i2c_init()` (fatal)
3. `bsp_axp2101_init()` (fatal) — enables ALDO2 (display power) + battery ADC
4. `bsp_xl9555_init()` (fatal) — configures expander, powers display/touch
5. `bsp_display_init()` (fatal) — CO5300 QSPI panel
6. Set brightness to `UI_DAY_MODE_BRIGHTNESS_PCT` (30%)
7. Show 4-quadrant color test pattern (~1.5 s) to verify orientation/rotation
8. `bsp_touch_init()` (non-fatal)
9. `bsp_rtc_init()` (non-fatal); if the RTC lost time, set a default 2026-01-01 12:00
10. `ui_clock_init()` (non-fatal) — LVGL clock face
11. `bsp_sd_init()` (non-fatal) — auto-mount SD
12. `powerlog_init()` (non-fatal)
13. `power_sleep_init()` (non-fatal)
14. Start console REPL over USB-Serial-JTAG; register commands

Main loop: `lv_timer_handler()` → `power_sleep_tick()` → delay. A heartbeat log
every ~50 ticks prints battery state.

## Display subsystem

### CO5300 panel
- Init command table in `bsp_display.c` (taken from LilyGoLib):
  `0xFE 0x00`, `0xC4 0x80`, `0x3A 0x55` (RGB565), `0x35 0x00` (TE on),
  `0x53 0x20`, `0x63 0xFF`, column/row address windows, `0x11` sleep out,
  `0x29` display on, `0x51 0x00` brightness.
- Visible area: columns 0x16..0x1AF (22..431), rows 0..0x1F5 (0..501).
  `esp_lcd_panel_set_gap(panel, 22, 0)`, no xy swap; LVGL x/y map directly.
- QSPI bus clock: **60 MHz (~30 MB/s)** — see [glitch fix](#key-decisions-log).
- RGB565 byte order: LVGL buffer is byte-swapped in the flush callback
  (`UI_DISPLAY_SWAP_RGB565`).

### Flush pipeline
- LVGL renders into two internal-RAM DMA buffers of `410 x 60 x 2` bytes each
  (`LV_DISPLAY_RENDER_MODE_PARTIAL`), so frames are written band by band.
- `lvgl_flush_cb` waits for a **TE edge** before every band (not just the first),
  byte-swaps, calls `esp_lcd_panel_draw_bitmap`, then blocks on
  `bsp_display_wait_flush_done()` before `lv_display_flush_ready()`.
  This guarantees LVGL never reuses a color buffer while the SPI DMA is still
  reading it (prevents mixed/teared bands).

### Diagnostics
- SPI transfer stats (`bsp_display.c`): flush starts, completions, overlaps.
- TE stats (`ui_clock.c`): calls, timeouts, max/last wait in µs.
- Both printed by the console `diag` command. Healthy state shows
  `overlaps=0` and `timeouts=0`.

### Brightness
- `bsp_display_set_brightness(pct)` → `esp_lcd_panel_co5300_set_brightness()`
  (DCS 0x51).
- Day mode: 30% (`UI_DAY_MODE_BRIGHTNESS_PCT`). Low-power red mode: 30%
  (`UI_RED_MODE_BRIGHTNESS_PCT`).

## Power management (light sleep)

`power.c` — light-sleep engine driven from the main loop:

- **Awake window:** 10 s after the last touch (`POWER_AWAKE_MS`).
- When idle, the panel is dimmed to `POWER_SLEEP_BRIGHTNESS` (20%, clamped to the
  user brightness), then light sleep is entered.
- **Wake sources:** timer (scheduled to the next minute boundary,
  `(60 - sec) * 1000` ms, min 1 s) and **EXT1 touch wake**
  (`GPIO12`, `ESP_EXT1_WAKEUP_ANY_LOW`).
- After wake: brightness restored, LVGL screen invalidated and the loop re-runs
  the LVGL tick immediately so the display reflects the new time.
- **Skipped entirely while a USB host is connected**
  (`usb_serial_jtag_is_connected()`) so the console stays live and the USB
  device does not re-enumerate every wake.
- Enable/disable and counters via the `sleep` console command.
- WDT note: earlier a busy-spin in `shot_task` starved `idle1` and the WDT fired;
  the shot task now `vTaskDelay(1)` per hex chunk.

### Panel brightness vs. sleep dimming
AMOLED power scales roughly linearly with brightness, so dimming to 20% during
the sleep window cuts panel power ~5x while the display stays visible.

## SD card

`bsp_sdcard.c`:

- **Power rail:** AXP2101 ALDO1 (SD_VDD 3300 mV). Init power-cycles the card:
  ALDO1 off 250 ms → on 250 ms (long enough for the rail cap to discharge — a
  mere brown-out is not a reset), plus 250 ms settle before talking to the card.
- **Card detect:** XL9555 pin 10, active low. If no card, init fails fast with
  `ESP_ERR_NOT_FOUND`.
- **Critical fix — shared SPI2 deasserts:** LoRa/NFC share SPI2 with the SD card.
  Before SD init, GPIO4 (NFC CS), GPIO36 (LoRa CS) and GPIO47 (LoRa RST) are
  forced high as outputs so they can never drive MISO during SD commands
  (matches LilyGo `initShareSPIPins()`). Without this, SD commands intermittently
  saw junk on MISO and CMD0/init timed out (`0x107` = `ESP_ERR_TIMEOUT`).
- Mount is retried up to 3x, power-cycling the card between attempts.
- SD bus: SPI2, 4 MHz (`BSP_SD_SPI_FREQ_KHZ`), FAT32/FAT16, no auto-format.
- `bsp_sd_last_error()` tracks the failing step for diagnostics (`sd` command).
- SD is **kept powered** at all times (ALDO1 stays on); no power gating after log.

## Power logger

`powerlog.c`:

- Called from the LVGL timer every second (`powerlog_tick()`).
- Logs one line every **5 minutes** (`POWERLOG_INTERVAL_MS`) to
  `/sdcard/power.log` (appended). Header on empty file:
  `datetime,capacity_pct,voltage_mv`.
- If the card is not mounted, retries mount; on failure backs off 30 s rather
  than hammering the bus.
- Sample row: `2026-08-04 11:00:00,96,4169`.
- Read back with the `cat /sdcard/power.log` console command.

## Console commands

All registered in `cmd_time.c`, use over `idf.py monitor` or any terminal on
`/dev/ttyACM0`:

| Command | Usage | Description |
|---|---|---|
| `settime` | `settime YYYY-MM-DD HH:MM:SS` | Set RTC time (validates + computes weekday) |
| `time` | `time` | Print RTC time |
| `mode` | `mode` or `mode 0-1` | Cycle display mode / select one (0=casio day, 1=red low-power) |
| `brightness` | `brightness 0-100` | Set panel brightness (also updates power engine user brightness) |
| `shot` | `shot` | Trigger screenshot capture + UART dump |
| `diag` | `diag` | Print SPI/TE transfer diagnostics |
| `sd` | `sd` | Print SD state (ALDO1 regs, detect, mount, capacity/free); re-probes card |
| `cat` | `cat <path>` | Print a file from the SD card (e.g. `/sdcard/power.log`) |
| `sleep` | `sleep on\|off` | Toggle/print light-sleep engine state and wake counters |

## Screenshots

`ui_capture_screenshot()` (triggered by `shot` command or a **long press** on the
touch screen, `UI_LONG_PRESS_MS` = 1 s):

- Swaps the LVGL flush callback to a capture callback and sets a full-frame
  `LV_DISPLAY_RENDER_MODE_DIRECT` buffer, forces `lv_refr_now`, then restores the
  normal partial-buffer pipeline.
- The captured buffer is handed to `screenshot.c`, which streams it out over the
  console as hex (host tooling re-assembles it into an image).
- The capture is queued/deferred; the shot task delays per chunk so it never
  starves the WDT idle task.

## Key decisions log

This is the accumulated "why" behind non-obvious code.

### 1. WDT reset root cause (fixed)
`shot_task` busy-spun writing hex chunks without yielding, starving the idle
task and tripping the watchdog. Fix: `vTaskDelay(1)` per chunk. Captures are
now clean and the watch never watchdog-resets during a screenshot dump.

### 2. SD mount failures → shared SPI2 chip selects (fixed)
Symptom: SD init intermittently failed with CMD0/init timeout (`0x107`).
Root cause: LoRa/NFC share SPI2 with the SD card and their floating CS lines
could drive MISO. Fix: force GPIO4/36/47 high as outputs before SD init
(matches LilyGo). This was the final, field-verified fix.

### 3. `CONFIG_FREERTOS_HZ=1000`
LVGL delays in ms map to real FreeRTOS ticks. At 100 Hz, `pdMS_TO_TICKS(5)`
truncates to 0, producing a busy main loop.

### 4. INFO log level only
DEBUG-level raw touch logs flooded the USB-Serial-JTAG and interfered with the
REPL and screenshot dumps. Keep INFO.

### 5. Never set stdout O_NONBLOCK
The USB-Serial-JTAG VFS shares one non-blocking flag across stdin/stdout/stderr.
Setting O_NONBLOCK makes the REPL's stdin reads fail with EWOULDBLOCK and it
spins printing `esp> ` at ~100 lines/s. In blocking mode the driver write path
blocks at most ~50 ms per byte, so a full TX buffer (host not reading) can never
stall app tasks permanently. The console stays interactive.

### 6. Display glitch / diagonal tearing — root cause and fix
- **Root cause:** the CO5300 GRAM scan runs at ~26 MB/s (410x502x2 @ 60 Hz).
  The display QSPI bus was 40 MHz (~20 MB/s), so full-frame/band writes could
  never stay ahead of the scan. Any write landing mid-scan collided with rows
  being read → diagonal scramble + tearing across mode switches.
- **Fix:** raise `io_cfg.pclk_hz` to **60 MHz QSPI (~30 MB/s)** in
  `bsp_display.c`, so band writes outrun the scan and each TE-synced band stays
  ahead. This is the actual glitch fix.
- **Double buffering attempts (failed, reverted):**
  1. Full-screen PSRAM buffers + SPI bounce path →
     `spicommon_dma_setup_priv_buffer(430): Failed to allocate priv TX buffer`
     (needs ~411 KB internal bounce; doesn't fit).
  2. `io_cfg.flags.psram_dma_direct = true` →
     `DMA TX underflow detected` + panel IO wedged (transactions not recycled).
  3. Kept: **internal-RAM partial double buffer** (`410 x 60 x 2` x2,
     `LV_DISPLAY_RENDER_MODE_PARTIAL`) — effective now that 60 MHz keeps bands
     ahead of the scan.
- `esp_cache_msync` / `esp_mm` were removed from `ui_clock.c` and
  `main/CMakeLists.txt` (not needed with the internal partial buffer).

### 7. TE-sync every band
A full redraw spans several panel frames; aligning only the first band is not
enough. Every band waits for a TE (v-blank) edge, so a partial-band write lands
between scans. TE is enabled via `0x35` in the init sequence. Timeouts are
monitored (`te` stats) and a 50 ms timeout keeps the pipeline safe if TE
disappears.

### 8. Day mode brightness 30%
`UI_DAY_MODE_BRIGHTNESS_PCT = 30` (was a debug 5%). Casio dark face on black —
the mostly-black image is itself burn-in friendly. The old "casio light" mode
was removed; the only two modes are casio dark and red low-power.

### 9. Red low-power mode
Red-only subpixels on black, at 30% brightness: the fewest lit subpixels and
the lowest PWM — the most burn-in-friendly mode.

### 10. Light sleep design
- Never sleep while USB is connected (console must stay live, no re-enumeration).
- Wake on minute boundary to update the display exactly once per minute; wake on
  touch (EXT1) to show the time immediately.
- Panel dimmed (not off) during sleep: visible but ~5x less power.

### 11. `0.107` = `ESP_ERR_TIMEOUT`
CMD0/init timeout diagnostic value printed by `esp_err_to_name()` as `ESP_ERR_TIMEOUT` (0x107).

## Field test results

- **Battery/SD field test (watch worn, ~90 min):**
  `/sdcard/power.log` showed 18 rows 10:54→12:24: 96→93%, 4169→4134 mV, 5-min
  cadence. Extrapolation ~3% per 90 min → **~35–50 h runtime** on a charge.
  Battery later back at 97%/4170 mV (charging).
- **Sleep stats:** `sleep: enabled=1 sleeps=127 wake_timer=79 wake_ext1=48`.
- **After 60 MHz + internal partial buffer** (verified clean boot):
  `brightness: OK`, `mode=0 fg=FFFFFF accent=00BFFF`, `ui clock ready`,
  `mounted at /sdcard, 30436 MB`; `diag` over mode cycles:
  `xfer: starts=38 completions=54 overlaps=0`, `te: calls=38 timeouts=0
  max_us=11815 last_us=11176`; screenshot pipeline works
  (`capture: buffer 410x502 stride=820`).
- **SD:** `detect=1 mounted=1 capacity=30436MB free=30420MB`.

## Known issues / open questions

- **Burn-in protection:** the CO5300's only AMOLED lifetime feature is **ACL
  (Auto Current Limit)** — WRACL 0x55, parameter `RAD_ACL[1:0]`:
  `00` = disabled (default), `11` = enabled. There is **no pixel-shift /
  screen-saver / image-sticking prevention** in the chip. ACL is currently NOT
  enabled in the init sequence (pending decision — one line would add it:
  `{ 0x55, (uint8_t[]){ 0x03 }, 1, 0 }`). The 30% brightness on black is the
  primary burn-in mitigation today.
- Visual confirmation of the tearing fix (mode 0↔1 switches, minute updates) is
  still pending on hardware.
- Light-sleep validation with USB unplugged: battery data was collected
  field-side, but a formal sleep-validity pass has not been explicitly confirmed.
- The 5 s boot test pattern could be removed once the bring-up suites stabilize.
- Direct JTAG GDB halted-calls into the running firmware fault (interrupt-driven
  I2C) — use console commands instead. OpenOCD also wedges after long uptime;
  restart via `kill openocd`, relaunch with `-c "adapter speed 20000"` quoted.

---

## Environment reference

- ESP-IDF: `/home/peter/esp/esp-idf` (`export.sh`).
- Managed components (gitignored): `managed_components/`
  (`espressif__esp_lcd_co5300`, `lvgl__lvgl`).
- Stock wiring reference (not in this repo): `/tmp/mammouth/twatch_ultra_pins.h`,
  `/tmp/mammouth/LilyGoLib/`.
- Console helper: `/tmp/mammouth/console_cmd.py <cmd>…`; inline python is used
  for boot-log / DTR-RTS resets.
