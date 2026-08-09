# UWatch

FreeRTOS / ESP-IDF firmware platform for the **LilyGO T-Watch Ultra** (ESP32-S3). Built entirely on Espressif native APIs (`driver/*`, `esp_lcd`, `sdspi_host`, ...) with no Arduino dependency. Debugging is JTAG-first (OpenOCD + GDB over the on-chip USB-JTAG), and the whole ESP-IDF toolchain runs in Docker.

## Hardware overview

| Component | Part | Interface |
|---|---|---|
| MCU | ESP32-S3 (dual-core @240 MHz) | 16 MB flash, 8 MB PSRAM |
| Display | CO5300 2.06" AMOLED, 410x502 | QSPI |
| Touch | CST9217 | I2C |
| PMU | AXP2101 | I2C |
| IMU / smart sensor | Bosch BHI260AP | I2C |
| RTC | PCF85063A | I2C |
| Haptics | DRV2605 | I2C |
| GPIO expander | XL9555 | I2C |
| LoRa | SX1262 (868 MHz, EU) | SPI |
| NFC | ST25R3916 | SPI |
| SD card | up to 32 GB (FAT) | SPI |
| Audio | MAX98357A Class-D | I2S |
| Microphone | T3902 PDM | I2S (PDM RX) |
| GNSS | u-blox MIA-M10Q | UART |

Full pin map and power tree: [docs/hardware.md](docs/hardware.md).

The screen has rounded corners; `assets/ui/safe_area_transparent.png` is the transparent safe-area overlay (410×502) to use when placing UI content within the rounded display.

## Project structure

```
UWatch/
├── CMakeLists.txt              # top-level ESP-IDF project
├── sdkconfig.defaults          # target esp32s3, 16MB flash, 8MB QSPI PSRAM, JTAG/OCD aware
├── partitions.csv              # nvs / phy_init / factory
├── Dockerfile                  # espressif/idf:v6.0.2
├── docker-compose.yml          # serial/device passthrough + GDB/monitor ports
├── .gdbinit                    # OpenOCD attach script
├── docs/hardware.md            # T-Watch Ultra pin + peripheral reference
├── main/                       # app entry (app_main, task graph)
└── components/
    ├── twatch_board/           # pins, I2C/SPI/QSPI bus init, AXP2101 power tree
    └── drivers/                # one component per peripheral, Espressif-API based
        ├── axp2101/            ├── cst9217/       ├── co5300/
        ├── pcf85063a/          ├── bhi260ap/      ├── drv2605/
        ├── xl9555/             ├── max98357a/     ├── t3902/
        ├── m10q/               ├── sx1262/        └── st25r3916/
```

`twatch_board` opens the shared I2C/SPI buses and hands out device handles to the driver components, so each peripheral stays a composable, Espressif-API-only component. Driver bodies are skeletons ready to be implemented.

## Prerequisites

- Docker (the ESP-IDF toolchain, OpenOCD and the Xtensa GDB run in the `espressif/idf:v6.0.2` container)
- A T-Watch Ultra (LoRa 868 MHz variant) connected via USB-C
- `docker compose` plugin (or plain `docker run`)

## Build & flash

```bash
docker compose up -d                                   # start esp-idf container
docker compose exec esp-idf idf.py set-target esp32s3  # first time only
docker compose exec esp-idf idf.py build
docker compose exec esp-idf idf.py -p /dev/ttyACM0 flash
```

The project directory is bind-mounted at `/project`; `~/.espressif` is cached for tool downloads. The USB port `/dev/ttyACM0` is passed into the container. If your board enumerates elsewhere, adjust the `devices:` entry in `docker-compose.yml`.

## JTAG debugging (primary workflow)

The ESP32-S3 exposes its USB-JTAG over the same USB-C port, so no external probe is needed.

1. Start OpenOCD inside the container (uses the on-chip USB-JTAG):
   ```bash
   docker compose exec esp-idf openocd -f board/esp32s3-builtin.cfg
   ```
2. In another terminal, flash the app, then attach the Xtensa GDB (also inside the container, or on the host via published port `3333`):
   ```bash
   docker compose exec esp-idf idf.py -p /dev/ttyACM0 flash
   docker compose exec esp-idf xtensa-esp32s3-elf-gdb build/UWatch.elf -x .gdbinit
   ```
3. GDB connects to OpenOCD on `:3333`. FreeRTOS task awareness is available via `task list`, `task current`.

The `.gdbinit` does `target remote :3333`, `monitor reset halt`, and `load`. If attach fails, confirm the board is in download/JTAG mode and OpenOCD reports listening on `3333`.

## ESP-IDF API reference

The components map 1:1 onto the official ESP32-S3 API docs
([API Reference, ESP-IDF v6.0.2](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/index.html)):

| Area | Component | API reference |
|---|---|---|
| I2C bus | `twatch_board`, I2C drivers | [I2C](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html) |
| SPI bus | `twatch_board`, `sx1262`, `st25r3916`, SD | [SPI Master](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html) |
| Display | `co5300` | [LCD / esp_lcd](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/lcd/index.html) |
| GPIO / buttons | `twatch_board` | [GPIO](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html) |
| UART / GNSS | `m10q` | [UART](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/uart.html) |
| I2S audio / PDM mic | `max98357a`, `t3902` | [I2S](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2s.html) |
| SD card (FAT) | (app) | [SD SPI host](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/sdspi_host.html), [SD/MMC + FATFS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/storage/sdmmc.html), [FATFS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/storage/fatfs.html) |
| Storage | NVS | [NVS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/storage/nvs_flash.html) |
| RTOS | FreeRTOS (IDF) | [FreeRTOS (IDF)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos_idf.html) |
| Power / sleep | (planned) | [Power management](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/power_management.html), [Sleep modes](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/sleep_modes.html) |
| Logging / time / events | app | [Logging](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/log.html), [Watchdogs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html), [esp_timer](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/esp_timer.html), [esp_event](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/esp_event.html) |
| Debug | JTAG trace | [Application Level Tracing](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/app_trace.html) |

Follow the [API conventions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/api-conventions.html) (`esp_err_t`, config structs) when implementing the driver skeletons. Networking (Wi-Fi/BLE/MQTT) can be added later via the [Networking](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/network/index.html), [Bluetooth (NimBLE)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/bluetooth/nimble/index.html) and [MQTT](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/protocols/mqtt.html) APIs.

## References

- [LilyGO T-Watch Ultra product page](https://www.lilygo.cc/products/t-watch-ultra)
- [LilyGO T-Watch Ultra hardware doc](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/master/docs/hardware/lilygo-t-watch-ultra.md)
- [Schematic (V1.0)](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/master/schematic/T-Watch%20Ultra%20V1.0%20SCH%2025-07-24.pdf)
- [LilyGoLib repo](https://github.com/Xinyuan-LilyGO/LilyGoLib)
- [ESP-IDF Programming Guide (ESP32-S3)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/index.html)
