# T-Watch Ultra — Hardware Reference

Source: [LilyGO T-Watch Ultra hardware doc](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/master/docs/hardware/lilygo-t-watch-ultra.md),
[Schematic (V1.0)](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/master/schematic/T-Watch%20Ultra%20V1.0%20SCH%2025-07-24.pdf),
[Product page](https://www.lilygo.cc/products/t-watch-ultra).

## SoC

| Feature | Value |
|---|---|
| MCU | Espressif ESP32-S3 (Xtensa LX7, dual-core @240 MHz) |
| Flash | 16 MB QSPI (external) |
| PSRAM | 8 MB QSPI (external) |
| Wireless | Wi-Fi 802.11 b/g/n, Bluetooth LE 5.0 |
| USB | USB-C (charge + program; no external power supply function) |

## Display

| Feature | Value |
|---|---|
| Panel | 2.06" CO5300 AMOLED, QSPI interface |
| Resolution | 410 x 502 |
| Colors | 16.7 M |
| Luminance | 600 nit |
| Touch | CST9217 capacitive touch (I2C) |

The panel has rounded corners; `assets/ui/safe_area_transparent.png` is the transparent safe-area overlay (410×502) for placing UI content inside the rounded edges.

### Display driver notes (verified on hardware)

- Driven via the official **`espressif/esp_lcd_sh8601`** component (SH8601 is CO5300-register-compatible) on **SPI3_HOST** (dedicated QSPI pins D0-3 = 38/39/42/45, SCK 40, CS 41, RST 37), through esp_lcd's QSPI panel IO (`quad_mode`, `use_qspi_interface`). The 22-column GRAM offset is applied via `esp_lcd_panel_set_gap(panel, 0x16, 0)`.
- The panel samples **RGB565 big-endian** (high byte first); host pixels are byte-swapped before transmission (`co5300.c`).
- The panel requires **even x/y draw boundaries**; fills/shapes must use even-sized windows (48-row bands; 2-row circle bands).
- Power: AXP2101 ALDO2 (display rail, 3.3 V) + XL9555 P7 `VCI_EN` high.

## Peripherals

| Peripheral | Part | Bus |
|---|---|---|
| Power management | X-Powers AXP2101 | I2C `0x34` |
| GPIO expander | Xinluda XL9555 | I2C `0x20` |
| Smart sensor / IMU | Bosch BHI260AP | I2C `0x28` |
| RTC | NXP PCF85063A | I2C `0x51` |
| Haptic driver | TI DRV2605 | I2C `0x5A` |
| Touch panel | CST9217 | I2C `0x1A` |
| LoRa transceiver | Semtech SX1262 (868/915/920 MHz) | SPI |
| NFC reader | ST ST25R3916 | SPI |
| SD card | — (FAT, up to 32 GB) | SPI |
| Audio amp | Analog MAX98357A (3.2 W Class D) | I2S |
| Microphone | TDK T3902 | PDM (I2S) |
| GNSS | u-blox MIA-M10Q | UART |

## Pin map

### I2C bus (SDA GPIO3, SCL GPIO2)

| Device | GPIO | Addr |
|---|---|---|
| Touch CST9217 | — | 0x1A |
| Expander XL9555 | — | 0x20 |
| Sensor BHI260AP | INT=8 | 0x28 |
| PMU AXP2101 | IRQ=7 | 0x34 |
| RTC PCF85063A | INT=1 | 0x51 |
| Haptic DRV2605 | — | 0x5A |

### SPI bus (MOSI GPIO34, MISO GPIO33, SCK GPIO35)

| Device | CS | Notes |
|---|---|---|
| SD card | 21 | SD detect via XL9555 GPIO12 |
| LoRa SX1262 | 36 | RESET=47, BUSY=48, IRQ=14 |
| NFC ST25R3916 | 4 | IRQ=5 |

### Display QSPI

| Signal | GPIO |
|---|---|
| CS | 41 |
| DATA0 | 38 |
| DATA1 | 39 |
| DATA2 | 42 |
| DATA3 | 45 |
| SCK | 40 |
| TE | 6 |
| RESET | 37 |

### Other

| Function | GPIO |
|---|---|
| GNSS MIA-M10Q TX | 43 |
| GNSS MIA-M10Q RX | 44 |
| GNSS PPS | 13 | 1PPS output (rising edge); used to discipline the RTC — see [`manual.md`](manual.md#time-source). |
| Audio MAX98357A BCLK | 9 |
| Audio MAX98357A WCLK | 10 |
| Audio MAX98357A DOUT | 11 |
| PDM mic T3902 CLK | 17 |
| PDM mic T3902 DATA | 18 |
| Touch panel INT (CST9217) | 12 |
| Button / download mode | 0 |
| PWR button | AXP2101 (PWRKEY) |

### XL9555 expander outputs

| Expand GPIO | Function |
|---|---|
| GPIO6 | Haptic driver enable (M_EN) |
| GPIO7 | Display power supply enable (VCI_EN) |
| GPIO8 | Touchpad reset (TP_RST) |
| GPIO10 | SD insert detect (SD_DET) |
| GPIO11 | LoRa RF switch select (LORA_SEL / SKY13453 VCTL) |

> Verified on hardware with the `verify_pins` diagnostic:
> - **TP_RST = P8** — driving it low NACKs the CST9217 touch controller on I2C
> - **SD_DET = P10** — reads 0 with an SD card seated, 1 without
>
> This matches the
> [arduino-esp32 variant](https://github.com/espressif/arduino-esp32/blob/master/variants/lilygo_twatch_ultra/pins_arduino.h)
> (DRV_EN=6, DISP_EN=7, TOUCH_RST=8, SD_DET=10). The LilyGO hardware
> doc/schematic listing TP_RST=P10, SD_DET=P12 is incorrect.
>
> **LORA_SEL = P11** — cross-verified against the schematic (net "LORA_SEL"
> feeding the SKY13453 RF switch's VCTL pin) and LilyGO's own reference
> firmware (`LilyGoLib`'s `LilyGoWatchUltra.h`: `EXPANDS_LORA_RF_SW = 11`).
> HIGH selects the built-in LoRa antenna (the normal/only end-user path);
> LOW instead routes the RF path out via the USB-C connector's SBU pins
> (LilyGO's "USB LoRa interface" - not a real antenna path, not used here).
> **Only toggle this pin while the SX1262 is powered down** (before ALDO3 is
> enabled, or after it's disabled) - hot-switching an RF path while the
> chip is actively driving it is not something this switch is meant to
> handle. `twatch_board_init()` sets it during the XL9555 setup step,
> before `axp2101_set_default_power()` enables ALDO3 - keep it there if the
> init order is ever restructured.

### SX1262 LoRa notes (verified on hardware)

- The onboard LoRa module (schematic ref "HPB16B3") has **no external crystal** visible on the schematic (unlike the ESP32's own crystal or the RTC's 32.768kHz crystal, both drawn explicitly) - it uses an internal **TCXO at 3.0V**, powered via the SX1262's DIO3 pin (`SetDIO3AsTcxoCtrl`, voltage code `0x06`). Confirmed against LilyGO's own reference firmware (`LilyGoLib`'s `examples/radio/SX1262/SX126x_Receive.ino`: `radio.setTCXO(3.0)`). Without this, the chip has no working 32MHz reference for any frequency-dependent operation (PLL, RF frequency, RX/TX) - symptoms are a cold-boot `XOSC_START_ERR` device error and a receiver that reports a flat, pinned-at-minimum RSSI floor (no real RF energy ever reaches the demodulator).
- **DIO2 drives the antenna TX/RX switch** (`SetDIO2AsRfSwitchCtrl(enable=true)`), separate from the SX1262's own antenna-path relative to the board-level `LORA_SEL` XL9555 pin above. Also confirmed against the same LilyGO reference firmware (`radio.setDio2AsRfSwitch()`) and the SKY13453 RF switch visible on the schematic next to the antenna connector.
- Both are configured once, early, in `sx1262_configure_lora()` (`components/drivers/sx1262/sx1262.c`), before any packet-type/frequency/modulation setup.

## AXP2101 power tree

| Channel | Rail |
|---|---|
| DC1 | ESP32-S3 |
| DC2–DC5 | Unused |
| LDO1 (VRTC) | GPS backup (cannot be off) |
| ALDO1 | SD card — **and part of the SPI2 bus power policy whenever a card is seated**, see Notes |
| ALDO2 | Display |
| ALDO3 | LoRa — also registered in the SPI2 bus power policy as a precaution, see Notes |
| ALDO4 | Sensor |
| BLDO1 | GNSS |
| BLDO2 | Speaker |
| DLDO1 | NFC |
| VBACKUP | RTC button battery |

## Power

| Parameter | Value |
|---|---|
| USB-C input | 3.9–6 V |
| Charge current | 0–1024 mA (programmable; keep below 500 mA) |
| Battery | 1100 mAh, 3.7 V (4.07 Wh) |

Deep sleep with power button + boot button: 1.1 mA; power-off keep-alive: 77 uA.

**ALDO1 stays on for as long as a card is seated**, including across light
sleep, so a seated card's idle draw is now permanent rather than paid only
while the filesystem is mounted. That is the cost of keeping SPI2 readable —
see Notes. The delta has **not been measured on this board**; an idle SD card
is typically a fraction of a milliamp, but treat that as an expectation to
verify, not a figure. With an empty socket nothing changes: the rail is cut as
before.

## Notes

- SD card must be FAT/FAT32 formatted.
- **SPI2's power is managed centrally, not per-device.** SPI2 is shared by the
  SD card, the SX1262 and the ST25R3916, and a device that is physically
  present on that bus but electrically unpowered does not release it: its I/O
  pins can clamp the shared MISO net through their own ESD protection diodes,
  so *every read on SPI2 returns `0x00`* while that's true — not just reads of
  the unpowered device. Writes are unaffected, since MOSI is host-driven —
  which makes this fail as "the device is dead" rather than as a bus error.
  Confirmed on hardware for the SD card: `sd_log_unmount()` cuts ALDO1 only
  when the socket is empty (`twatch_sd_card_seated()`, XL9555 P10). LoRa's
  ALDO3 was assumed to behave the same way (the SX1262 module has a single
  VCC pin, no separate always-on I/O rail like the ST25R3916 has) but
  **measured not to** — NFC's identity register reads correctly with ALDO3
  off, as long as ALDO1 is on. The policy for each rail is owned by
  `components/spi2_power` (see `docs/adr/0005-spi2-bus-power-abstraction.md`),
  not scattered across each driver. This clamp mechanism was the root cause
  of the entire ST25R3916 bring-up saga; see `docs/nfc.md` for the full
  measurements.
- ST25R3916 (NFC) has no integrated capacitive presence detection — the reader must be enabled to detect cards.
- ESP32-S3 uses external QSPI flash and PSRAM (not in-package).
