# T-Watch Ultra hardware foundation

This branch contains the ESP-IDF hardware components and a minimal bring-up
application for the LilyGO T-Watch Ultra. It intentionally does not include the
existing watch UI, screen navigation, alarms, application services, or
connectivity policy. A future application can build on the board package and
initialize only the peripheral drivers it needs.

## Board bring-up

The `twatch_board` component creates the I2C, shared SPI2, and I2S channels,
registers bus device handles, establishes safe XL9555 control-pin levels,
initializes the AXP2101 power tree, and configures the shared-SPI rail policy.
It does not initialize sensor, haptic, audio, display, touch, radio, or NFC
chip drivers. Those components are available independently to the application.

The minimal `main/app_main.c` initializes NVS, brings up the board resources,
and initializes the RTC as a small example. It does not automatically start
the BHI260AP or any other optional peripheral. An application that chooses to
initialize BHI260AP must mount storage containing `/assets/bhi260/BHI260AP.fw`
first.

## Peripheral components

For a bus-level picture of how the peripherals connect to the ESP32-S3, see
[Peripheral connectivity overview](docs/peripherals-overview.md).

`components/drivers/` contains the board's I2C, I2S, and SPI peripherals,
including AXP2101, XL9555, PCF85063A, DRV2605, MAX98357A, T3902, BHI260AP,
M10Q, SX1262, ST25R3916, CO5300, and CST9217. `components/meshtastic/` contains
the optional Meshtastic radio preset, packet crypto, and protobuf decoders;
the SX1262 component itself is chip-level only. `components/spi2_power/`
contains the shared-bus rail policy.

Some components retain their own app-facing concerns: BHI260AP includes
activity/day persistence and expects firmware from application-provided storage;
M10Q uses the vendored ubxlib GNSS stack. They are buildable components but are
not started by board bring-up. CST9217 exports the native `esp_lcd_touch` handle
and panel dimensions; input-adapter/UI glue belongs in the application.

## Build

With ESP-IDF 6.0.2 installed and exported:

```sh
idf.py build
```

The host-only, no-hardware logic tests can be run separately; see the test
README files under individual driver components.

No peripheral hardware validation is implied by a successful host build.
