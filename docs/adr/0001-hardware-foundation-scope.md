# Hardware foundation and application boundary

Status: accepted for the `hardware-foundation` integration branch.

## Decision

Keep reusable ESP-IDF peripheral components and the T-Watch Ultra board
resource/rail layer independent of any one UI/application. Board initialization
creates the I2C, SPI2, and I2S resources, registers bus devices, sets safe
XL9555 control outputs, initializes the AXP2101 power tree, and registers the
shared SPI2 power policies. Peripheral chip initialization is initiated by the
application, not implicitly by board setup.

A minimal bring-up app initializes NVS, the board resources, and the RTC as a
small example. It does not mount application assets or automatically start the
BHI260AP, M10Q, radio, display, touch, haptic, or audio peripherals. A future
application that enables BHI260AP must provide and mount its firmware asset
storage before initializing that sensor.

## Component boundaries

- The SX1262 component contains chip-level LoRa commands only. Meshtastic
  preset, packet crypto, and protobuf helpers live in a separate optional
  `meshtastic` component.
- The CST9217 component wraps the vendor `esp_lcd_touch` implementation and
  exposes its handle and native dimensions. LVGL or other input-adapter glue
  is application-owned.
- The board package may depend on the generic SX1262 interface for the
  SPI2-rail-needed callback, but not on Meshtastic.
- `spi2_power` remains board-specific shared-SPI rail policy and is initialized
  by the board package after PMU and expander bring-up.
- BHI260AP's activity/day persistence and M10Q's ubxlib GNSS integration stay
  within their optional peripheral components for now. A future application
  may decide to split those app-oriented facilities if it needs bare-metal
  sensor/receiver access.

## Consequences

The repository root is a small hardware-foundation ESP-IDF project with a
minimal `app_main`; application screens, services, connectivity, and UI assets
are not part of this branch's scope. Optional display/touch managed components
remain available as peripheral dependencies but do not require LVGL. Buildable
components and host logic tests do not constitute device verification; actual
peripheral operation still requires testing on the watch.
