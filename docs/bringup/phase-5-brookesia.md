# Phase 5 — Brookesia shell preparation

The T-Watch Ultra board HAL currently provides the verified LVGL display and
CST9220 touch path. The next layer must reuse those interfaces instead of
initializing buses or devices a second time.

The selected Brookesia snapshot is `2c09459a97a82c8d477c543eb2b43f4956b8a5be`.
The repository is componentized into `brookesia_hal_interface`,
`brookesia_hal_adaptor`, and `brookesia_hal_boards`; the board adaptor is
expected to consume board configuration and register the HAL interfaces.

## Current status

- Board display/touch path: device-tested.
- LVGL partial redraw with Ultra invalidation rounder: device-tested.
- Brookesia source/components: not present in the workspace.
- Brookesia-to-IDF compatibility: **UNVERIFIED** for the pinned snapshot and
  this project's ESP-IDF 6.0.2 environment.

## Required next change

Fetch the exact pinned Brookesia source or registry component lock, inspect the
closest Waveshare AMOLED board adaptor, and add only the minimal T-Watch Ultra
board configuration/adaptor. Keep Wi-Fi, audio, GNSS, NFC, SD, sensors, and
LoRa disabled until the shell passes the one-hour lifecycle test.

No Brookesia dependency or board API is guessed in the firmware.
