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
- Brookesia source/components: pinned as a submodule at
  `2c09459a97a82c8d477c543eb2b43f4956b8a5be`.
- Brookesia HAL interface and utility components: **build-tested** with
  ESP-IDF 6.0.2; the application binary links successfully.
- T-Watch Ultra provider bridge: **device-tested** with a delayed probe after
  normal UI startup.

The isolated bridge image was flashed and captured with elevated USB access.
Device boot evidence is **device-tested**: ESP-IDF 6.0.2 reached `READY`, the
CO5300 reported `410x502 QSPI`, LVGL initialized, and health samples remained
stable. The capture also reports 16 MB flash, 8 MB PSRAM, and battery voltage
4171 mV. The serial device may be invisible to sandboxed commands; use elevated
access for hardware logs.

The delayed probe then initialized `TWatchUltra`, registered and acquired both
`TWatchUltra:CO5300` and `TWatchUltra:CST9220`, released both handles, and
logged `BROOKESIA_HAL_PROBE panel_touch=ready` at about 16 seconds. Subsequent
health and PMU samples continued normally.

The provider-backed LVGL image was then flashed and **device-tested**. The
operator confirmed the UI is visible and the touch button works. Serial output
shows the Brookesia interfaces acquired before LVGL initialization, followed
by `READY heap_integrity=ok`; no reset or corruption was observed.

A 60-second elevated serial soak is also **device-tested**: health samples
advanced from roughly 0 to 5988, internal free heap stayed at 276884 bytes,
PSRAM free stayed at 8384132 bytes, and no reset, assertion, or flush error
appeared. The full one-hour lifecycle gate was intentionally skipped by
operator decision; the shorter soak and interactive device tests remain the
available evidence.

The board layer now exposes explicit CO5300 hardware inversion (`0x21`/`0x20`)
for dark-background operation. It is enabled in the current build as a
power/burn-in mitigation experiment; the effect and current draw require
physical confirmation after flashing.

The inverted image is **device-tested**: the operator confirmed the display
renders correctly and touch remains functional after flashing. A current-draw
measurement is still required before quantifying the power benefit.

## Next step

The minimal board bridge and provider-backed LVGL path are complete. The next
change should add the smallest Brookesia application shell or navigation demo,
while continuing to delegate hardware ownership to `twatch_board`. Keep
Wi-Fi, audio, GNSS, NFC, SD, sensors, and LoRa disabled until that shell is
stable. The one-hour lifecycle gate is waived by operator decision; preserve
the shorter soak and interactive device evidence.

The bridge now registers `TWatchUltra:CO5300` and `TWatchUltra:CST9220`.
Panel draws delegate to `twatch_display_flush`; touch remains polling through
`twatch_board_read_touch`. No interrupt line or second bus owner is claimed.

No Brookesia dependency or board API is guessed in the firmware.

The pinned source was inspected locally. Its HAL adaptor (`0.8.4`) initializes
devices through `esp_board_manager` and the board package (`0.8.0`) supplies
standard LCD/touch YAML entries. The Ultra's proven path cannot use those
entries unchanged: CO5300 writes use the custom QSPI transaction sequence and
both display enable and touch reset are XL9555 outputs. A custom bridge is
therefore required before enabling the stock display/touch adaptor.

The temporary geometry diagnostic was disabled again after the mask photo was
captured. The restored UI-default image was flashed and the operator confirmed
that the LVGL screen is visible and touchable. This keeps the board HAL and
rendering path accepted while Brookesia components are evaluated separately.

## Redraw distortion diagnosis

Early LVGL builds showed diagonal or sideways corruption only in newly updated
text regions; the static full-screen quadrant and geometry patterns were clean.
This isolated the fault to partial-update geometry rather than panel timing,
color order, or QSPI signal integrity. The CO5300 Ultra path requires the
LilyGO invalidation rounder: force `x1` to zero for narrow invalidations and
apply the panel's even-coordinate adjustments to `x2`, `y1`, and `y2`. With
that callback enabled, the operator confirmed clean counter and `Touches`
redraws. The RGB565 swap and transaction-size changes were retained for
LilyGO compatibility but were not the root-cause fix.
