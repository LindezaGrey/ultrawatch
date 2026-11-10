# CST9217 touch controller

The watch integrates the Waveshare `esp_lcd_touch_cst9217` driver via the
`cst9217` wrapper. The checked-in startup log confirms the controller metadata,
reported native resolution (410×502), touch-controller creation, and LVGL IRQ
registration. The driver provides single-point pointer input (the vendored
parser caps points at one); multi-touch gestures are not enabled.

## Software behavior and bounds

- The wrapper's checked raw-point callback is passed to LVGL when the input
  device is created. It validates 410×502 coordinate bounds and returns read/
  ACK failures to the adapter, which then emits a released frame instead of
  consuming stale vendor-cached coordinates. The geometry is the board's known
  panel size; the vendor constructor reports its resolution but this wrapper
  does not yet validate that report.
- Initialization cleans up the panel-I/O device if touch-driver creation fails.
  Init is one-shot; there is no runtime deinit/retry API after successful init.
- The LVGL adapter owns the touch interrupt callback. The power-management
  layer separately arms/disarms GPIO12 as an active-low light-sleep wake source;
  wake-configuration errors are logged.
- Reset is on XL9555 P8 and is owned by `touch_controller`/the board API. CO5300 recovery uses its separate GPIO37 reset and XL9555 P7 panel-VCI control; ALDO2 / connector `LCD_VDD` (which powers touch) remains on, and touch wake configuration is unchanged. The schematic supports this separation; functional touch behavior after display VCI cycling has not been device-tested.

Host tests cover project-owned resolution/coordinate bounds logic. They do not
exercise the vendor parser, I²C transactions, GPIO interrupt timing, or touch
wake behavior.

## Pending device verification

Before calling the current touch path device-verified, test on the watch:

1. Center and all four screen corners, confirming orientation, scale, and
   inclusive/exclusive edge behavior.
2. Press, hold, release, repeated taps, and a temporary I²C/invalid-response
   failure; confirm LVGL releases rather than sticking at the last point.
3. Touch wake from ordinary light sleep, touch suppression in night mode, a
   touch held across sleep entry, and repeated sleep/wake cycles.
4. Touch reset and retry behavior after an induced init failure, if recovery is
   expected to be supported in the product.

`logs/serial.log` establishes startup/configuration only; it does not include
coordinate samples or touch-originated wake traces. No device test has yet been
run for this revised custom-read path.