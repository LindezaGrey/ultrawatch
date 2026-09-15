# BHI260AP interrupt handoff

The BHI260AP firmware and virtual-sensor presence map are device-tested. The
next implementation will add the verified HIRQ path from the schematic:

- sensor HIRQ: ESP32-S3 GPIO8
- electrical path: RS0104 level translator, sensor side at 1.8 V and MCU side
  at 3.3 V
- handoff: GPIO ISR records only an event and wakes a bounded task; all I²C and
  BHY2 FIFO work stays in task context
- failure behavior: queue overflow is counted and logged; the ISR never blocks
  or performs I²C

The first runtime check will configure no sensor stream. It will verify GPIO8
idle level, host interrupt status reads, bounded event delivery, and clean
shutdown of the task and GPIO handler. After that gate passes, the test stream
will use corrected accelerometer virtual sensor ID 4 at 1 Hz with zero report
latency. The latest x/y/z sample will be cached for diagnostics and logged;
other sensors remain disabled.

Source evidence: the T-Watch Ultra V1.0 schematic labels U7 HIRQ on GPIO8 and
routes the signal through U8 RS0104. See `docs/hardware-matrix.md` and the
schematic copy recorded there.

The handoff was device-tested after flashing commit `ba52dd0`: boot completed,
the GPIO8 input reported idle level `1`, and the watch remained responsive for
the capture interval. No interrupt events were expected because no virtual
sensor stream was enabled.

The first accelerometer stream trial (commit `425f37e`) also booted and kept
the application responsive, but produced no FIFO callback samples during the
capture. In that image GPIO8 reported idle level `0`. This leaves interrupt
polarity/mode and FIFO status unresolved; the stream gate is therefore not
passed and no motion data is yet claimed as device-tested.

After the callback-table fix (`b0a7953`) and FIFO fallback poll (`ba70ef0`),
the stream is device-tested. The watch reported interrupt status `0x0b`, then
delivered repeated ID 4 callbacks with stationary samples such as
`x=1378 y=-230 z=3870`; FIFO processing returned `0` throughout the capture.
Using Bosch's `1/4096 g` scale, that sample is approximately
`(+336, -56, +944) mg`, consistent with a stationary watch measuring gravity
on its Z axis. The Diagnostics UI presents the converted milli-g values.
