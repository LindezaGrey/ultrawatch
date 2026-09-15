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
shutdown of the task and GPIO handler. A single low-rate virtual sensor stream
will be added only after this handoff passes on the watch.

Source evidence: the T-Watch Ultra V1.0 schematic labels U7 HIRQ on GPIO8 and
routes the signal through U8 RS0104. See `docs/hardware-matrix.md` and the
schematic copy recorded there.
