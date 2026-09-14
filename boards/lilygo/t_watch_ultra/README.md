# LilyGO T-Watch Ultra board adaptor boundary

This board description records the operator-confirmed V1.0 / SX1262 target.
The verified low-level implementation remains in `components/twatch_board`.
The Brookesia adaptor must reuse that owner for buses, display buffers, touch,
PMU telemetry, and XL9555-controlled display/touch enables.

## Verified interfaces

- CO5300: 410×502 QSPI; CS41, RST37, SCLK40, D0/1/2/3 = 38/39/42/45;
  active-area X offset 22.
- CST9220: I²C address 0x1A, native resolution 410×502; reset through XL9555
  P10, interrupt GPIO12.
- AXP2101: I²C address 0x34 on SDA3/SCL2.
- XL9555: candidate I²C address 0x20; display enable P07 and touch reset P10.

`board_devices.yaml` and `board_peripherals.yaml` are intentionally not added
yet. The stock Brookesia device entries assume direct GPIO reset and standard
LCD panel ownership; this board requires the existing custom QSPI and expander
paths. Adding guessed entries would create a second bus/device owner.
