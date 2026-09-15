# BHI260AP integration boundary

The T-Watch Ultra V1.0 mapping is now device-tested: AXP2101 ALDO4 is enabled
at 1800 mV and the sensor acknowledges I²C address `0x28`. The interrupt net is
GPIO8 (`HIRQ`), with the sensor side of RS0104 at VDD1V8 and the MCU side at
VDD3V3.

LilyGoLib revision `2eadcf7239a0e69fcfa37e6e8714eb611faa7b5c` initializes the
sensor through `SensorBHI260AP`, uploads a Bosch firmware image, and configures
a rising-edge interrupt. The local SensorLib is Arduino-oriented and bundles
about 26 MB of unrelated drivers; it is not an ESP-IDF component and must not
be added wholesale.

Because the Ultra build defines `USING_XL9555_EXPANDS`, LilyGo selects
`BOSCH_BHI260_GPIO`, whose pinned artifact is
`baseline/.pio/libdeps/twatch_ultra/SensorLib/src/bosch/firmware/Bosch_BHI260_GPIO.fw`
(119,716 bytes, SHA-256
`e82c8075534d30391b00d384afc0abe89b0c993c95df2797b333738f4f268b9e`).

The next implementation must isolate the Bosch BHI3/BHY2 transport and the
specific firmware image used by LilyGo behind a board-owned ESP-IDF component.
It must provide explicit I²C bus ownership, bounded task/queue behavior, GPIO8
ISR handoff, firmware-version/readback checks, and a disable path that leaves
ALDO4 unchanged unless explicitly requested. Until that component exists, the
native port performs only the validated rail preparation and read-only ACK
probe.

The native diagnostics now also reads Bosch BHY2 product ID register `0x1C`
and chip ID register `0x2B` after the ACK probe. These reads remain
configuration-free; the expected product ID is `0x89`. The build is ready for
device validation. On commit `65eaa2c`, the operator reported `P89 C70` on the
watch, confirming the expected product ID and chip ID `0x70`.
