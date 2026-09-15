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

The requested Bosch SensorAPI repository provides the generic artifact
`firmware/bhi260ap/BHI260AP.fw`. It is vendored at
`components/bhi260ap_core/BHI260AP.fw` (103,676 bytes, SHA-256
`318def511fd8eb762bf1e61cf02cfd6ecac70f8627d3bacfa7d24a1641f9e915`) and
embedded by the ESP-IDF component. Bosch's generated C representation is also
preserved as `components/bhi260ap_core/BHI260AP.fw.h`; parsing its byte array
produces the same 103,676-byte SHA-256. The LilyGo-specific GPIO image remains in
the historical SensorLib cache for comparison; it is no longer the selected
artifact for the native port.

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

The isolated `bhi260ap_core` ESP-IDF component now compiles the upstream Bosch
`bhy2`, `bhy2_hif`, and `bhy2_parse` core sources without Arduino wrappers or
unrelated SensorLib drivers. It is a compile-only boundary at this stage; the
transport callbacks, firmware loader, interrupt handoff, and runtime motion
stream are not yet connected.

The loader now uses the embedded Bosch `BHI260AP.fw` image to upload firmware
to sensor RAM, boot it, and verify the host firmware status bits. This path is
device-tested: the watch reported boot status `0x38` and continued into the
normal display, touch, RTC, and diagnostics application.

After a verified boot, the core reads Bosch parameter `0x011F`
(`BHY2_PARAM_SYS_VIRT_SENSOR_PRESENT`) and logs each available virtual sensor
ID from the 256-bit presence map. This inventory is read-only; it does not
enable or configure a sensor stream.

Device validation after flashing commit `e3c80cf` reported firmware boot
status `0x38` and 33 available virtual sensor IDs:
`1, 3, 4, 5, 6, 7, 10, 12, 13, 14, 15, 16, 28, 29, 31, 32, 37, 38, 48,
50, 52, 53, 55, 57, 59, 61, 63, 67, 69, 70, 75, 77, 94`.

The first ESP-IDF transport adapter is now device-tested. It connects the
Bosch BHY2 core to board-owned I²C callbacks, performs the documented Bosch
soft reset, verifies product ID `0x89`, uploads the firmware, and reads the
virtual-sensor presence map during boot. GPIO8 interrupt handoff and runtime
motion streams remain intentionally disabled; those are the next isolated
changes.
