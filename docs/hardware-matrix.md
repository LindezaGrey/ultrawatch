# T-Watch Ultra V1.0 hardware matrix

Review date: 2026-09-14. Operator identifies the watch as **V1.0, SX1262**.
Except for the limited device evidence under Test status below, physical
identities, voltages and behavior remain **UNVERIFIED on device**. Schematic evidence below describes
the published V1.0 design, not a measurement of the connected board.

## Evidence and naming

- **S**: [V1.0 schematic, 5 sheets](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/c4a10b29b05c95983f2310eaa2a3c5802044dcba/schematic/T-Watch%20Ultra%20V1.0%20SCH%2025-07-24.pdf).
  All five sheets rendered and visually reviewed. The filename says 25-07-24;
  sheet title blocks contain an older 2021 date. Use the pinned file and V1.0
  marking for provenance, not the title-block date as a manufacturing date.
- **H**: [LilyGO hardware notes](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/c4a10b29b05c95983f2310eaa2a3c5802044dcba/docs/hardware/lilygo-t-watch-ultra.md).
- **V**: [Official PlatformIO variant pins](https://github.com/Xinyuan-LilyGO/LilyGoLib-PlatformIO/blob/df59195707a9684d829a41d665a2ef20626c8e73/variants/lilygo_twatch_ultra/pins_arduino.h).
- **L**: [LilyGoWatchUltra.cpp](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/c4a10b29b05c95983f2310eaa2a3c5802044dcba/src/LilyGoWatchUltra.cpp)
  and [QSPI implementation](https://github.com/Xinyuan-LilyGO/LilyGoLib/blob/c4a10b29b05c95983f2310eaa2a3c5802044dcba/src/LilyGoDispInterface.cpp).
- **T**: [LilyGO third-party driver bundle](https://github.com/Xinyuan-LilyGO/LilyGoLib-ThirdParty/tree/c8e6de3b4684df60101569cfe8ead8adbbb79643).
- **B**: [Brookesia Waveshare AMOLED 2.16 reference](https://github.com/espressif/esp-brookesia/tree/2c09459a97a82c8d477c543eb2b43f4956b8a5be/hal/brookesia_hal_boards/boards/waveshare/esp32_s3_touch_amoled_2_16).

`GPIO` denotes an ESP32 pin. XL9555 `P10` means port 1 bit 0 and therefore
**linear software index 8**. `P12` means index 10. Never translate P10 to the
decimal index 10. This corrects the ambiguous "IO10 touch reset" wording in
the initial conversational plan.

Native I2C0 is assigned by the Phase 2 readout patch (initial USB readout
device-tested). Other native controller instances remain unassigned. The
schematic establishes nets, not ESP-IDF software controller numbers. The
official QSPI implementation selects SPI3_HOST for the watch; preserve this
as baseline evidence without treating it as a schematic requirement.

## Essential devices

| Device / published part | Bus and signals | Power / control | Driver and dependencies | Evidence / limitations |
| --- | --- | --- | --- | --- |
| MCU U21, ESP32-S3 | Native USB: GPIO19 D-, GPIO20 D+; GPIO0 BOOT; CHIP_PU reset | AXP2101 DC1 -> VDD3V3 | Native ESP-IDF 6.0.2 after Phase 0 | S2; exact die revision UNVERIFIED; no boot tests |
| Flash U9, W25Q128JVPIQ | Dedicated memory SPI, 128 Mbit / 16 MB by part designation | VDD_SDIO | IDF flash support; mode/frequency must match baseline detection | S2; physical JEDEC identity/capacity UNVERIFIED |
| PSRAM U12, APS6404L-3SQR-ZR | Quad SPI, 64 Mbit / 8 MB by part designation | VDD_SDIO | QSPI PSRAM, not Waveshare's octal configuration | S2, V/board JSON; 8 MB reported by device diagnostic; mode/frequency not independently measured |
| Shared I2C | SDA GPIO3, SCL GPIO2; R3/R4 2.2 kohm pull-ups to VDD3V3 | VDD3V3 | One board-owned bus; bounded transactions | S1/S3/S4; native I2C0 configured at 100 kHz; PMU reads device-tested, clock not instrument-measured |
| PMU U1, AXP2101 | I2C `0x34` per H; IRQ GPIO7 | VSYS/VBUS/VBAT inputs; DC1 powers MCU | Baseline XPowersLib 0.3.1; native checked adapter after identity validation | S1, H, L; native ID 0x4a, USB/charging status and battery ADC 4038–4040 mV device-tested; external voltage check pending |
| Expander U22, XL9555QF24 | I2C `0x20`; P06/index6 M_EN, P07/index7 VCI_EN, P10/index8 TP_RST, P12/index10 SD_DET, P13/index11 LORA_SEL | VDD3V3 | Baseline SensorLib 0.3.3; minimal native masked read/modify/write adapter | S3, V; essential for display/touch; IRQ net MCU_INT routing to MCU UNVERIFIED, unused in initial port |
| AMOLED module, CO5300 per H/L | QSPI: CS GPIO41, CLK40, D0 38, D1 39, D2 42, D3 45; RST37, TE6 | ALDO2 -> LCD_VDD; XL9555 P07 -> VCI_EN | Baseline LilyGoDispQSPI; native esp_lcd_co5300 2.0.2 candidate; PMU and expander first | S1/S4, H, L; connector does not identify panel silicon; 410x502 and controller identity UNVERIFIED on device |
| Touch, CST9220 (reported chip `0x9220`) | Shared I2C; `0x1A` device-tested; fallback `0x5A`; IRQ GPIO12; reset XL9555 P10/index8 | Display connector LCD_VDD; exact internal module supply topology UNVERIFIED | Native vendor-sourced identity read; coordinate driver pending | S4, H, L; address, chip ID, 410x502 resolution and firmware `0x01000003` device-tested; coordinate range/wake behavior UNVERIFIED |

## Deferred inventory

These devices receive no project-owned functional integration until the board
milestone passes. The unmodified vendor baseline may initialize them.

| Device / published part | Bus and GPIOs | Power / reset / enables | Reference driver; dependencies | Evidence / status |
| --- | --- | --- | --- | --- |
| RTC U4 PCF85063ATL | I2C `0x51` per H; IRQ GPIO1; application interpretation UTC | VBACKUP, MS621FE backup cell | SensorPCF85063; shared I2C | Address and register map supported by local `baseline/.pio/libdeps/twatch_ultra/SensorLib/src/REG/PCF85063Constants.h`; fitted chip, voltage/time/IRQ UNVERIFIED |
| Motion U7 BHI260AP | I2C `0x28`; IRQ GPIO8 (`HIRQ`); RS0104 level translator U8; SDA/SCL shared on IO3/IO2 | AXP2101 ALDO4 -> VDD1V8; translator B-side VDD3V3; RESETN pulled to VDD1V8 | LilyGo `LilyGoWatchUltra.cpp` (`2eadcf7`) and SensorLib examples use GPIO8 with a rising-edge IRQ | Schematic `T-Watch Ultra V1.0 SCH 25-07-24.pdf`, LilyGo Ultra docs/source, ACK at 0x28, and device FIFO/IRQ samples confirm the mapping. GPIO8 is verified; occasional missing edges are an interrupt delivery/configuration issue, not an alternate pin. |
| GNSS U2 MIA-M10Q | UART MCU TX43/RX44, PPS GPIO13; UART instance UNVERIFIED | BLDO1 -> GPS_VDD; VRTC backup; no speculative backup disable | GPS/TinyGPSPlus; supply and UART | S1/S2/S5, V; UART direction verified by MCU net names; operation UNVERIFIED |
| microSD | Shared SPI MOSI34/MISO33/CLK35, CS21; detect P12/index10 | ALDO1 -> SD_VDD | SD; SPI and rail | S1/S2/S3, V; no card/mount/format tests |
| Haptic U11 DRV2605 | I2C `0x5A` per H | VDD3V3, enable P06/index6 | SensorDRV2605; I2C and expander | S3, H; exact suffix/effects/current UNVERIFIED |
| Speaker U27 MAX98357A | I2S BCLK9/LRCLK10/DIN11; I2S instance UNVERIFIED | BLDO2 -> SPK_VDD; SD_MODE circuit on S5 | Baseline audio support; no native selection yet | S1/S5; audio/current UNVERIFIED |
| Microphone U6 MMIC T390200012 | PDM CLK17/DATA18; peripheral instance UNVERIFIED | VDD3V3; channel-select assembly option | Baseline PDM; no native selection yet | S4; channel assembly and capture UNVERIFIED |
| NFC U10 ST25R3916 | Shared SPI MOSI34/MISO33/CLK35, CS4, IRQ5 | DLDO1 and VDD3V3 domains; exact safe sequencing UNVERIFIED | ST25R3916-fork 1.1.0, NFC-RFAL-fork 1.0.1 | S1/S4, H; radio operation deferred |
| Radio U3, schematic module HPB16B3; operator says SX1262 | Shared SPI MOSI34/MISO33/CLK35; CS36, RESET47, BUSY48, IRQ14 | ALDO3 -> LORA_VDD; RF switch select P13/index11 | RadioLib 7.4.0 in baseline; module identity/RF-switch policy must be checked before later use | S3/S5, V; exact module marking and frequency UNVERIFIED |
| Buttons | BOOT GPIO0; reset CHIP_PU; power PMU PWRON | Pull-ups and switches per S1/S2 | Boot/reset baseline only; product button behavior deferred | S1/S2; physical behavior UNVERIFIED |

## Conflicts and implementation consequences

1. **Expander numbering:** H uses GPIO10/GPIO12 names; V uses indices 8/10.
   S3 confirms these are P10/P12. Use explicit port/bit and linear index in
   native constants. Display/touch require Phase 2 expander support; BHI260AP
   is not required for these pins on the published V1.0 schematic.
2. **Display dimensions, reconciled in source:** V stores 502x410. The QSPI
   driver's `setRotation(0)` swaps these to width 410, height 502. This agrees
   with H and the operator's panel specification. Preserve the official
   variant; physical rendering remains a Phase 0 gate.
   The operator confirms the panel presents as visibly high-DPI; UI layout
   should use logical sizing and scaled typography rather than treating each
   physical pixel as a user-facing layout unit.
3. **Display offset, reconciled in source:** Although the board constructor's
   `setGapOffset(22, 0)` call is commented out, `setRotation(0)` sets gap (22,0)
   for the stored width 502. Subsequent draw windows add this offset. This
   explains the init sequence's columns 22..431. Native edge-pattern testing
   must still verify the effective panel window before claiming device success.
4. **Touch address:** L probes 0x1A and otherwise tries 0x5A. H also lists
   DRV2605 at 0x5A. An ACK alone cannot identify touch. Require controller
   identity/protocol validation, especially for the fallback address.
5. **Unrelated board reference:** B uses different pins, 480x480 resolution,
   octal PSRAM, and a touch address configuration of 0xB4. Its configuration
   cannot be copied to this watch; determine its address convention separately.
6. **Power defaults:** L enables unrelated rails, writes gauge calibration,
   and configures charging. Preserve it only as an official baseline. The
   native port must have an audited minimal write list and measured results.
7. **Display sleep:** L warns against removing ALDO2 in deep sleep due to
   increased current. This is vendor evidence, not our measurement. Initial
   display-off retains rails; MCU sleep/shutdown remain separate future work.

## Test status

Schematic/source review completed. The first official-baseline display/input
check is **device-tested**: operator reports Hello World and an increasing
button counter; serial captured two button events. The flasher detected
ESP32-S3 revision v0.2 and a 40 MHz crystal. A subsequent read-only esptool
flash-identity command detected **16 MB**, manufacturer EF, device 4018, and
quad flash configuration. The fitted flash package suffix remains UNVERIFIED.
The separate diagnostic reports `psram_found=1 psram_bytes=8388608`, confirming
8 MB initialized PSRAM. This is not a comprehensive memory-integrity test.
I2C ACKs were observed at 0x1A, 0x20, 0x28, 0x34, 0x51 and 0x5A;
these do not by themselves prove which silicon occupies each address.

For the deferred motion sensor, exact board evidence is now recorded from the
cached LilyGoLib revision `2eadcf7239a0e69fcfa37e6e8714eb611faa7b5c`:
`docs/hardware/lilygo-t-watch-ultra.md` names BHI260AP at address `0x28` and
GPIO8 as its interrupt, while the checked-in Ultra schematic
`T-Watch Ultra V1.0 SCH 25-07-24.pdf` labels U7 as BHI260AP, routes SDA/SCL to
the shared IO3/IO2 bus, routes HIRQ to IO8, and shows ALDO4 feeding VDD1V8
through the RS0104 level translator. LilyGo's `LilyGoWatchUltra.cpp` loads
Bosch BHI260 firmware and attaches a rising-edge interrupt on GPIO8. These
sources establish the board mapping; they do not yet justify enabling ALDO4 or
uploading firmware in the native port.

The native rail reader now also reads AXP2101 ALDO4 enable bit 3 at register
`0x90` and its voltage code at `0x95`, using the pinned XPowersAXP2101
definitions (`ALDO4` range 500–3500 mV, 100 mV steps). The change is strictly
read-only and build-tested. Device validation on commit `6d93ea6` reports ALDO4
enabled at 1800 mV.
The new BHI preparation path follows LilyGo's documented sequence by setting
ALDO4 to 1800 mV and enabling it, with masked writes and readback verification;
this is the first native PMU write for the sensor rail.

The native diagnostics worker now performs a read-only ACK probe at BHI260AP
address `0x28` once per second and reports `ACK` or `absent`. It does not read
sensor registers, upload Bosch firmware, enable interrupts, or change PMU state.
The probe is build-tested and awaits device validation.
Device validation on commit `6d93ea6` reports `BHI_PROBE candidate=0x28
present=1`; chip identity and firmware acceptance remain unverified.
The native identity read was device-tested on commit `65eaa2c`: product ID
`0x89`, chip ID `0x70`. Bosch firmware acceptance and runtime sensor data remain
unverified.

Ten controlled resets each reached the I2C scan in a 20-second capture, with
the same three known errors and no new fault messages; the operator reports
Hello World on observed restarts. Three battery-only shutdown/startup and touch checks on the shutdown diagnostic
passed by operator observation; current measurements, coordinate samples and
display photographs are not recorded yet. Startup logged a vendor constructor-order error before
reaching the UI. See [Phase 0](bringup/phase-0.md) for logs and the source-level
diagnosis. Phase 0 remains incomplete; no pin-level verification is inferred
solely from a working UI.

Power-button issue: the original baseline did not shut down after a 6-10-second
PWR hold with USB disconnected. See the [shutdown fix](bringup/power-button.md). Readout confirms long-press shutdown
was disabled (`0x22=0x04`) and both button IRQs work. The authorized fix is
deployed and readback confirms `0x22=0x06`, enabling four-second power-off.
The operator confirms that a five-second hold now turns the screen dark and
it stays dark for at least ten seconds on battery. All three battery-only restarts restored Hello World and the working touch
counter. Repeated shutdown worked after battery startup, supporting setting
persistence across the tested cycles; post-cycle register readback and shutdown
current remain unverified. No battery-startup serial evidence was captured.

A focused [display initialization-order patch](bringup/display-startup.md) is
build-tested and host-tested in the separate `display_startup` diagnostic. It
preserves geometry and delays the rotation hardware command until SPI exists.
The authorized upload is device-tested with one 20-second boot capture: all
three startup errors are absent, PSRAM reports 8 MB, and PMU shutdown remains
enabled (0x06). The operator confirms Hello World and the touch counter still
work. Ten subsequent reset traces on this exact firmware passed strict checks with
no errors, 8 MB PSRAM, and enabled shutdown. The operator confirms Hello World
after every reset and three successful battery shutdown/startup cycles with
working touch on this exact build. Battery-startup serial logs and current
measurements remain unavailable. Vendor archive sources are unchanged.

Native [Phase 1](bringup/phase-1.md) is build-tested using IDF 6.0.2 with
DIO flash at 80 MHz, 16 MB flash size, quad PSRAM at 80 MHz, and USB Serial/JTAG.
Physical sizes are checked at boot and PSRAM startup testing is enabled. Native
operation passed one 35-second USB boot: detected sizes match, PSRAM startup
test passed, and seven heap samples are stable. Ten native reset trials now pass with startup PSRAM testing and stable heap
samples. Native cold boots remain UNVERIFIED; no display, touch, PMU, or radio driver is integrated. This target introduces no new peripheral pin assignments.

A subsequent read-only USB serial capture confirms native HEALTH samples 21-24
with stable heap while the display is black. The native target intentionally
has no display driver. No boot/reset-cause evidence was captured, so this
observation is not counted as a native cold boot.

First native cold-boot capture missed startup/reset-cause evidence and was not
counted. A build-tested firmware update repeats boot evidence for late USB
capture; deployment and its device validation are pending. No pin or PMU
configuration change is included.

The boot-evidence update is now deployed and device-tested: three USB-arrival
cold-start captures each report POWERON (1), passed startup checks and stable
heap. Reset trials on the updated image remain pending. Early boot logs were
not captured; firmware repeats the established startup results.

Final native Phase 1 gate: **passed** on image `93be72d2…7a74aa3`, with ten
consecutive resets and three verified POWERON cold starts on the same image.
See [full hash and combined device results](bringup/2026-09-14-native-boot-evidence-device.json).
All captured memory/heap/watchdog checks passed; cold startup facts were read
from repeated firmware reports. Current consumption and native PMU/display/
touch operation remain unverified.

Phase 2 read-only adapter is build-tested and host-tested. It verifies AXP2101
ID 0x4a at 0x34 before decoding status, battery ADC and IRQ registers, using
I2C0 SDA3/SCL2 at 100 kHz. All PMU configuration writes are deferred; disabled
ADC produces unavailable voltage. See [evidence and limits](bringup/phase-2.md).
Native PMU device validation and measured current remain pending.

Initial native PMU USB readout passed: identity 0x4a, battery/USB present,
charging active, ADC enabled and voltage 4038–4040 mV. Six health samples are
stable; no PMU configuration writes. See [device record](bringup/2026-09-14-native-phase2-device.json).
Battery-only operation, measured current and essential rail setup remain open.

Battery-only observation update (prepared, not deployed) retains sampled PMU
voltage/status in RAM for readback after USB return; resets clear the record.
No new pin, PMU or rail configuration is introduced. Physical validation is
pending; see [Phase 2 procedure](bringup/phase-2.md).

Native battery-only run device-tested on `0.2.1-battery-evidence`: 14 samples
span 65 seconds, battery ADC 4158–4161 mV, no charging samples, and retained RAM
evidence survives USB reconnection with healthy runtime reports. See
[battery result](bringup/battery-run-1-20260914T122854Z.json). Current, temperature,
long-duration stability and essential rail/expander state remain unverified.

Prepared read-only rail audit adds DC1/ALDO2 setpoint/enable reads and XL9555
candidate output, polarity and direction registers. P07 display enable and
P10 (port1 bit0/index8) touch reset are decoded separately as direction and
latch. No input-port reads or configuration writes. Build/host-tested; native
state capture pending. See [rail evidence](bringup/rail-audit.md).

Read-only native rail audit is device-tested: DC1 and ALDO2 enabled at 3300 mV
setpoints; expander candidate 0x20 has output ff/ff, polarity 00/00 and direction
ff/ff. P07 and P10 therefore remain inputs, with high latches. Actual voltages
and pin levels are not measured. See [audit result](bringup/2026-09-14-rail-audit-device.json).
No PMU or expander setting was changed.

Prepared native essential IO setup targets only XL9555 P07 (port0 bit7) and
P10 (port1 bit0/index8): preload high, then output, with masked writes and full
readback. DC1/ALDO2 must already be enabled at 3300 mV; no PMU writes.
Build/host-tested; see [procedure and power limits](bringup/essential-io.md).

Native essential IO setup **device-tested on USB**, version `0.2.3-essential-io`:
P07/P10 direction readback changes ff/ff → 7f/fe, output latches remain ff/ff,
and all audited PMU settings are unchanged. Five health snapshots are stable.
This proves register configuration; physical pin levels/current are unmeasured.
Native panel and touch drivers remain absent; Phase 2 gate remains incomplete.
See [device result](bringup/2026-09-14-essential-io-device.json) and
[procedure/power conditions](bringup/essential-io.md).

Essential IO firmware `0.2.3-essential-io` also passes short battery continuity:
12 battery-only samples span 55110 ms (4164–4167 mV); the RAM record survives
USB reconnection without reboot. P07/P10 output/direction and PMU setpoints are
retained in five reconnection snapshots. Physical voltages, current, temperature
and longer-duration acceptance remain unverified. See
[battery evidence](bringup/battery-run-1-20260914T125844Z.json).

## Display recovery after panel-I/O experiment

Operator reported a black display after commit 5950eba; previous serial health
was not evidence of visible display success. Recovery removes the second SPI
device and disables the System Super startup attempt. The second device claimed
hardware CS on GPIO41, while the validated renderer controls that pin manually;
this ownership conflict is the suspected regression (visual retest pending).
The Storage dependency failure occurs before LVGL adapter registration, so the
new panel-I/O integration was never validated by the Brookesia runtime.

## NVS Storage HAL preparation

`components/twatch_storage` isolates the pinned upstream NVS key/value backend
(commit 2c09459a97a82c8d477c543eb2b43f4956b8a5be); provenance and local
changes are recorded in its PROVENANCE.md. Initialization errors return without
automatically erasing NVS. Existing nvs partition: offset 0x9000, size 0x5000.
Uses ESP-IDF nvs_flash; no GPIO, external bus, PMU rail, or expander dependency.
The board bridge publishes TWatchUltra:NVS. Its one-time startup probe initializes
the backend, lists uw_probe, and deinitializes it. It sets/deletes no keys;
NVS initialization itself may maintain flash metadata. Backend calls follow
upstream's lifecycle and must be serialized by the owning Storage service.

The native UI remains selected. Device validation pending: STORAGE_PROBE init=ok
list=ok, visible native UI/touch, and healthy telemetry. Persistent round-trip
and reset retention are UNVERIFIED. A KV backend satisfies Storage initialization
but does not supply the filesystem needed for System Super resources. Display
binding and the panel adapter contract still need separate validation.

Storage probe device result (firmware e1ba453): flash hashes verified;
STORAGE_PROBE init=ok list=ok entries=0 kv_writes=0. LVGL ready and
heap_integrity=ok observed after startup. See
`docs/bringup/2026-09-15-storage-serial.log`. No KV write/retention or
Display-service binding test yet; operator screen/touch confirmation pending.

## Storage-dependent Display service probe

Next build starts the service manager, binds Display (which requires Storage),
releases the binding, and stops/deinitializes the manager before native LVGL
starts. This exercises service lifecycle without creating an LVGL adapter or
claiming hardware CS. SERVICE_PROBE logs binding success, cleanup completion,
internal heap before/after, and heap integrity. Native touch starts only after
service workers stop. Device results remain pending; inspect service logs and
confirm native display/touch after cleanup. No persistence claim follows from
this probe. System Super remains disabled.

Service binding device result (647218e): flash verified; STORAGE_PROBE init=ok
list=ok; SERVICE_PROBE display_bind=ok. Display and Storage stopped and
manager cleanup completed before LVGL ready. Internal heap before=222376,
after=192636 (29,740 bytes less free); heap_ok=1. Subsequent native HEALTH
samples remain at 188008 bytes free during the short capture. Retained memory
is unresolved; this single cycle cannot establish a leak or lifecycle stability.
Wi-Fi service initialization fails because its HAL is absent; it did not block
Display binding. Filesystem and full LVGL adapter/System Super remain untested.
Evidence: docs/bringup/2026-09-15-service-binding-serial.log.
Operator visual/touch confirmation pending for this image.

## Repeated service cleanup diagnostic

Operator confirmed screen and touch after 647218e. Next build performs at most
three service bind/release/stop/deinit cycles before native LVGL. SERVICE_CYCLE
logs internal heap before, immediately after cleanup, and after a 100 ms
sampling interval, plus task counts and heap integrity. The interval observes
possible idle-task reclamation; it is not a synchronization fix. Stops on a
failed bind/integrity result. Compare settled values across cycles to distinguish
first-use retention from recurring growth; background tasks introduce noise.
The 29,740-byte observation remains unexplained until measured. No hardware
control or flash is part of this preparation.

Service-cycle device result (dcb12b0): flash verified; all three bindings pass.
Internal heap before/immediate/settled: cycle1 222376/192636/192636;
cycle2 192636/192636/192636; cycle3 192636/192536/192536. Task counts
7/7/7 each cycle; heap integrity passes. The 100 ms observation did not recover
the first-cycle 29,740 bytes. This large delta does not recur in these three
cycles, consistent with first-use retention, but allocation ownership remains
unproven. The additional 100 bytes in cycle3 are unresolved; no leak-free claim.
Native LVGL starts and subsequent heap samples are stable at 187908 bytes.
Evidence: docs/bringup/2026-09-15-service-cycles-serial.log.
Visual/touch confirmation pending for this image. Next investigation: attribute
retained allocations with heap tracing or JTAG before declaring cleanup complete.

## First-cycle allocation trace preparation

Operator confirmed screen/touch after dcb12b0. Next diagnostic enables IDF
standalone heap tracing, depth 6, with 2048 records allocated in PSRAM before
the first-cycle baseline. Tracing covers first-cycle init/bind/cleanup and the
100 ms sampling interval, then stops and dumps surviving internal allocations.
Summary includes capacity/high-water/overflow. Overflow invalidates completeness;
PSRAM trace records exclude ISR allocations (per IDF esp_heap_trace.h).
Concurrent sensor and logging tasks can contribute records. Surviving allocations
are candidates, not automatically leaks. Trace buffer is detached/freed before
native UI. Decode PCs using the exact matching ELF and addr2line; retain build
identity alongside the serial trace. Tracing alters timing and heap accounting;
compare attribution rather than demanding identical deltas to dcb12b0.
No automatic flash; allocation owners remain unverified until device capture.

## Allocation trace device evidence and cleanup correction

Original d7349f5 trace: 408 surviving records, peak 1567/2048, overflow=0.
Decoded first boot contains 376 internal records requesting 23,193 bytes:
string allocation stacks 11,797 bytes; FunctionParameterSchema vectors 7,488;
EventItemSchema vectors 1,716; remaining runtime/other allocations 2,192.
Requested payload sizes exclude allocator overhead and cannot be equated directly
to the 29,740-byte free-heap delta. Display helper get_function_schemas and
get_event_schemas use static materialized arrays (service_helper/media/display.hpp
lines 209-225). This supports first-use schema retention; six stack frames do
not identify the exact owner of every string. No claim that every byte is explained.
Original trace and matching-ELF symbolization are stored in
2026-09-15-allocation-trace-{serial.log,decoded.txt}.

The diagnostic itself aborted at heap_trace_init_standalone(NULL,0): IDF 6.0.2
header documents detachment but heap_trace_standalone.c rejects NULL or zero.
Recovery rebinds to a static one-record buffer, starts/stops an empty trace to
reset record links, then frees the PSRAM buffer. Built and flashed successfully;
recovery capture records overflow=0, cycles 2/3 unchanged at 190872 bytes free,
LVGL ready, and stable native heap=186244 with heap_integrity=ok. See
2026-09-15-trace-cleanup-recovery-serial.log. Screen/touch confirmation pending.
Tracing is temporary diagnostic overhead and should be disabled before normal
UI integration. Allocation ownership remains partly unresolved, not an observed
29 KiB-per-cycle leak. ISR allocations excluded and capture is short.

## Trace retirement and adapter prerequisites

Operator confirmed display and touch on the trace-cleanup recovery image.
Temporary heap tracing is now removed from the application and disabled in
sdkconfig.defaults. Three service binding/cleanup cycles and heap integrity
sampling remain; System runtime remains disabled. This preparation is not a
new device test.

Pinned adapter inspection identifies these requirements before enabling runtime:
- The LVGL9 OTHER bridge rejects null panel_io even with custom drawing and
  IDF callback registration disabled (lvgl_bridge_v9.c, bridge creation).
  Do not restore the second hardware-CS device that stole GPIO41 routing.
  Select a supported IO integration that preserves the single renderer owner.
- Successful custom drawing does not call flush_ready automatically
  (display_bridge_v9_flush_default). DisplaySource forwards ColorTransDone
  through its Panel event dispatcher. The current synchronous HAL must supply
  an appropriate completion path; verify task/ISR context before wiring it.
- Preserve main/lvgl_app.c rounder_cb, including left-edge expansion and even
  coordinate bounds. Generic alignment metadata alone does not express this.
- The adapter swaps RGB565 in the OTHER default flush path. Do not also call
  the native flush callback's byte swap.
- Establish one LVGL tick/task/input owner and install the watch rounder before
  the first refresh; do not run native initialization alongside adapter ownership.

Sources: pinned vendor GUI display_source.cpp submit_frame, panel_event_callback,
start_lvgl_adapter; managed adapter LVGL9 bridge; native lvgl_app.c.
Next functional patch must address these together, then build before requesting
a device test. No panel IO, power, display timing, or touch wiring changed here.

Trace-retirement build passed on pinned ESP-IDF 6.0.2. Artifact identity:
2026-09-15-trace-retirement-build.txt. No flash or new device validation.

## Adapter ownership preparation

CO5300/CST9220 hardware configuration unchanged. The board now supplies borrowed
synchronous ESP LCD wrappers over the existing software-CS41 SPI3 device;
async callbacks and unvalidated control operations are unsupported. The watch UI
moves to the pinned LVGL adapter worker, preserving the board rounder and HAL
input/drawing. System Super/DisplaySource remain disabled pending their separate
handoff. See `docs/bringup/brookesia-system.md`, "Adapter-owned watch UI
preparation", for ownership, completion contract and required device checks.
This image has no device result yet.

Adapter image 429e37c flashed and starts successfully; flushes advance,
short-run heap remains stable at 179120 bytes and worker stack margin >=3828
bytes in the capture. Visual/touch confirmation pending. See
`docs/bringup/2026-09-15-adapter-owned-ui-serial.log` and bring-up notes.
