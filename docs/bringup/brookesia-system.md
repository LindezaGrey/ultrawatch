# Brookesia System Super integration

The project resolves the pinned ESP-Brookesia 0.8.x System Super and LVGL
components from `vendor/esp-brookesia`. `main/brookesia_system_entry.cpp`
contains the smallest upstream-shaped startup sequence: service manager,
LVGL display source, and `system::super::System` over the registered T-Watch
panel/touch HAL.

The current firmware attempts this sequence once and falls back to the
validated native LVGL shell if any stage fails; this keeps a single LVGL owner.

Validation: ESP-IDF reconfigure and full build succeed with 29 resolved
dependencies, including `brookesia_system_super` 0.8.3 and
`brookesia_gui_lvgl` 0.8.4. The panel-I/O integration image flashed and
verified on the physical watch.

The first System Super runtime flash caused the USB serial/JTAG device to
disappear immediately after reset. Recovery firmware restored the native LVGL
owner while retaining the compiled integration. The latest device log shows
continuous RTC and BHI260AP samples, battery readings near 4.17 V,
`reset_reason=11`, and `heap_integrity=ok`.

## Current runtime blocker

The latest device log still reports `BROOKESIA_STAGE current=3` because the
pinned Display service requires Storage. The board HAL publishes panel and
touch only, so Storage fails with `Storage KV HAL interface is not available`
and `Storage file-system HAL interface is not available`; DisplaySource then
logs `Failed to bind Display service` and the application falls back to native
LVGL. This is a service dependency issue, not a CO5300/QSPI rendering fault.

Next step: add a board-owned NVS-backed key/value Storage HAL and verify that
the Display service binds before enabling the full Brookesia shell.

## Display recovery after panel-I/O experiment

Operator reported a black display after commit 5950eba; previous serial health
was not evidence of visible display success. Recovery removes the second SPI
device and disables the System Super startup attempt. The second device claimed
hardware CS on GPIO41, while the validated renderer controls that pin manually;
this ownership conflict is the suspected regression (visual retest pending).
The Storage dependency failure occurs before LVGL adapter registration, so the
new panel-I/O integration was never validated by the Brookesia runtime.

Recovery validation: ESP-IDF 6.0.2 build passed, binary 0x203890 bytes;
flash hashes verified. Serial capture records CO5300 ready, LVGL ready,
READY heap_integrity=ok, and ongoing RTC/BHI260AP telemetry. See
`2026-09-15-display-recovery-serial.log`. Operator visual confirmation pending.

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

Build-tested: final ESP-IDF 6.0.2 build passed; see 2026-09-15-storage-build.txt.
No device deployment performed for the Storage preparation.

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

Build-tested: ESP-IDF 6.0.2 build passed; evidence in
2026-09-15-service-probe-build.txt. Awaiting device deployment.
