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
