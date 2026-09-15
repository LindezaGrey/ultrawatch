# Brookesia System Super integration

The project now resolves the pinned ESP-Brookesia 0.8.x System Super and LVGL
components from `vendor/esp-brookesia`. `main/brookesia_system_entry.cpp`
contains the smallest upstream-shaped startup sequence: service manager,
LVGL display source, and `system::super::System` over the registered
T-Watch panel/touch HAL.

The function is deliberately not called by the current firmware. The validated
native LVGL shell remains the device baseline while this stack is build-tested;
calling it alongside the native LVGL loop would create two LVGL owners. The
next integration change will make System Super the sole UI owner, then remove
the temporary native shell after a device test.

Validation so far: ESP-IDF reconfigure and full build succeed with 29 resolved
dependencies, including `brookesia_system_super` 0.8.3 and `brookesia_gui_lvgl`
0.8.4. No device flash was performed in this step.

The first System Super runtime flash caused the USB serial/JTAG device to
disappear immediately after reset. Recovery firmware restores the native LVGL
owner while retaining the compiled integration. Direct elevated serial capture
after recovery shows continuous RTC and BHI260AP samples, battery readings near
4.17 V, `reset_reason=1`, and `heap_integrity=ok`. The System Super startup
sequence therefore remains build-tested but not device-tested.
