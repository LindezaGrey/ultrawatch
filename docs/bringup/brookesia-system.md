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
