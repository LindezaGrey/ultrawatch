# Display controller and touch ownership

Status: phases 1-6 are implemented: initial controller, controller-owned boot
initialisation, first-frame gating, normal light-sleep entry/exit,
controller-serialized diagnostics/recovery, and CST9217 ownership by
`touch_controller`. The touch-controller handoff (normal gestures and
light-sleep touch wake) is hardware-validated. Ultra-Sparmodus and shutdown
are routed through explicit display-controller sleep modes.

## Goal

Give the physical display one owner. `display_controller` owns the CO5300's
complete lifecycle: initialisation, sleep/wake, visibility, brightness
application, repaint coordination, recovery, and the display-rail recovery
path. No other production or diagnostic code sends a CO5300 lifecycle command.

This replaces the current hand-written sequences in `power_mgmt.c`,
`lvgl_app.c`, and `debug_cmds.c`. It deliberately favors reliable visibility:
every normal light-sleep wake requests a visible display. IMU-only panel
deferral is removed in this pass.

## Ownership

```
power_mgmt       policy: sleep, night mode, effective brightness, touch-wake permission
       |                                      |
       v                                      v
touch_controller                         display_controller
CST9217 reset/init + GPIO12 wake         CO5300 lifecycle + repaint coordination
       |                                      |
       +------------> LVGL adapter <---------+
                         |
                         v
                 UI gestures and navigation
```

`twatch_board` owns only buses, pins, and narrow board primitives. It must not
initialise the CO5300 or make its lifecycle decisions. The board provides a
display-rail setter for explicit controller-directed recovery only; normal
sleep keeps the existing ALDO2/touch policy.

`touch_controller` is a sibling boundary, not part of `display_controller`:

- it owns CST9217 reset/init and arming/disarming GPIO12 as a wake source;
- `power_mgmt` decides whether touch wake is permitted, including night mode;
- the LVGL adapter consumes touch samples, and the UI owns gestures and navigation;
- current CO5300 recovery resets panel GPIO37 and/or cycles XL9555 P7 `VCI_EN`;
  it does not switch AXP2101 ALDO2 / connector `LCD_VDD`, pulse CST9217 reset
  (XL9555 P8), or change touch-wake policy (GPIO12). The schematic separates
  those signals, so the display recovery path should not power-cycle touch. If
  future recovery needs to reset or power-cycle touch, it must coordinate
  through `touch_controller` rather than manipulate those resources directly.

`touch_controller` is deliberately a service rather than a task: touch reset
and startup occur once, and wake configuration runs synchronously as part of
the existing sleep-entry sequence. New display-controller code must not take
over touch IRQ or gesture handling, and new power-management code must not add
direct CST9217 or GPIO12 wake control.

## Display-controller contract

`display_controller.[ch]` exposes:

```c
esp_err_t display_controller_init(void);
void display_controller_attach_lvgl(void);
void display_controller_request_visible(display_reason_t reason);
void display_controller_request_repaint(void);
void display_controller_note_draw_complete(void);
void display_controller_set_brightness(uint8_t level);
esp_err_t display_controller_sleep(TickType_t timeout);
display_state_t display_controller_get_state(void);
esp_err_t display_controller_recover(display_recovery_t recovery, TickType_t timeout);
```

The request APIs are non-blocking and safe from any task, including the LVGL
flush callback. `display_controller_sleep()` and recovery are synchronous only
because sleep entry and console diagnostics need a completed transition before
they proceed.

States are:

```
INITIALIZING -> READY -> AWAITING_DRAW -> VISIBLE
                       ^                   |
                       |                   v
                    SLEEPING <-------------+

Any state -> RECOVERING -> AWAITING_DRAW | FAULT
```

`READY` means the panel is initialised, blank, and output-off. `AWAITING_DRAW`
means it is ready to accept pixels but must remain output-off until a real draw
completes. `FAULT` records an unrecoverable command failure and is visible through diagnostics. A visible request from `FAULT` retries panel reset, reinitialisation, blanking, and brightness before requesting a full redraw.

Brightness is controller state, but brightness policy is not: `power_mgmt`
persists normal brightness, decides the night-mode override, and submits the
effective value. A value submitted while asleep is cached and applied during
the next show transition.

## Lifecycle and concurrency

The controller task consumes a command queue and is the sole caller of:

- `co5300_init()`, `co5300_reinit()`, `co5300_sleep()`;
- display on/off, blank, wake, and brightness operations;
- CO5300 register diagnostics and explicit display-rail recovery.

The LVGL flush path remains the sole normal producer of pixel transactions.
It is not a lifecycle owner.

### Initial boot

1. `display_controller_init()` runs before LVGL display registration. It
   initializes the CO5300 with output off, blanks it, and reaches `READY`.
2. `lvgl_app` registers the stable CO5300 handle and starts the adapter.
3. `display_controller_attach_lvgl()` enables LVGL-coordinated transitions.
4. Once the boot screen is built, the app requests visibility.
5. The first successful bitmap draw posts `display_controller_note_draw_complete()`.
   The controller then sends `DISPON` and enters `VISIBLE`.

`co5300_init()` needs an output-off option, parallel to the existing reinit
option, so boot cannot flash an undefined frame.

### Show, wake, and recovery

For a show request from `SLEEPING`, `READY`, or `FAULT`, the controller:

1. takes `esp_lv_adapter_lock()` before commands that can race a flush;
2. reinitializes with hardware reset and output off;
3. blanks GRAM and applies cached effective brightness;
4. releases the lock and requests a full LVGL redraw;
5. waits for the post-draw notification, then sends `DISPON`.

Duplicate visible/repaint requests are coalesced. The custom bitmap callback
posts the draw-complete notification only after its successful draw call has
returned; it never calls CO5300 lifecycle functions itself. This prevents panel
IO from re-entering an in-flight flush and prevents reset garbage becoming
visible.

On normal light-sleep exit, `power_mgmt` always requests visible display and
then resumes the adapter. The old pending/deferred-panel flags and
`power_mgmt_panel_wake_if_pending()` are removed.

### Sleep and shutdown

Before light sleep or Ultra-Sparmodus, `power_mgmt` calls
`display_controller_sleep()` and waits for completion. The controller locks
against LVGL flushing, performs `DISPOFF`, blank, and `SLPIN` in that order,
then reports completion. Shutdown calls the explicit
`display_controller_power_off()` mode, which omits `SLPIN` because the PMIC
power-off sequence immediately removes panel power.

### Diagnostics

Existing display commands remain useful but route through controller requests:

- repaint, safe output/brightness commands, reset/reinit, and register reads
  are serialized by the controller;
- `disppwr` becomes explicit controller rail-cycle recovery;
- `dispcycle` uses serialized sleep/show transitions;
- the intentionally unsafe `dispcycle ... flush` experiment is removed;
- TE GPIO observation stays a read-only diagnostic outside lifecycle control.

## Migration and acceptance

1. Add the controller and change CO5300 initialisation to support output-off.
2. Move initial panel ownership from `twatch_board` to the controller.
3. Integrate LVGL registration, redraw completion notification, and first-frame
   gating.
4. Replace all lifecycle calls in power management and diagnostics, then remove
   deferred wake machinery.
5. Add the narrow board display-rail primitive and route recovery through it.
6. Implement `touch_controller` separately; migrate CST9217 setup and GPIO12
   wake arming without changing LVGL gesture behavior.

Verification requires:

- host tests for state transitions, request coalescing, cached brightness,
  draw gating, recovery, and failure-to-`FAULT` behavior;
- `idf.py build`, existing host driver tests, and simulator build;
- hardware checks for cold boot, timeout/wake via touch/buttons/RTC/alarm/mesh,
  night mode, repeated sleep/wake, and each safe diagnostic command;
- a static search confirming lifecycle `co5300_*` calls exist only in the
  controller and CO5300 driver.

The implementation is successful only if a draw is authoritative: any real
LVGL draw eventually makes the display visible, while no panel lifecycle IO can
run from a flush callback or race a panel reset.

## Navigation and live inactivity settings

`main/screens/navigation.c` is shared with the SDL simulator. It owns the
four-screen ring, Apps launcher, parent navigation and return-home policy.
Its injected monotonic clock includes sleep time; `power_mgmt_exit_sleep()`
calls `navigation_resume()` under the LVGL lock before requesting the first
visible frame. Settings and unfinished editors survive normal display sleep.
An automatic return never reports display activity.

`power_mgmt` owns the live display deadline (10 s default, persisted `disp_to`)
and return setting (30 s default, persisted `return_to`). Explicit activity
reports only update its timestamp under a critical section. A 100 ms LVGL
timer feeds the adapter while the display deadline has not elapsed. The
adapter uses a fixed 250 ms final idle gate, then the existing PAUSE callbacks;
sleep entry rechecks the owned deadline after the adapter's flush wait. Thus
sleep takes roughly the configured interval plus up to 250 ms of scheduling
overhead, without modifying the managed adapter component or restarting it.
The LVGL timer stops with the adapter during sleep; wake starts a fresh display
interval. USB keep-awake and foreground NFC/alarm holds use this same policy.

Normal sleep no longer touches GNSS power: its worker retains ownership of
BLDO1 and may acquire in the background. Active tracking retains the IMU stream
with the display off. Ultra-Sparmodus still owns its full shutdown sequence.

Host verification: `cmake -S sim -B sim/build`, `cmake --build sim/build`, then
`ctest --test-dir sim/build --output-on-failure`. The navigation test drives
real LVGL pointer input with a controllable clock; physical sleep remains a
firmware/hardware check. Screenshots use `UWATCH_SIM_SCREEN=apps` or
`settings_disp` with `UWATCH_SIM_SHOT`, as for the existing screen previews.

GNSS acquisition is bounded independently of display sleep: `gps_ctrl_task`
allows five minutes without adequate reception and caps no-fix acquisition at
ten minutes. Position callback timestamps use `esp_timer_get_time()`, not LVGL
uptime. The task polls instead of waiting inside a long acquisition loop, so
queued manual requests are handled throughout acquisition. It owns GNSS
preference writes and shutdown; automatic cutoff saves the switch off.
The pure reception policy is covered by the `gps_reception` CTest target.
