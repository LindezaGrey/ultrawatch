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

Repeated-cycle diagnostic build passed on ESP-IDF 6.0.2; see
2026-09-15-service-cycle-build.txt. Device measurements pending.

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

Trace build passed; exact ELF/binary hashes in 2026-09-15-service-trace-build.txt.
Use xtensa-esp32s3-elf-addr2line -pfiaC -e build/twatch_ultra_phase1.elf
with allocation PCs after verifying this ELF hash. No flash performed.

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

## Adapter-owned watch UI preparation

The next image runs the existing Home/Menu/Diagnostics/Sensors UI under the
pinned esp_lvgl_adapter 0.6.4. System Super and upstream DisplaySource remain
disabled: this validates adapter ownership before changing the shell and its
service lifecycle. The three-cycle service probe still completes first.

The board exports process-lifetime borrowed esp_lcd panel/IO interfaces over
its existing SPI3 device. Panel draw and inversion delegate to the established
board operations; IO commands and synchronous 0x2c color transfers share the
same transport. Unsupported async callback registration, deletion, rotation and
power operations are rejected. No extra SPI device is created, no GPIO routing
changes, and the pixel transaction loop retains 0x32/0x002c00 with continuous
software CS41 and 16,384-pixel chunking. Callers must serialize these interfaces;
they do not add a second locking or ownership layer.

Application ownership:
- Adapter initializes LVGL and owns its tick timer and sole rendering worker.
  Main no longer calls lv_tick_inc or lv_timer_handler.
- Registration, custom draw callback, rounder, pointer input and all UI objects
  are installed before esp_lv_adapter_start. Only the worker touches UI objects
  afterward; the diagnostics worker supplies mutex-protected snapshots.
- Adapter uses its pinned defaults: 8192-byte internal stack, priority 6,
  unpinned task, 1 ms tick, 1–15 ms worker delay; automatic sleep disabled.
  These are initial upstream settings, not a device-validated optimal schedule.
- RGB565, rotation 0, 16-row internal draw buffer, single buffering, no TE mode.
  The adapter packs stride and swaps RGB565 exactly once. The existing watch
  invalidation rounder is unchanged and registered before the first refresh.
- Custom draw routes through Brookesia Panel HAL. On synchronous success it
  calls lv_display_flush_ready on the LVGL task. On failure it returns an error;
  the pinned adapter releases that flush. No ISR notification API is called.
- ADAPTER_HEALTH reports successful flush count and worker stack free bytes
  every five seconds from that worker. Main retains watchdog/heap evidence.

No upstream/dependency patches. To later use DisplaySource, add a supported
pre-start board-configuration hook and synchronous completion handling there;
its current event-dispatch contract is not satisfied merely by these handles.

Device test after authorized flash: confirm owner=adapter startup, advancing
ADAPTER_HEALTH with usable stack margin and healthy main heap; verify UTC ticks,
Menu/Back, Diagnostics counter and changing sensor readings. Look specifically
for the former narrow-redraw shift and corrupted full-page navigation. Capture
logs and operator visual/touch result. Build success alone does not validate
scheduling, display or touch. No new power state or current-consumption claim.

Adapter-owned UI build passed on pinned ESP-IDF 6.0.2. See
2026-09-15-adapter-owned-ui-build.txt for exact artifact identity and source
comparison checks. Dependency lock only adds the existing adapter as a direct
dependency; no versions changed. Not flashed; all device checks above remain pending.

## Adapter-owned UI device capture (429e37c)

Flashed over /dev/ttyACM0 at 921600 baud; esptool verified data hashes and reset
via RTS. Firmware identity matches 2026-09-15-adapter-owned-ui-build.txt.
The bounded 55-second serial capture ended normally via timeout (exit 124).
Startup confirms `LVGL ready owner=adapter ... sync_flush=1`. Service binding
cycles pass with heap integrity; first-use internal heap falls 238696 -> 208956,
cycle2 unchanged, cycle3 208856. Main health samples from 18.59–53.63 seconds
report stable internal_free=179120 and heap_integrity=ok. Adapter health from
23.43–53.43 seconds advances 57 -> 510 successful flushes; smallest observed
worker stack margin is 3828 bytes. RTC and accelerometer updates continue.
No crash, assertion, or watchdog error observed in this short capture.
Evidence: 2026-09-15-adapter-owned-ui-{flash.log,serial.log}.

Device-tested: upload, startup, continuing adapter execution and short-run heap
observations. Operator display/touch confirmation remains pending; serial
flushes alone do not establish visual correctness. Longer-run stability and
power consumption are not established. Next: verify clean UTC/navigation/
Diagnostics/Sensors on the physical screen before System Super integration.

## Adapter panic reported after initial capture

Operator reports a crash after Diagnostics -> Touch me twice -> Back on 429e37c.
Follow-up serial reports reset_reason=4 (ESP_RST_PANIC), whereas deployment
capture reports reset_reason=11. This confirms an intervening panic-related
restart. The panic/backtrace itself preceded capture and is unavailable; do not
attribute the cause to stack, display timing, or navigation without evidence.
The first captured repeat shows continuously increasing uptime and successful
flushes, stable internal heap=179220, worker stack margin at least 3812 bytes,
and no new panic. Back currently intentionally loads Home, not Menu.
Evidence: 2026-09-15-adapter-panic-followup.log and
2026-09-15-adapter-first-repro.log. Further serial reproduction capture armed;
no reset, debugger operation, source fix, or reflash performed during this
investigation. The adapter stability gate is NOT passed; the earlier short-run
success does not supersede this reported and reset-cause-confirmed failure.
Next: capture a matching-ELF panic backtrace on recurrence, using JTAG if needed.

Operator subsequently reports the crash cannot be reproduced. Extended capture
continues through uptime 258.696 s with internal_free=179220, heap_integrity=ok,
3133 successful flushes and minimum observed worker stack margin 3812 bytes.
No further panic observed. Logger stopped cleanly by terminating its timeout
process (exit 143); watch left running without reset or flash. Evidence:
2026-09-15-adapter-extended-repro.log. Original panic remains unresolved. Retain
matching 429e37c ELF identity for a future panic backtrace; do not mark this fixed.

## Diagnostics Back latency instrumentation

Operator clarified the latest observation: Back from Diagnostics feels slow.
The earlier panic remains a separate unresolved event. This diagnostic records
only Diagnostics -> Home navigation, preserving the renderer, 16-row buffer,
rounder, navigation destinations and adapter scheduling.

One BACK_TIMING summary after the first nonempty Home refresh reports elapsed
microseconds: press_to_click (includes finger hold; -1 if unavailable), load_us
(lv_screen_load), wait_refresh_us (load return to REFR_START), refresh_us
(REFR_START to REFR_READY), draw_us (sum inside synchronous HAL calls),
max_draw_us, total_us (click callback entry to refresh ready), flush count,
pixel count and failed draw count. draw_us is a subset of refresh_us; timings
include task preemption. They do not measure AMOLED scanout or touch latency
before LVGL input dispatch. REFR_READY also occurs for empty refresh cycles;
the logger waits for at least one flush. All counters belong to the LVGL task;
no allocations or per-flush logging are added. Failure count must be zero before
interpreting the result as a completed successful transfer.

Device procedure after authorized flash: open Diagnostics, tap Touch me twice,
then tap Back normally; repeat a few times, including one deliberately held Back
press. Save BACK_TIMING and ADAPTER_HEALTH alongside any panic/backtrace. Compare
load/wait/refresh and HAL transfer contribution before choosing an optimization.
No latency improvement or cause is claimed from compilation alone.

Back timing diagnostic build passed. Exact hashes in
2026-09-15-back-timing-build.txt; ELF/binary also preserved locally under
.cache/firmware/back-timing for matching backtraces. Not flashed; physical
timing samples pending.

## Back timing device result (4f776e5)

Flashed at 921600 baud with verified hashes. Operator completed the requested
Diagnostics -> two taps -> Back trials, including a held Back press. Four
summaries were captured. Click-to-refresh-ready totals: 140.105, 141.638,
132.990, 133.737 ms. Load takes 0.178–0.378 ms; post-load refresh wait
0.076–0.161 ms. HAL drawing sums to 27.311–27.825 ms. Each transition sends
205820 pixels (410x502), in 32 flushes, with failures=0. The longest individual
HAL draw is 0.947 ms. The held trial records press_to_click=1063.359 ms, then
133.737 ms to completed synchronous refresh.

Inference: callback loading/scheduling and raw HAL drawing are not the majority
of the measured latency. Roughly 105–114 ms remains within the refresh cycle
outside HAL drawing, including rendering, buffer preparation and possible task
preemption; this is not a CPU-only rendering measurement. Panel scanout and
pre-dispatch touch latency remain unmeasured. No optimization is justified from
these results alone without distinguishing those remaining costs.

No panic observed during these trials. Recent health samples remain at
internal_free=179040 and heap_integrity=ok; worker stack margin >=3812 bytes.
This does not resolve the earlier intermittent panic. User confirms test
completion but does not explicitly rate perceived latency in the reply.
Evidence: 2026-09-15-back-timing-{flash.log,serial.log,results.json}. Serial
logger stopped after the test (exit 143); watch left running. Next: profile the
non-HAL part of full-screen refresh before changing buffers or display timings.

## Back refresh phase instrumentation

Extend the existing timing diagnostic with LVGL9 public display events; no
upstream patch or hardware-setting change. BACK_PHASES follows BACK_TIMING:
pre_render_us measures refresh start through layout/invalidation processing to
render start; render_wall_us spans rendering AND flushes; flush_wall_us sums
FLUSH_START/FINISH; adapter_pre_us measures flush start to custom HAL callback
entry (including packing/swapping); adapter_post_us measures HAL return to flush
finish (including completion/counter overhead); flush_wait_us and wait_calls
measure LVGL's flush-wait scopes; post_render_us measures render end to refresh
ready. render_other_us subtracts flush and flush-wait scopes from render wall
time. In the selected single-buffer partial synchronous path these scopes are
within rendering and nonoverlapping. Recheck that assumption if modes change.

Sources: pinned LVGL9.4.0 lv_refr.c refr_invalid_areas, call_flush_cb,
wait_for_flushing and refresh timer; adapter0.6.4 OTHER default flush. Public
render events enclose transfers, so render_wall_us must not be called pure
rendering time. All values include preemption, callback overhead and any internal
draw-task wait not exposed as a flush wait. No scheduler instrumentation is
added; CPU work versus preemption is still unseparated. Two aggregate logs are
emitted after completion, never per transfer. Only task-owned counters are added.

After authorized flash, repeat the same Diagnostics/touch/Back trials. Check
zero failures and nonnegative components, then compare phase sums with total
refresh time. Identify dominant scopes before optimizing. Prior latency results
remain baseline evidence, not validation of this new diagnostic image.

Back phase build passed; see 2026-09-15-back-phases-build.txt. Exact ELF and
binary preserved locally in .cache/firmware/back-phases. No flash performed.

## Phase timing deployment and startup PMU failure (9f25a27)

Flash hashes verified. First boot after RTS reset aborted at app_main line 60,
ESP_ERROR_CHECK(twatch_board_init()). Preceding error: I2C address 0x34 register
0x03 returned ESP_ERR_INVALID_RESPONSE during PMU identity read. This occurs
before LVGL initialization. Matching ELF cd323c472... decoded the backtrace to
this startup check; see 2026-09-15-back-phases-panic-decoded.txt. The automatic
restart succeeded and emitted LVGL ready owner=adapter followed by health logs.
No manual reset, recovery image, speculative PMU write or retry patch was used.
The reason for the I2C failure remains unverified. Do not conflate this captured
startup abort with the earlier operator-reported navigation panic, whose actual
backtrace was not captured. Boot reliability remains an open issue.

## Captured runtime panic and telemetry error handling

At uptime 113.756 s on 9f25a27, a periodic XL9555 read at 0x20/reg0x05 failed
with ESP_ERR_TIMEOUT. app_main line 152 ESP_ERROR_CHECK(twatch_board_read_rails)
aborted the watch. The exact-ELF backtrace confirms this path (see
2026-09-15-telemetry-panic-decoded.txt). This reproduces a crash during the
operator's navigation trials; it is not an LVGL exception. Whether the earlier
uncaptured panic had this same cause cannot be established.

The patch handles periodic PMU/rail read errors independently: count/log failures,
skip the incomplete snapshot, keep watchdog/heap reporting and UI running, and
sample again on the normal five-second schedule. Failed power reads do not enter
battery observations; failed rails do not print rail values. No immediate retry,
new bus lock, bus reset or regulator write. Essential startup checks are retained;
the separate startup identity-read failure remains unresolved.

Pinned IDF i2c_master.c s_i2c_synchronous_transaction already takes bus_lock_mux.
ESP_ERR_TIMEOUT can arise waiting for that lock or from the transaction itself.
Bus contention is therefore possible, not proven; the electrical/protocol reason
for either captured read failure still needs investigation. Adding an outer
mutex or increasing timeout without measurements is not part of this fix.

Host fault injection compiles the actual main-loop telemetry block with mocked
board readers: both succeed, power fails, rails fail, both fail, and subsequent
recovery. It checks independent snapshot suppression, battery sample validity,
failure counters, one call per reader, and no abort. This validates application
error handling, not physical I2C behavior. Run tools/test_telemetry_failure.py.

Three phase-timing trials before the panic measured click-to-refresh totals
129.659–139.018 ms. Render excluding flush/wait: 89.483–98.313 ms; whole flush
38.970–39.249 ms, including HAL 27.586–27.814 ms, adapter-pre 9.760–9.940 ms and
adapter-post 1.422–1.638 ms. Explicit flush waits total 0.688–0.710 ms. All three
send 205820 pixels in 32 flushes with zero transfer failures. Rendering-related
elapsed time dominates; task preemption and internal drawing waits remain inside
that category. See 2026-09-15-back-phases-{serial.log,results.json}.
Prioritize validating nonfatal telemetry behavior before optimizing rendering.

Telemetry recovery build and host test passed; exact identity in
2026-09-15-telemetry-recovery-build.txt, artifacts cached under
.cache/firmware/telemetry-recovery. Not flashed. After authorization, repeat
navigation with logs; on natural telemetry failure expect TELEMETRY_READ and
continuing HEALTH/ADAPTER_HEALTH without restart. Do not disconnect or short
I2C hardware to induce faults. No-failure trials alone do not device-validate
the failure branch.

## Telemetry recovery device result (b3f05aa)

Flash verified and first boot reached the adapter UI without startup abort.
At uptime 23.651 s and 53.731 s periodic power reads returned ESP_ERR_TIMEOUT;
TELEMETRY_READ logged power_failures=1 then 2, rails=ESP_OK, with incomplete
snapshots skipped. Main HEALTH and adapter execution continued on the same boot
(reset_reason=11), and later PMU reads succeeded. Through uptime 98.701 s,
recent internal_free remains 178960 and heap_integrity=ok; smallest observed
adapter stack margin is 3812 bytes, main 2536 bytes. No panic in this capture.
The operator explicitly confirms Diagnostics -> Touch me -> Back works and the
screen remains responsive. The logger was stopped (exit 143); firmware left
running. Evidence: 2026-09-15-telemetry-recovery-{flash.log,serial.log}.

Device-tested: recovery from natural periodic power-read timeouts, subsequent
successful samples, and operator navigation responsiveness. Rail-read failure
branch remains host-tested in this run (no natural rails error captured).
Underlying I2C failures and the earlier startup identity-read failure remain
unresolved; this validates nonfatal observation handling, not a repaired bus.
Next investigate timeout origin/contending callers without speculative PMU
writes, bus resets or timeout increases. Preserve the current responsive build.

## I2C timeout origin: pinned synchronous driver audit

The previous statement that ESP_ERR_TIMEOUT might also be a transaction error
was provisional. A complete return-path audit narrows it for this firmware:
read_device_regs -> i2c_master_transmit_receive -> s_i2c_synchronous_transaction
uses the synchronous branch (bus trans_queue_depth defaults to zero). In pinned
IDF 6.0.2 i2c_master.c, line 1010 returns ESP_ERR_TIMEOUT only after failure to
take bus_lock_mux within the configured timeout. s_i2c_transaction_start returns
ESP_ERR_INVALID_RESPONSE when command/status completion fails; its reset/clear
helper returns ESP_OK or ESP_ERR_INVALID_STATE, not ESP_ERR_TIMEOUT. PM is not
enabled. Thus the captured 0x107 runtime read errors are semaphore acquisition
timeouts on this exact path. This conclusion combines device error logs with
source inspection; it is not a new physical fault reproduction.

The semaphore is created with xSemaphoreCreateBinaryWithCaps, not a mutex with
priority inheritance. Current callers include main telemetry (priority 1),
diag_sample (priority 2), bhi_irq (priority 5) and lvgl (adapter default priority
6). They share one 100 kHz I2C0 bus. Which owner/activity delays main long enough,
and whether priority-related starvation occurs, still require measurement.
The separate startup 0x108 identity error is not classified as lock contention.
No new bus lock, task-priority change, timeout increase, reset or register write.

Prepared board diagnostic logs I2C_READ on read_device_regs failure or a
successful register read taking at least 50 ms (half the 100 ms timeout).
Fields: task name, priority, actual device address, register, elapsed microseconds,
result, and lock_timeout flag derived from this audited error path. Timings include
both lock wait and transfer; successful slow reads cannot be split further by
this wrapper. It covers telemetry/identity single-register reads, not direct
BHI FIFO bulk traffic or touch transfers, and does not identify the current
semaphore owner. Error address reporting also no longer labels every non-PMU
read as 0x20; RTC and BHI identities now report their own addresses.
Two timestamp reads are added per single-register transfer; only slow/error
cases log. Logging can perturb subsequent contention and is temporary evidence
instrumentation. Device verification pending; current responsive image retained.

Optional next JTAG check, using the matching ELF: inspect the failed semaphore
take branch and all task stacks, resume immediately and detach cleanly. Do not
use source line 1010 blindly: optimized GDB line information maps it to a success
instruction in the current ELF. Resolve the actual conditional failure branch
from disassembly for each build before setting a breakpoint.

I2C contention diagnostic build passed. See 2026-09-15-i2c-contention-build.txt;
matching ELF/binary cached under .cache/firmware/i2c-contention. No device control
or flash in this investigation. After authorization, capture I2C_READ during
repeated navigation and idle operation; retain the nonfatal telemetry behavior.

## I2C caller diagnostic device result (d08530f)

Flash verified, startup clean; operator repeated Diagnostics/touch/Back and
reports "looks fine, no crash". Capture through uptime 98.661 s contains no
I2C_READ slow/error marker, TELEMETRY_READ failure, or panic. Recent internal
heap remains 178960 with integrity checks passing; minimum observed adapter
stack margin 3812 bytes, main 2488 bytes. Logger stopped after test (exit 143),
watch left running on d08530f. Evidence:
2026-09-15-i2c-contention-{flash.log,serial.log}.

Device-tested: ordinary navigation and idle operation with instrumentation.
Not device-tested in this run: new error/slow-read logging path, because no
qualifying read occurred. Contention cause/delaying owner remain unverified;
no-failure capture does not establish a bus fix. Nonfatal telemetry handling
remains enabled and was separately exercised on the previous image. Retain this
build for a longer natural capture or JTAG investigation on recurrence; no
speculative hardware or priority changes justified by this run.

## Extended natural capture and prepared JTAG trap

Three-minute read-only serial capture on d08530f ended normally by timeout
(exit 124). Operator used the UI and reported completion; idle period followed.
One slow read was observed: uptime 268.771 s, task=main priority=1, PMU 0x34
register 0x00, elapsed_us=76932, result=ESP_OK, lock_timeout=0. There were no
telemetry timeouts or panics. Final health at uptime 353.721 s: internal_free
178960, heap_integrity=ok, main stack margin 2488 bytes. Successful slow-read
elapsed time combines semaphore wait, transfer and preemption; do not claim
76.932 ms was spent entirely waiting for the semaphore. Evidence:
2026-09-15-i2c-extended-serial.log. Firmware unchanged.

Prepared tools/i2c_lock_timeout.gdb and documented its one-shot use in
jtag-debugging.md. Exact d08530f ELF disassembly confirms failed semaphore-take
branch +168 at 0x4216d37c (movi a5,0x107). Script checks ELF hash before connecting,
captures task stacks on its first hardware breakpoint hit, then detaches/resumes.
A host negative test with the wrong preserved ELF rejects before connection;
see 2026-09-15-i2c-gdb-guard-test.log. This is not device-tested and cannot verify
the flashed image automatically. Next useful step is a controlled JTAG capture,
not another timing/priority patch. Actual delaying task remains unverified.

## First live JTAG semaphore failure capture

On unchanged d08530f, matching-ELF hardware breakpoint hit 0x4216d37c. Main was
returning a failed 100 ms semaphore take via read_device_regs(PMU reg0) ->
twatch_board_read_power -> app_main. At the halt, lvgl was in xQueueReceive ->
s_i2c_send_commands (line582), waiting for a transaction event while inside the
I2C transfer. diag_sample was in vTaskDelay, bhi_irq waiting for notification,
and CPU1 in idle. This identifies the LVGL task as the active I2C transfer path
at the timeout snapshot, not proof of who held the semaphore during the entire
100 ms interval. It does not support a simple all-CPU-busy explanation. Five
frames were insufficient to show the exact LVGL caller or current bus address;
the next script captures 16 frames and active/caller addresses plus bus status.
See 2026-09-15-i2c-lock-jtag.log. No firmware change or flash.

Debugger cleanup issue: after the original script disabled breakpoints and
detached, a 25 s serial capture was empty and the operator reported a crash.
A new OpenOCD session found both cores halted at the diagnostic breakpoint PC.
Its breakpoint list was empty (new server bookkeeping), so hardware state was
cleared explicitly with reg ibreakenable 0 on both cores and resume on CPU0.
Both targets then reported running. OpenOCD was shut down and serial confirmed
continued heap/adapter/sensor execution through uptime 234.771 s, with
internal_free=178960 and heap_integrity=ok. The debugger attach had caused the
documented reset (serial reset_reason=3); no new panic appears in the resumed
capture. This apparent crash was a debugger halt, not an established firmware
panic. The script now deletes breakpoints and explicitly requests monitor resume
before detach. Revised automatic cleanup is not yet device-tested; manual
cleanup and serial recovery were tested. Always verify execution after detach.

Next inspect the full LVGL I2C stack, active transfer address/status and event
wait before choosing a bus/driver change. No timeout, priority or PMU changes.
