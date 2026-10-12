# UWatch Firmware - Known Issues and Code Review

Updated: 2026-10-10

## Recently Fixed

- **Alarm ring-task buffer leak** — superseded by the audio-service refactor. The ring task no longer allocates audio memory; the audio task reuses one fixed lifetime buffer, and queued PCM copies are freed on completion or enqueue failure.
- **Tracking/GNSS shared-state races** — mutex protection now covers `s_fix`, `s_nav_status`, satellite lists, statistics snapshots, and session state; the tracking task performs sensor reads outside its state lock.
- **`uwatch_main.c` command dispatcher** — re-indented `debug_process_cmd()` for readability.
- **ESP-IDF version** — verified `espressif/idf:v6.0.2` is available locally; no Dockerfile change needed.
- **IMU wake-state race** — `s_imu_wake_armed` is now protected by an ISR-safe critical section shared by the GPIO ISR and the sleep-entry/exit paths. Battery-powered wrist-tilt wake was confirmed working on hardware (2026-09-04).
- **Crash-dump SD writes** — raw ELF and report saves now verify every write and the final flush before the flash copy is erased.
- **Audio recording-task input guard** — `main/debug/debug_audio.c` now rejects an absent or empty buffer before calling `t3902_read()`.
- **Battery runtime estimate** — the gauge now retains a real five-minute sample ring, so each percentage is paired with its original timestamp.
- **GNSS startup failures** — core power-on and streamed-position failures now roll the receiver back to off; optional skyplot/status telemetry failures remain non-fatal and are logged.
- **GNSS switch flicker** — the GPS switch now reflects the requested state while startup runs; normal on/off behavior was hardware-confirmed.
- **Dead `firmware/` directory** — removed (2026-08-27). It duplicated `main/` with an older, simpler implementation and wasn't referenced by the root `CMakeLists.txt`; its field-tested fixes (60 MHz QSPI glitch fix, shared-SPI2 CS handling) were confirmed superseded by the current `main/`/`components/` code before deletion.
- **Brace-style consistency** — project-owned control statements consistently use same-line braces. The Allman-style code is confined to imported Bosch BHI260AP and ubxlib sources, which should retain upstream formatting to avoid noisy vendor diffs.
- **Configuration-constant ownership** — alarm amplitude, tracking cadence, battery-estimator thresholds, and power policy remain named constants beside their owning modules; a global configuration header would add coupling without enabling a current use case. The duplicated power-management GPIO numbers now reference the canonical `twatch_board.h` pin definitions. Promote individual values to Kconfig only when a supported build-time variant requires it.
- **Hardcoded timezone** — local-time conversion now uses the build-time `CONFIG_UWATCH_TIMEZONE` POSIX string, defaulting to CET/CEST. The hardware RTC continues to store UTC.
- **Factory-app partition headroom** — the 2026-09-04 image uses 2,554,224 bytes of the 3 MiB app partition, leaving 591,504 bytes (19%). The already-linked SX1262, ST25R3916, and NDEF archives total about 10 KiB, so no layout change is currently justified. Revisit below 10% free or before adding a large dependency; resizing would relocate the coredump/assets partitions and requires a coordinated full flash.
- **Static analysis** — CI installs cppcheck and runs `tools/run_cppcheck.sh` over project-owned firmware and simulator sources. Imported ubxlib/Bosch code and generated fonts are excluded; intentional `panictest` behavior and one unsupported Bosch preprocessor expression have narrowly scoped suppressions.
- **NFC antenna tuning/range validation** — hardware sweeps and repeated tag reads selected `A=0x82`, `B=0x00`. Register A showed a broad reliable plateau, while B=0 improved a fixed-position read test from 37/70 to 109/109; moving edge-of-range behavior was also verified.
- **RTC ten minutes fast** — measured against an NTP-synced host: the RTC held 688 s of absolute error but kept correct rate (153 s per 153 s wall clock, 60.003 s per RTC second), so this was an unset clock rather than drift. GNSS had no fix that day, so no sync had run. Corrected with `settime` from the host clock; readback matched within about one second. Findings below cover why it could not self-correct.
- **Weekday convention mismatch (driver vs alarm service)** — `pcf85063a_time_t.weekday` is now the raw Register_03 value and matches `tm_wday` (0=Sunday…6=Saturday). Updated `pcf85063a_epoch_to_time()`, the header contract, the unit-test anchors, `sim/mock_hw.c` and the `rtc_fields_plausible()` guard; added a test that pins the field against libc `gmtime_r()` across a week. The device's stored weekday stays invalid (`WD=0x07`) until the time is written again.
- **OS flag blocking GNSS time sync** — `rtc_sync_from_utc()` no longer returns early when `pcf85063a_get_time()` reports the sticky oscillator-stop bit; it logs, skips the offset report, and writes the time unconditionally, which clears `OS`. (There is no `pcf85063a_clear_os_flag()` in the tree — the only clearer was always `pcf85063a_set_time()`.)
- **`rtccal` only trimming rate** — `m10q_rtc_calibrate()` now carries the current fix's UTC time into `s_pending_utc` (new `fix_utc_epoch()` helper, also used by `nav_pvt_cb()`), so a manual calibration corrects absolute time as well as ppm.
- **`s_rtc_offset` sentinel collision** — the "unset" marker is now `INT16_MIN` instead of −1, which is a legal −1 LSB offset.
- **`s_wake_sources` cleared outside its critical section** — `pm_arm_gpio_wakeup()` now clears it under `s_wake_sources_lock`, matching the GPIO ISR and `pm_wake_task()`.
- **Torn 64-bit PPS timestamp in `m10q.c`** — `pps_isr` brackets the timestamp with a sequence counter and `pps_snapshot()` retries until it reads a stable pair, so a PPS edge can no longer tear the 64-bit value into a bogus calibration sample. The window now counts edge deltas, so a missed edge no longer divides a two-second span by one interval.
- **`ble_debug.c` capture buffer race** — the vprintf hook appends under `s_capture_lock` and `ble_debug_send_capture()` swaps to a second buffer before notifying, so a log line from another task can no longer keep the buffer from draining.
- **Unbounded `haptic_test` task creation** — the waveform preview is single-flight behind a `s_test_running` flag; a tap while one runs is dropped instead of stacking another 2 kB task on the same HAPTIC_EN rail.
- **ST25R3916 concurrent SPI sessions** — `open()` now takes a driver mutex for the whole session and `close()` releases it, so `nfc_ctrl_task` and the `nfcpoll`/`nfctune` commands can no longer interleave bring-up, RF field state, or the antenna-tuning registers. A colliding caller gets `ESP_ERR_INVALID_STATE` immediately; `close()` after a failed `open()` no longer touches another task's session, and `nfcprobe` checks `st25r3916_busy()` before sweeping rails.

## Critical Issues (remaining)

Full detail, evidence and suggested fixes are in [the review log](todo.md#rtc-time-source-review-10-oct) and [the 10 Oct re-review](todo.md#code-review-findings-10-oct-full-re-review).

No open critical items.

## Medium-Priority Fixes

- **Unchecked FreeRTOS creation returns** in `ble_debug.c`, `syslog_capture.c` and `daily_log.c`.
- **`debug_cmd_dispte` busy-waits 200 ms** at priority 5 without yielding.
- **Inconsistent synchronization in `power_mgmt.c`** (`s_brightness`, `s_sparmodus_active`, `s_night_mode_auto`) and **no synchronization at all in `cd_timer.c`**.

## Low-Priority / Cleanup

- **Deferred NFC tuning automation** — replace manual setting-by-setting tests with a repeatable passive-tag fixture and automated sweep. Evaluate the available PN532 as an additional target; Flipper/VNA options and measurement limits are recorded in [the NFC follow-up plan](nfc.md#deferred-easier-antenna-tuning-and-measurements). Current validated tuning stays unchanged.
