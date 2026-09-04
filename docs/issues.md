# UWatch Firmware - Known Issues and Code Review

Updated: 2026-09-04

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
- **Factory-app partition headroom** — the 2026-09-04 image uses 2,554,208 bytes of the 3 MiB app partition, leaving 591,520 bytes (19%). The already-linked SX1262, ST25R3916, and NDEF archives total about 10 KiB, so no layout change is currently justified. Revisit below 10% free or before adding a large dependency; resizing would relocate the coredump/assets partitions and requires a coordinated full flash.
- **Static analysis** — CI installs cppcheck and runs `tools/run_cppcheck.sh` over project-owned firmware and simulator sources. Imported ubxlib/Bosch code and generated fonts are excluded; intentional `panictest` behavior and one unsupported Bosch preprocessor expression have narrowly scoped suppressions.

## Critical Issues (remaining)

No critical issues are currently tracked.

## Medium-Priority Fixes

No medium-priority fixes are currently tracked.

## Low-Priority / Cleanup

No low-priority fixes are currently tracked.
