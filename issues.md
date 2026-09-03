# UWatch Firmware - Known Issues and Code Review

Updated: 2026-09-03

## Recently Fixed

- **Alarm ring-task buffer leak** — `main/services/alarm.c` now frees the ring buffer on task termination.
- **Tracking/GNSS shared-state races** — added mutex protection in `main/services/tracking.c` and `components/drivers/m10q/m10q.c` for `s_fix`, `s_nav_status`, and session state.
- **`uwatch_main.c` command dispatcher** — re-indented `debug_process_cmd()` for readability.
- **ESP-IDF version** — verified `espressif/idf:v6.0.2` is available locally; no Dockerfile change needed.
- **IMU wake-state race** — `s_imu_wake_armed` is now protected by an ISR-safe critical section shared by the GPIO ISR and the sleep-entry/exit paths. Hardware gesture-wake regression remains to be run.
- **Crash-dump SD writes** — raw ELF and report saves now verify every write and the final flush before the flash copy is erased.
- **Audio recording-task input guard** — `main/debug/debug_audio.c` now rejects an absent or empty buffer before calling `t3902_read()`.
- **Battery runtime estimate** — the gauge now retains a real five-minute sample ring, so each percentage is paired with its original timestamp.
- **Dead `firmware/` directory** — removed (2026-08-27). It duplicated `main/` with an older, simpler implementation and wasn't referenced by the root `CMakeLists.txt`; its field-tested fixes (60 MHz QSPI glitch fix, shared-SPI2 CS handling) were confirmed superseded by the current `main/`/`components/` code before deletion.

## Critical Issues (remaining)

### 1. Resource / concurrency issues
- **`components/drivers/m10q/m10q.c`** — `m10q_power()` ignores failures from `uGnssPwrOn()`, `U_GNSS_CFG_SET_VAL_RAM()`, and `uGnssPosGetStreamedStart()` after logging; consider retry/fail-fast paths.

## Medium-Priority Fixes

1. Surface GNSS power-on/open failures instead of continuing blindly.

## Low-Priority / Cleanup

1. Standardize brace style across the codebase (mixed K&R / Allman).
2. Centralize magic numbers (`RING_*`, `TRACK_*`, `PM_*`, `GAUGE_*`) into a config header or Kconfig.
3. Document or make configurable the hardcoded `TZ` in `app_main()`.
4. Consider increasing the factory app partition beyond 2 MB if LoRa/NFC drivers are filled in.
5. Add static analysis (`cppcheck`/`clang-tidy`) to the existing CI build job.
