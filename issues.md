# UWatch Firmware - Known Issues and Code Review

## Critical Issues

### 1. Missing NULL/Dereference Checks
- **`lvgl_app.c:620-626`** - `alarm_check()` calls `pcf85063a_alarm_triggered()` without verifying `twatch_rtc_dev` is initialized
- **`tracking.c:198`** - `tracking_get_estimated_position()` dereferences `s_prev_lat` without checking if a fix has ever been obtained (`s_session_has_pos`)
- **`sensor_cache.c:112-133`** - `cache_task` continues even if individual I2C reads fail; `c.valid` is set but some fields may contain garbage from uninitialized stack memory

### 2. Resource Leaks
- **`lvgl_app.c`** - Multiple LVGL objects created without proper cleanup paths. Power screen, BHI screen, and GPS screen objects persist indefinitely
- **`alarm.c:188-193`** - `ring_task` allocates `buf` with `heap_caps_malloc` but the buffer is never freed when the ring loop exits normally (only freed on allocation failure)
- **`uwatch_main.c:434-441`** - `rec_task`/`tonerec`: `s_rec_n` updated even if `heap_caps_realloc` fails, leaving `s_rec_buf` pointing to old size
- **`lvgl_app.c:581-593`** - `sweep` command has similar realloc issue as `tonerec`

### 3. Error Handling Inconsistencies
- **`uwatch_main.c:75-79`** - USB-JTAG install failure just logs warning and deletes task; driver may be left in inconsistent state
- **`power_mgmt.c:264-293`** - `power_mgmt_enter_sleep()` returns `ESP_ERR_NOT_SUPPORTED` on USB power but doesn't clean up partially-enabled rails
- **`sensor_cache.c:135-141`** - Semaphore take gives even on failure; mutex is taken and given regardless of success, potentially causing state desync

### 4. Potential Bugs
- **`sensor_cache.c:83-85`** - Gauge window sliding: `s_gauge.first_ms += 1000` assumes 1ms resolution but boundary conditions with `> GAUGE_WINDOW_MS` could miss edge cases
- **`tracking.c:210-212`** - Course calculation: `cos(s_prev_lat * π / 180)` when `cos` argument near ±π/2 could cause large `dlon` values; no guard against division by zero when `cos(s_prev_lat)` ≈ 0
- **`power_mgmt.c:241-248`** - IMU wake re-arming checks `s_imu_wake_armed` but variable state is set/cleared in multiple places (`button_isr`, `pm_arm_gpio_wakeup`, `pm_wake_task`), making state tracking complex and error-prone

### 5. Code Style/ maintainability
- Mixed brace styles throughout (Allman vs K&R)
- Magic numbers everywhere: `TRACK_STEPS_PER_FIX 50`, `GAUGE_WINDOW_MS 5min`, `RING_CHUNK_SAMPLES`, etc.
- `alarm.c:50-57`: `#define` constants with obscure naming (`RING_EDGE_MS 12`, `RING_BEEP_HZ 880`)
- `uwatch_main.c:debug_process_cmd` has inconsistent indentation in if-else chains (some `}` on same line as else, some on new line)

### 6. Missing Features/Edge Cases
- **`alarm.c:315`** - `alarm_set()` validates `hour > 23 || min > 59 || ring_mode > ALARM_RING_BOTH` but doesn't check `hour >= 0 || min >= 0` before assignment
- **`tracking.c:106-109`** - When activity check fails, `s_last_fix_steps` is reset but `s_fix_due` is also reset; however, if tracking was recently active and becomes inactive, the 50-step window counter doesn't reset properly
- **`sensor_cache.c:143-148`** - RTC sync every minute: `s_sync_system_count` increments per cache tick (1s), but the condition `>= (60u * 1000u) / CACHE_PERIOD_MS` evaluates to `>= 60`. If `CACHE_PERIOD_MS` changes, this could drift

### 7. Concurrency Concerns
- **`sensor_cache.c`** - Mutex protects `s_cache` and `s_est` separately but `gauge_feed()` modifies `s_est` outside the mutex (called after `xSemaphoreGive(s_mux)` at line 140)
- **`tracking.c`** - `s_fix_due` flag set in `tracking_task()` (called from FreeRTOS task) and read in `gps_ctrl_task()` (different task) without mutual exclusion beyond NVS persistence

## Medium-Priority Fixes

1. Add NULL checks before all pointer dereferences in `alarm_check()`, `tracking_get_estimated_position()`, and sensor read paths
2. Ensure all `heap_caps_malloc` have corresponding `heap_caps_free` on all code paths
3. Standardize error handling pattern across all files (early returns with logs)
4. Fix gauge window sliding boundary conditions
5. Add guards for `cos()` near ±π/2 in tracking course calculation
6. Add range validation (`>= 0`) in `alarm_set()`
7. Fix mutex usage in `sensor_cache` - move `gauge_feed()` call inside critical section or protect `s_est` separately
8. Document all magic numbers or move to configuration structure

## Low-Priority / Cleanup

1. Standardize brace style across codebase
2. Add documentation comments for all public APIs
3. Create unit tests for core modules (alarm, tracking, sensor_cache)
4. Add static analysis (cppcheck, clang-tidy) to CI pipeline
5. Review and document all `#define` constants with meaningful names