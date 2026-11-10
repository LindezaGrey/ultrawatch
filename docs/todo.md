# UWatch resolved engineering review log

The 2026-10-10 and 2026-10-12 sections below have open `[pending]` items. All
earlier sections are resolved history; their previously open items were also
tracked in `docs/issues.md`.

## Overview
This document captures the differences between the `docs/m10Q.md` best-practices guide and the actual code changes implemented in this session. The two are largely **orthogonal** — they address different aspects of M10Q usage.

---

## docs/m10Q.md
*A comprehensive "Best Practices Configuration Guide" for u-blox M10Q, created during this session (14. Aug 20:51).*

### Covered Topics:
- Configuration persistence (RAM + BBR, layer 3)
- GNSS constellation selection (GPS+Galileo vs all, power saving vs accuracy)
- Dynamic platform model (Airborne<2g=7, Automotive=4, Pedestrian=3, Stationary=2)
- Signal integrity (Super-S, ITFM interference detection)
- Power save mode (PSM/CT, cyclic tracking)
- UART & output rates (115200+/baud, NMEA filtering)
- Quick checklist for drone/High-Performance
- Example `UBX-CFG-VALSET` sequence (pseudo-code)

### Scope: Ongoing configuration settings applied by the user/application.

---

## Code Implementation (commit 0efeda5)
*Focused on RTC-aided GNSS initialization and module version detection.*

### Added Features:
| Feature | Lines | Description |
|---------|-------|-------------|
| `rtc_fields_plausible()` | m10q.c | Validates RTC year 2000-2100, month 1-12, day 1-31, etc. Prevents sending garbage (year-2070) time to GNSS. |
| `mga_ini_seed()` enhancement | m10q.c | Skips MGA-INI time injection if module already has valid VBACKUP-backed time. Reduces unnecessary aiding data sends. |
| `m10q_get_versions()` API | m10q.h + m10q.c | New function to read UBX-MON-VER (sw, hw, mod, fw, prot strings). Added `gnssver` console command. |
| Module MOD field detection | uwatch_main.c + m10q.c | Differentiates genuine MIA-M10Q from clones via the MOD field in UBX-MON-VER. |
| RTC plausibility guard before GNSS aiding | uwatch_main.c | Ensures only plausible local RTC time is injected into the GNSS module. |

### Scope: Initialization-time checks and API additions. Runs once at startup, not ongoing configuration.

---

## Comparison Summary

| Aspect | docs/m10Q.md | Code Implementation |
|--------|-------------|---------------------|
| **RTC validation** | ❌ Not mentioned | ✅ `rtc_fields_plausible()` guards against year-2070 |
| **Module identification** | ❌ Not mentioned | ✅ `m10q_get_versions()` + MON-VER + MOD field |
| **Persistence (BBR/RAM)** | ✅ Full coverage (layer 3 details) | ❌ Already working, not changed |
| **Dynmodel/constellations/PSM** | ✅ Full coverage | ❌ Not touched (per-session config) |
| **Scope** | Ongoing configuration settings | Initialization-time checks + new APIs |
| **When it runs** | User/application sets these | Runs once at startup (mga_ini_seed, version check) |

---

## Recommendation
- **Keep docs/m10Q.md as-is**: It remains a valid general M10Q configuration reference.
- **No conflict**: The new code operates at initialization time; the docs cover ongoing user configuration.
- **Optional**: Add a brief note in the docs or a new section acknowledging the RTC-aided GNSS initialization features, but not required since they're orthogonal.

---

## Session Commit History (related)
```
0efeda5 feat: add gesture consume in main loop + m10q improvements + rtccal command
d6cecea m10q: inject MGA-INI time+position aiding, read UBX-NAV-STATUS
ec7cfb3 docs: describe watch face HH:MM:SS + UTC layout; fix font comment
92842fc m10q: PPS-disciplined RTC calibration + GNSS time sync
```

## Historical validation

The changes in this log were built and hardware-tested during their respective
sessions. Current build status belongs in the active development handoff, not
in this historical checklist.

---

# Code Review Findings (16. Aug)

Status labels: `[pending]` not started, `[in-progress]`, `[done]`.

## High
1. `[done]` **RTC/PWRKEY/BOOT GPIO ISR re-arm at sleep entry** (2026-09-03) — `pm_arm_gpio_wakeup()` now restores the shared ISR sources after configuring their low-level wake conditions, preventing a post-wake button or RTC event from being silently missed. Battery-powered wrist-tilt wake was confirmed working on hardware (2026-09-04).
2. `[done]` **power_mgmt.c — `s_wake_gpio` last-writer-wins with simultaneous wake sources.** (2026-08-27)
   Gesture + alarm waking together attribute to one source; the other's selective ISR re-arm is skipped. Impact limited by the 1 Hz `alarm_check` polling while awake, but compounds with #1.
   **Fix:** replaced `s_wake_gpio` with a `s_wake_sources` bitmask (`PM_WAKE_PWRKEY`/`BOOT`/`IMU`/`RTC`), OR'd in from `button_isr` under a critical section; `pm_wake_task` now processes every set bit in one pass instead of only the last writer. See ADR [0001](adr/0001-wake-source-bitmask.md).

## Medium
3. `[done]` **tracking.c — tracking task held `s_mux` across I2C reads + `esp_lv_adapter_report_activity()`.** (2026-09-03)
   The task now samples activity and step count outside the state lock, then rechecks the session before applying results.
4. `[done]` **m10q.c — data mutex taken inside ubxlib rx callbacks (`pos_cb`, `nav_status_cb`).** (2026-09-04)
   Verified safe: ubxlib runs the NAV message receiver at priority 19 and the streamed-position callback at priority 2, while LVGL runs at priority 6. `s_data_mux` is created with `xSemaphoreCreateMutex()`, so ESP-IDF FreeRTOS priority inheritance bounds any inversion when a higher-priority callback waits for a reader. Lock holders perform only bounded field updates or structure copies, with no I/O, delays, or nested locks; no code change is needed.
5. `[done]` **`imon` GPIO8 monitor busy-poll** (2026-09-03) — superseded: the debug command samples at 10 ms intervals with `vTaskDelay()`, yielding between reads.
6. `[done]` **Display TE/VSync flush sync (co5300).** (2026-09-04) — not applicable on the current hardware/configuration. With ALDO2 and board DISP_PWR both enabled and USB preventing auto-sleep, the `dispte` diagnostic observed GPIO6 flat low (`0` edges in 200 ms, no high level). A TE-gated wait would therefore stall the 40-row flush bands. Keep the existing partial-band QSPI DMA flush without delay-based pacing.

## Low
- `[done]` **Configuration and formatting cleanup review** (2026-09-04) — project-owned constants remain beside their owning modules; duplicated power-management GPIO values now reference the board pin map. Brace-style differences are confined to imported vendor sources and retain upstream formatting.
- `[done]` **Alarm ring buffer ownership** (2026-09-04) — superseded by the audio-service refactor. `ring_task` no longer allocates a buffer; `audio_task` owns one fixed alarm-cycle buffer for its lifetime, so repeated rings do not allocate or leak memory. Queued PCM copies are released through `audio_reply()` or immediately if enqueueing fails.

## Process / Validation
- `[done]` **Battery-powered wrist-tilt wake** (2026-09-04) — confirmed working on hardware. USB operation cannot exercise this path because VBUS intentionally prevents automatic sleep.
- `[done]` **BHI260AP gyroscope orientation matrix** (2026-08-27) — `bhi260ap_init()` sets `BHY2_PHYS_SENSOR_ID_GYROSCOPE`'s orientation matrix to `diag(-1,-1,-1)` (`bhi260ap.c`), replacing the app-side quaternion-conjugate hack previously in `lvgl_app.c`'s `bhi_cube_update()`. Fixes the rotation-sense mismatch at the source (before on-chip fusion) instead of only patching the cube widget, so gesture/activity detection benefit too. **Confirmed working on hardware**: cube tracks physical rotation correctly on all axes.

---

# Code Review Findings (17. Aug full re-review)

Full read-only review of main/, components/ (Bosch vendor lib at integration points only). Verified closed: `ring_task` buffer free (alarm.c:279-284), coredump erase-on-CRC (crash_dump.c:184-194, confirmed live on watch), m10q UART driver delete + ttfs persistence, tracking mutex.

## Medium
1. `[done]` **GNSS rail (BLDO1) never turns off when GPS is disabled** (2026-08-27) — `enter_sleep()` had since grown a `lvgl_gps_enabled()` check, but `exit_sleep()` still restored BLDO1 unconditionally on every wake, desyncing m10q's `s_powered` from the physical rail (`m10q_power(false)` then permanently no-ops) and leaving the rail stuck on (~25-30 mA) after the first wake with GPS off.
   **Fix:** made `exit_sleep()`'s BLDO1 restore conditional on `lvgl_gps_enabled()`, symmetric with `enter_sleep()`.
2. `[done]` **Battery runtime estimate after five minutes** (2026-09-03) — resolved with a five-minute sample ring that keeps every percentage paired with its original timestamp.
3. `[done]` **BLE vprintf hook recursion** (2026-09-03) — removed the self-logging call, preserved the chained hook’s `va_list`, and bounded captured output. The BLE debug bridge remains deliberately disabled while its DMA reservation conflicts with the display buffer; the guarded scan screen is separate.

## Low
4. `[done]` **DST inconsistency** (2026-09-03) — superseded by UTC RTC storage and `pcf85063a_time_to_epoch()` in `twatch_board.c`; system-clock sync no longer uses local-time conversion or `tm_isdst`.
5. `[done]` **`parse_nav_sat()` wrote `s_fix` without `s_data_mux`** (2026-09-03) — satellite-list updates now use the same mutex as other receiver callbacks.
6. `[done]` **`m10q_get_stats()` read `s_stats` unlocked** (2026-09-03) — readers and callback updates now take a coherent mutex-protected snapshot.
7. `[done]` **Night-mode red filter off-by-one** (2026-09-03) — superseded by the adapter’s end-exclusive draw-bitmap API. `area_rounder_cb()` explicitly converts LVGL’s inclusive invalidation bounds before this callback computes its pixel count.
8. `[done]` **Alarm `ring_start()` reset race** (2026-09-03) — superseded by the current ring state machine: `s_ringing` remains true until a pending dismiss/snooze is consumed, so RTC handling cannot start a second ring in that window.
9. `[done]` **Watch-face month index unguarded** (2026-09-03) — superseded by `localtime_r()` formatting in `screens/watch_face.c`; no direct RTC month-array indexing remains.
10. `[done]` **System-log flush dropped its buffered batch on SD failure** (2026-09-03) — `syslog_capture.c` now restores a failed batch ahead of concurrently captured lines when mounting, opening, writing, or closing the log file fails.
11. `[done]` **Alarm auto-snooze cap and escalation** (2026-09-03) — unanswered alarms retry at most six times (10-minute intervals), with generated speaker amplitude rising from roughly 10% to 50%; the final ring continues until dismissed.

## Trivial
- `[done]` **`crashread` filename validation** (2026-09-03) — the debug-only reader now accepts only a single safe crash-report filename, preventing directory traversal outside its crash-report directory.
- `[done]` USB debug console oversized line recovery (2026-09-03) — a full unterminated input buffer is discarded with a diagnostic so later commands continue to be read.
- `[done]` **GNSS control request hand-off** (2026-09-03) — the intentionally single-slot, latest-request-wins GNSS command is now exchanged atomically between the UI and the slow control task.
- `[done]` **Duplicated screenshot documentation** (2026-09-04) — removed the stale block that described base64 output and LVGL-task-only use; the remaining comment matches the raw RGB565 stream and internal locking implementation.
- `[done]` Alarm ring task/resource allocation checks (2026-09-03) — `alarm_init()` returns `ESP_ERR_NO_MEM` with a diagnostic if its semaphore or task cannot be created.
- `[done]` **CO5300 teardown** (2026-09-03) — the dormant `co5300_deinit()` path now releases its panel-I/O handle before freeing the SPI bus.
- `[done]` **Boot display rail-cycle margin** (2026-09-03) — boot and diagnostic recovery now both hold DISP_PWR low for 200 ms.
- `[done]` **`disppwr` reinitialization** (2026-09-03) — it routes through `display_controller_recover()`, which rail-cycles and reruns the complete CO5300 initialization sequence.

---

# Code Review Findings (10. Oct full re-review)

Read-only review of `main/` (37 `.c` files, ~13.6k lines) and the project-owned
`components/drivers/` plus `components/spi2_power`. `components/ubxlib` and
`managed_components/` were reviewed at integration points only. Every item
below was re-read against the source before being written down; the "Verified
clean" list at the end records what was checked and deliberately not raised.

Status labels: `[pending]` not started, `[in-progress]`, `[done]`.

## High

1. `[done]` **ST25R3916 has no internal locking — concurrent SPI sessions on one device.**
   `components/drivers/st25r3916/st25r3916.c` carries no mutex. Two independent
   callers drive it: `nfc_ctrl_task` (`main/lvgl_app.c:479-545`) and the
   `dbg`-task commands `nfcpoll` / `nfctune` / `nfcprobe`
   (`main/debug/debug_cmds.c:428-620`). Either can be inside
   `st25r3916_open()` → register I/O → `close()` while the other starts.
   `spi2_power` refcounts the rail but does not serialize a command sequence.
   Impact: interleaved transactions corrupt field state, can leave the field
   enabled, or let one task address a rail the other has just cut.
   **Fix:** one mutex in the driver covering open/session/close, or a single
   owner task that both callers post requests to.
   Done with the mutex: `s_session_lock` is taken non-blocking by
   `st25r3916_open()` and dropped by `st25r3916_close()` (which is now a no-op
   unless the calling task owns the session, so the existing "close after a
   failed open" call sites stay harmless). `try()` / `tuning_measure()` /
   `field_reset()` / `read_type2()` return `ESP_ERR_INVALID_STATE` when called
   outside the caller's own session, and the new `st25r3916_busy()` lets
   `nfcprobe` refuse to sweep rails under a live session. A colliding caller
   fails fast rather than queueing behind an 8 s scan.

2. `[done]` **`power_mgmt.c` — `s_wake_sources` cleared outside its critical section.**
   `pm_arm_gpio_wakeup()` (`main/system/power_mgmt.c:988`) did a plain
   `s_wake_sources = 0;`, while `button_isr` (`:729-731`) and `pm_wake_task()`
   (`:904-907`) both use `portENTER_CRITICAL`. An edge landing in that window
   loses its wake-source attribution. The comment at `:68-69` shows the
   discipline is understood elsewhere; this line just missed it.
   **Fixed (10. Oct):** the clear now runs under `s_wake_sources_lock`.

3. `[done]` **`m10q.c` — torn 64-bit read of the PPS timestamp.**
   `pps_isr` writes `volatile int64_t s_pps_time_us` (`components/drivers/m10q/m10q.c:125`);
   `cal_task` read it at `:248`. Xtensa performs a 64-bit volatile access as
   two 32-bit halves, so an edge mid-read tears the value. One bad sample in
   the 120 s window produces a wrong OFFSET written to the RTC and persisted
   to NVS.
   **Fixed (10. Oct):** `pps_isr` brackets the timestamp with a `s_pps_seq`
   seqlock and `pps_snapshot()` retries until it sees a stable, even sequence,
   returning the edge count with the timestamp so both describe the same edge.
   The calibration window now counts edge deltas rather than loop passes, so a
   missed edge no longer divides a two-second span by one interval.

4. `[done]` **`ble_debug.c` — command capture buffer is a data race.**
   `s_capture` / `s_capture_len` (`main/connectivity/ble_debug.c:314-316`) are
   appended by the vprintf hook, which runs in whichever task happens to log,
   while `ble_debug_exec_command()` and `ble_debug_send_capture()` (`:333-353`)
   reset and drain them on `ble_telem`. `s_capturing` / `s_resp_pending` are
   `volatile`, which does nothing for the buffer or its length. `s_capture_len`
   can be re-incremented after the drainer zeroes it, so the buffer never
   drains. `s_conn_handle` and `s_adv_active` are likewise shared unlocked
   between the NimBLE host task and `ble_telem`.
   **Fixed (10. Oct):** two capture buffers swapped under
   `s_capture_lock`, appends serialized by the same lock. `send_capture()`
   takes the swap (never holds the lock across `ble_gatts_notify_custom()`)
   and drains the buffer it took, so a late log line can no longer stop the
   drain. `s_conn_handle` / `s_adv_active` are still unlocked — NimBLE's own
   host-task writes versus `ble_telem` reads of a scalar handle, left as is.

5. `[done]` **`haptic.c` — unbounded task creation.**
   `haptic_play_test_async()` (`main/controllers/haptic.c:85`) created a fresh
   2048-word `haptic_test` task per call, with no dedup and no handle reuse.
   Repeated taps on a test screen piled up tasks and heap.
   **Fixed (10. Oct):** a `s_test_running` flag under `s_test_mux` makes the
   preview single-flight; a second request while one runs is dropped with a
   debug log. `mesh_log.c:327` still calls the synchronous
   `haptic_play_test()` directly, so a mesh ring can overlap a preview —
   rare, and left alone.

## Medium

6. `[done]` **Unchecked FreeRTOS creation returns.** Mostly the codebase is
   careful; these are the gaps:
   - `main/connectivity/ble_debug.c:484` `xQueueCreate` and `:489`
     `xTaskCreate(telemetry_task, "ble_telem", 3072, …)` — return values ignored
   - `main/system/syslog_capture.c:153-154` — `xSemaphoreCreateMutex` /
     `xSemaphoreCreateBinary` unchecked
   - `main/services/daily_log.c:39` — `daily_log_init()` ignores a NULL mutex
     (the tick path at `:185` does guard it)

   Under heap pressure these fail silently and the feature is simply dead.
   **Fixed (12. Oct):** each site logs and degrades instead of failing quietly.
   `ble_debug_init()` only installs its vprintf mirror once both the queue and
   `ble_telem` exist (the queue is drained solely by that task, so installing
   the hook first would leak one `strdup` per log line); `syslog_capture_init()`
   keeps both-or-neither — on failure it deletes whatever it got and leaves the
   hook uninstalled, and `syslog_capture_service()` then just holds the
   caller's cadence instead of taking a NULL semaphore. `daily_log_init()` logs
   the allocation failure.

7. `[done]` **`debug_cmds.c` — busy-wait without yield.** `debug_cmd_dispte`
   spins for 200 ms polling `CO5300_PIN_TE` (`main/debug/debug_cmds.c:882-890`)
   on the priority-5 `dbg` task, starving every task at priority ≤ 5 on that
   core including IDLE.
   **Fix:** `vTaskDelay(1)` inside the sampling loop, or an esp_timer.
   **Fixed (12. Oct):** `vTaskDelay(1)` per sample. At `CONFIG_FREERTOS_HZ=1000`
   that is 1 ms against a ~16 ms TE period, so the edge count and the derived
   Hz stay meaningful while IDLE gets the core back.

8. `[done]` **`power_mgmt.c` — inconsistent synchronization policy.**
   `s_display_timeout_s` and `s_return_timeout_s` are guarded by `s_idle_mux`
   (`:326-371`), but `s_brightness` (`:176`, read `:385`, written `:314`/`:393`,
   used `:688`), `s_sparmodus_active` (`:177`) and `s_night_mode_auto` (`:168`)
   are touched from the UI task, `pm_wake_task` and housekeeping with nothing.
   Byte accesses do not tear on Xtensa, so the impact is stale reads — but the
   read-modify-write at `:390` and the sleep-entry read at `:688` can act on a
   value that just changed. Pick one policy for this file.
   **Fixed (12. Oct):** one policy, stated at the declarations: these scalars
   are `volatile`, written inside `s_idle_mux` (so a setter's compare-and-store
   cannot lose to another writer), and read without it — a lock-free reader
   gets the old or the new value, never a mix. Paired reads
   (`s_display_activity_ms` with the timeout in `display_idle_expired()`) keep
   the mux on the read side. NVS writes and the display/UI calls stay outside
   the critical section, and `set_brightness()` now uses its local `level`
   instead of re-reading the global.

9. `[done]` **`cd_timer.c` — no synchronization at all.** `s_active` /
   `s_remaining_s` (`main/services/cd_timer.c:10-11`) are written by
   `cdtimer_start`/`cdtimer_cancel` (UI screen *and* the `dbg` console task),
   decremented by `cdtimer_check()` (watch-face UI tick), and read by the UI
   getters. A `start` racing a `check` can be clobbered, or a `cancel` undone.
   **Fixed (12. Oct):** a `s_mux` spinlock around all four state accesses;
   `cdtimer_check()` decides expiry under the lock and calls
   `alarm_ring_now()` outside it.

## Low

- `[done]` **`sx1262.c` — `s_rx_active` (`:76`) and `s_recv_task` (`:88`)
  have no protection.** Currently benign: only `mesh_log`'s task calls the RX
  API, and `s_recv_task` is already documented as a deliberate single-pointer
  atomic (`:79-88`). Worth a comment on `s_rx_active` recording that it is
  single-task by design, so nobody adds a second caller.
  **Fixed (12. Oct):** `s_rx_active` now carries that same single-owner note.
- `[done]` **`gpx_log.c` — `s_path` (`:41`)** is written from the UI/dbg task
  and read by the track task; a path swap mid-write can produce a torn filename.
  **Fixed (12. Oct):** `s_mux` now guards every access to the start/stop state,
  and the track task copies the path into its own local buffer under that lock
  before opening the file, so no SD I/O happens inside the critical section.
  `gpx_log_stop()` reports the path through a caller-owned buffer
  (`gpx_log_current_path()`) instead of returning a pointer into `s_path`.
  `s_claimed` rejects a second `gpx_log_start()` while a track task still owns
  the old path, which is what made the stale-path window reachable at all.
- `[done]` **`ble_scan.c` / `wifi_scan.c`** control tasks poll on a 200 ms
  `vTaskDelay` loop even when the radio is off; a notification with timeout
  avoids the wakeups.
  **Fixed (12. Oct):** both idle waits are now binary-semaphore takes with the
  previous timeout kept as a safety net, and `set_enabled()` gives the semaphore
  so an enable is acted on immediately instead of up to 200/500 ms later.
  `ble_scan.c` also gives it from the NimBLE `BLE_GAP_EVENT_DISC_COMPLETE`
  handler, which removes the second 200 ms poll loop that used to run for the
  whole duration of every scan.

## Verified clean (checked, not raised)

- **ISRs** — only four exist in project code (`button_isr`,
  `pm_imu_wake_is_armed_from_isr`, `pps_isr`, `sx1262_dio1_isr`). None log,
  allocate, take mutexes, or call blocking APIs. `sx1262_dio1_isr` uses
  `vTaskNotifyGiveFromISR` + `portYIELD_FROM_ISR` correctly.
  `esp_timer_get_time()` inside `pps_isr` **is** IRAM-safe on ESP32-S3:
  `esp_timer_impl_get_time` is `ESP_TIMER_IRAM_ATTR` with `esp_timer_get_time`
  as an alias (`~/esp/esp-idf/components/esp_timer/src/esp_timer_impl_systimer.c:68-75`).
- **Critical sections** — every `portENTER_CRITICAL` region found is short and
  contains no I/O or blocking calls. `display_controller.c` does panel I/O
  outside the lock and uses a refcounted `display_sync_t`.
- **Lock ordering** — no AB-BA inversion in `alarm.c` (three mutexes) or
  `bhi260ap.c` (mux ordering is consistent).
- **`alarm.c` accessors** — `alarm_is_armed`, `alarm_is_ringing`,
  `alarm_is_snoozing`, `alarm_get_ringing_index`, `alarm_get_sound_enabled` all
  take `s_lock`.
- **`audio.c`** — the `audio_completion_t` refcount via `__atomic_sub_fetch` is
  correct on both the send-failure and caller-timeout paths.
- **`spi2_power.c`** — rail refcounting is correctly locked; the settle delay is
  deliberately taken outside the lock.
- **`esp_lv_adapter_lock(-1)`** in `power_mgmt_exit_sleep()` is the documented
  `portMAX_DELAY` idiom, not a violation.
- **`vTaskSuspend(NULL)`** at `main/connectivity/mesh_log.c:203` is intentional
  and documented (the rail is about to be cut) — not a leak.
- **Task stacks and priorities** are consistent: UI/drivers at 5, services 2-4,
  tracking 2.

# RTC time-source review (10. Oct)

Prompted by the watch showing a time about ten minutes off. Reviewed
`components/drivers/pcf85063a/`, the RTC paths in `components/drivers/m10q/`,
`main/services/alarm.c` and the `settime`/`rtcdump`/`rtccal` debug commands, and
measured the running device over `/dev/ttyACM0`.

## Measurement result

The RTC was **688 s (11 min 28 s) ahead** of this host's NTP-synced clock
(host 09:16:35 → RTC 09:28:04, host 09:19:08 → RTC 09:30:37). It was not
drifting: it advanced exactly 153 s over 153 s of wall clock, and a five-sample
run measured 60.003 s per RTC second. So the rate/offset correction is working
and the time was simply never set — GNSS was off (`state=0`, 0 satellites in
view, `fixes=259 today=0`), so no sync had run today.

Corrected with `settime 2026-10-10 11:11:02` from the host clock; readback
matched the host within about one second.

The offset register is `OFF=0x7f` — MODE 0 (normal correction, once per 2 h,
LSB 4.34 ppm), value −1 LSB ≈ −4.3 ppm ≈ 0.4 s/day. Note the trim range is only
+63/−64 LSB, i.e. **±273 ppm ≈ ±23.6 s/day**, so this part cannot absorb a large
error by rate alone.

## High

1. `[fixed 10. Oct]` **Weekday convention mismatch makes weekday-matched alarms
   impossible.** `pcf85063a_epoch_to_time()` writes `weekday = tm_wday + 1`
   (`components/drivers/pcf85063a/pcf85063a_time.c`), i.e. 1=Sunday…7=Saturday,
   and `test/test_pcf85063a_time.c:33,49` assert that convention. The PCF85063A
   Register_03 weekday field is 0–6, and `main/services/alarm.c:224` correctly
   uses `tm_wday` (0=Sunday). The two sides never agree.
   Live evidence: after `settime` on a Saturday, `rtcdump` showed `WD=0x07` —
   outside the chip's legal range — while `ALM=0000098005` arms 09:00 with the
   weekday field masked. Any alarm that enabled weekday matching can never fire,
   and `settime` itself writes the invalid value.
   **Fix (done):** the struct field is now the raw Register_03 value, i.e.
   `tm_wday`, 0=Sunday…6=Saturday. Updated `pcf85063a_epoch_to_time()`, the
   header contract, both unit-test anchors, `sim/mock_hw.c`, and the
   `rtc_fields_plausible()` guard in `m10q.c` (was `1..7`, now `<= 6`). Added
   `test_weekday_matches_tm_wday()`, which pins the field against libc
   `gmtime_r()` across a week — that is the invariant `alarm.c` depends on.
   The device still holds `WD=0x07` until the time is written again.

2. `[fixed 10. Oct]` **OS flag makes the RTC permanently uncorrectable by GNSS.**
   `pcf85063a_get_time()` returns `ESP_ERR_INVALID_STATE` when the sticky
   oscillator-stop bit is set, and `rtc_sync_from_utc()` bailed on its first
   call. There is no `pcf85063a_clear_os_flag()` in the tree (an earlier reading
   of that was wrong); the only code that clears `OS` is
   `pcf85063a_set_time()` — which the bail-out made unreachable from the GPS
   path. So one brownout disabled GPS time sync for good; only a manual
   `settime` could recover it. Latent: `OS=0` now.
   **Fix (done):** `rtc_sync_from_utc()` no longer returns early. An unreadable
   RTC is logged, reported as an unknown offset, and written unconditionally,
   which clears `OS` as part of the time write.

## Medium

3. `[fixed 10. Oct]` **`rtccal` cannot correct an absolute time error.**
   `m10q_rtc_calibrate()` notified `cal_task` with `s_pending_utc == 0`, so the
   `rtc_sync_from_utc()` branch was skipped and only `cal_run()` (rate) executed
   (`components/drivers/m10q/m10q.c:141`). The command name promises a full
   calibration; it only trimmed ppm.
   **Fix (done):** `m10q_rtc_calibrate()` now sets `s_pending_utc` from the
   current fix before notifying, via the new `fix_utc_epoch()` helper that
   `nav_pvt_cb()` also uses.

## Low

- `[fixed 10. Oct]` **`s_rtc_offset` sentinel collision.** `int16_t s_rtc_offset`
  used −1 for "unset", but −1 is also a valid offset of −1 LSB (−4.34 ppm), and
  `offset_load()` accepted the stored −1 and re-applied it. Now uses
  `INT16_MIN`, which the existing range check already rejects.
- `[pending]` **`CAP_SEL` is never configured.** `CTRL1=0x00` leaves the POR
  7 pF load capacitance. If the crystal needs a different value the resulting
  ppm error is systematic and eats into the ±273 ppm trim budget.
  **Doc'd (12. Oct):** the board's 12 pF external caps (C31/C32 on Y4) are now
  recorded in `docs/hardware.md` with the measurement procedure needed to pick
  0 or 1. No driver change until that comparison exists.
- `[done]` **`pcf85063a_set_time()` writes the time registers without setting
  `CTRL1_STOP` first**, which the datasheet recommends against; a time write
  while running can produce an intermediate state.
  **Fixed (12. Oct):** follows §7.2.1.2 — set `STOP`, write the time registers,
  release `STOP`, then clear `OS`/`OSI`. Releasing `STOP` is not conditional on
  the caller's previous `CTRL1`, so a failed time write cannot leave the
  oscillator stopped. The first second after release is advanced by the
  datasheet's 0.5078 s (Table 6), which is well inside the 1 s sync threshold.

## Verified clean (checked, not raised)

- Register map (`REG_SEC=0x04`, `REG_ALM_SEC=0x0B`, `REG_TIMER_VALUE=0x10`,
  `REG_TIMER_MODE=0x11`), `CTRL1_STOP = 1 << 5`, and the 24-hour-mode assumption
  (Register_01 resets to 0x00) all match the datasheet.
- The offset sign convention in `cal_run()` is correct.
- `OFF=0x7f` is **not** a disabled correction: Register_02 bit 7 is `MODE`
  (0 = normal, once per 2 h, LSB 4.34 ppm; 1 = course, every 4 min,
  LSB 4.069 ppm), not an enable bit. An earlier reading of it as "offset
  correction inert" was wrong.
- `position_updated()` gates on `pvtValid` plus `fixType` 2/3, so
  `mga_ini_seed()` does not create a circular RTC-aiding path.
- An apparent erratic rate (+30 s in one window, −18 s in another) was a
  measurement artifact: the first sample of a run can be stale, and the
  0.5078 s first-second step after a `STOP` release lands inside a 1 s sync
  threshold. Continuous single-session sampling disproved it. (An earlier
  version of this bullet blamed "`esp_timer` pauses during light sleep" —
  wrong, see the corrections in the 12 Oct section below.)

# Battery drain and low-battery handling (12. Oct)

The watch looked dead on the morning of 12 Oct. Everything below comes from
`battery.csv`, `steps.csv` and the SD syslog mirror read over the console
(`synclog`/`dailycat`/`crashls`/`rtcdump`), not from a bench repro.

## What actually happened

| wall clock | uptime | evidence |
| --- | --- | --- |
| 2026-10-11 23:02 | 221,502 s | last `battery.csv` row: 0 %, 3496 mV, `stop` |
| 2026-10-11 23:02 | reset | new boot banner, `W crash_dump: core dump image check failed: ESP_ERR_INVALID_SIZE` |
| +57 min | 0 → 3,421,219 ms | "zombie" boot: only the 60 s `twatch_board: system clock synced from RTC (UTC)` line plus one wake gesture; zero battery/steps rows; `E esp_lvgl:adapter: Auto sleep enter callback failed (ESP_ERR_NOT_SUPPORTED)` at 2,487 ms |
| 2026-10-12 09:22 | 4 s | plugged in: 45 %, `cc` |

The cell ran to 0 and the ESP32-S3 brownout-rebooted, then came back into a
state that could neither sleep nor log, and died again. `crashls` shows no
dumps after 31 Aug and the dump image check fails, so this is hard power loss,
not a firmware crash. The same failure already happened once: the 12→14 Sep
cycle also runs to 0 % (07:15) with the session ending at 07:18.

## Measured baseline (use this to spot a regression)

`pct` points lost per 60-minute window, both endpoints `charge_state=stop`
(discharge only), all boot sessions in `battery.csv` split on `uptime_s` reset:

| date | n | p10 | median | p90 | max |
| --- | --- | --- | --- | --- | --- |
| 2026-09-02 | 489 | 5 | 6 | 9 | 10 |
| 2026-09-04 | 505 | 6 | 8 | 11 | 12 |
| 2026-09-05 | 1046 | 0 | 1 | 3 | 6 |
| 2026-09-06 | 1272 | 4 | 4 | 7 | 19 |
| 2026-09-08 | 840 | 4 | 5 | 8 | 9 |
| 2026-09-09 | 721 | 0 | 1 | 2 | 3 |
| 2026-09-11 | 3883 | 1 | 4 | 6 | 7 |
| 2026-09-12 | 2349 | 0 | 3 | 13 | 17 |
| 2026-09-13 | 3843 | 1 | 4 | 7 | 8 |
| 2026-09-14 | 786 | 2 | 3 | 14 | 15 |
| 2026-10-09 | 1714 | 2 | 3 | 5 | 6 |
| 2026-10-10 | 3300 | 1 | 3 | 4 | 6 |
| 2026-10-11 | 2827 | 0 | 2 | 6 | 16 |
| **all** | 23575 | 1 | **4** | 7 | 19 |

Median 4 %/h ≈ 44 mA on the 1100 mAh cell, i.e. about **25 h of run time per
charge**, so one charge lasting just over a day is the expected figure, not a
regression. The measured fatal cycle agrees: unplugged at 100 % on 10 Oct 17:07,
dead at 23:02 on 11 Oct = **29.9 h**, ≈3.3 %/h. **10-11 is the best day in the
log, not a regression.** The failure is at the bottom of the gauge:

| cycle | 30 % | 20 % | 10 % | 5 % | 1 % | 30→1 | 10→1 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 12 Sep 15:13 → 14 Sep 07:18 | 04:55 | 05:55 | 06:38 | 06:59 | 07:15 | 140 min | 37 min |
| 10 Oct 17:07 → 11 Oct 23:01 | 20:35 | 21:40 | 22:23 | 22:43 | 22:58 | 143 min | 35 min |

The last 30 % lasts ~2.4 h and the last 10 % only ~35 min, on both cycles. So
the single battery threshold in the firmware buys 35 minutes — and as finding 2
shows, it does not actually buy anything.

The same fatal cycle, split into 10-point bands, shows how little `pct` tracks
charge:

| band | 100→90 | 90→80 | 80→70 | 70→60 | 60→50 | 50→40 | 40→30 | 30→20 | 20→10 | 10→0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| %/h | 3.6 | 4.4 | 3.7 | 2.8 | 2.9 | **1.0** | 4.4 | 9.3 | 13.8 | 15.7 |
| crossed | 19:54 | 22:09 | 00:52 | 04:22 | 07:50 | 18:19 | 20:35 | 21:40 | 22:23 | 23:01 |

The 50→40 band (07:50–18:19, a working day on the wrist) runs at 1 %/h; the
bands below it run at 4–16 %/h. The evening is the *quieter* of the two — the
longest sleep episode in the whole log, 19:21→22:02, sits inside the fast bands
— so the acceleration is the gauge and the cell voltage, not a load spike.
`pct` is a fuel-gauge estimate, not a coulomb count, and it collapses at the
bottom of the discharge curve (finding 8).

## High

1. `[done]` **No low-battery warning and no low-battery shutdown anywhere in
   `main/`.** The only battery threshold in the whole project is
   `PM_SPARMODUS_BATT_PCT 10` (`main/system/power_mgmt.c:464`, used at 624 and
   902). The only UI signal is the status-bar battery icon turning red at
   `batt_pct <= 15` (`main/screens/status_bar.c:204`) — there is no warning
   event, no warning screen, no log line at 5 %/3 %, and no shutdown path
   triggered by state of charge — a grep for `battery_low`/`batt_low` in `main/`
   returns nothing. The device therefore relies entirely on the AXP2101's
   hardware UVLO, which is what produced the brownout reboot, the truncated
   append-only CSVs, and the 57-minute zombie boot.
   `axp2101_soft_poweroff()` already exists
   (`components/drivers/axp2101/axp2101.c:418`) and `power_mgmt_shutdown()`
   (`main/system/power_mgmt.c:798`) already does the right sequence — display
   off, `daily_log_flush()`, `sd_log_unmount()`, soft power off — but it is only
   reachable from the PWRKEY long-press. Reusing it on a SOC threshold is a
   small change.

   **Fixed (13. Oct):** `pm_battery_guard()` in `main/system/power_mgmt.c` now
   owns the decision. It skips when VBUS is present or the battery is absent
   (`axp2101_get_battery_pct()` reports 0 % for an absent cell, so checking the
   gauge alone would shut the watch down with no battery fitted), warns once at
   `PM_BATT_WARN_PCT` 15 %, and calls `power_mgmt_shutdown()` after **two
   consecutive** readings at or below `PM_BATT_SHUTDOWN_PCT` 3 % — the gauge
   reads 0 % while VBAT is still 3496 mV, so a single sample is not trusted.
   It runs from both the AXP IRQ path and the 30 s sparmodus tick, so it is not
   gated on a wake-to-idle transition (finding 2). The warning line also states
   the lead time using `PM_BATT_MIN_PER_PCT` 4 min/%, the measured 10 %→1 %
   figure: at 15 % that is ~48 min. No new UI screen — the red icon at 15 %
   already covers the visible case, and a shutdown at 3 % leaves no time to read
   a screen.
2. `[done]` **The one low-battery action in the code sets a flag nothing
   consumes while the device is asleep.** The `<10 %` check runs on the periodic
   tick of `pm_wake_task()` (`main/system/power_mgmt.c:899-919`) and the
   auto-entry branch (916) only sets `s_sparmodus_active`. The only place that
   flag becomes action is `power_mgmt_enter_sleep()` at an idle timeout, gated
   on `!vbus` (`main/system/power_mgmt.c:1117`) - so it needs a *fresh*
   wake-to-idle transition. Set while the device is already asleep, it does
   nothing until the next such cycle. The log shows exactly that:
   `Ultra-Sparmodus: battery below 10%, auto-activating` at uptime 219,422,410 ms
   (22:26), 5.6 min into a sleep episode that began at 219,083,235 ms, and there
   is no `entering sleep` line after it - the device never completed another
   wake-to-idle cycle before the brownout. The cell went 10 % at 22:26 to 0 % at
   23:01 (35 min) with the flag set the whole time. The one
   `power_mgmt_sparmodus_enter_sleep()` call that sleeps without waiting for an
   idle transition is on the "USB removed, still low" path (909), which never
   ran because that session had no VBUS at all.

   **Fixed (13. Oct):** the shutdown decision no longer travels through
   `s_sparmodus_active` at all. `pm_battery_guard()` is called directly from the
   AXP IRQ handler and from the 30 s sparmodus check in `pm_wake_task()`, so it
   runs whether or not a wake-to-idle transition ever happens. `PM_SPARMODUS_BATT_PCT`
   is unchanged — it only selects a sleep *mode* — but it is no longer the last
   line of defence. The one path that still skips the guard is
   `power_mgmt_sparmodus_enter_sleep()`, which does not return; the guard
   resumes on the next 30 s tick after wake, and the `socwl1` IRQ is itself a
   wake source.
3. `[done]` **`battery.csv` — the only power telemetry — is gated behind the
   IMU daily path.** `daily_log_tick()` (`main/services/daily_log.c:191`,
   called from `main/services/housekeeping.c:50`) holds `s_log_mutex` and then
   `goto done` unless `local_date()`, `sd_log_session_begin()`,
   `bhi260ap_daily_update()` and `bhi260ap_daily_snapshot()` all succeed and
   `day.ymd == ymd` (daily_log.c:216). The `battery.csv` row is written inside
   that same block (`append_lines()`, daily_log.c:219). Consequence: any IMU or
   RTC problem silently stops the power history, and the frozen step counter
   (finding 7) suppresses the steps/activity rows for the same reason. Power
   telemetry must not depend on the IMU snapshot.

   **Fixed (13. Oct):** `daily_log_tick()` is split into `append_battery_line()`
   and `append_activity_lines()`. The battery row now only needs the clock and an
   SD session; the BHI260AP snapshot gate guards the steps/activity rows alone,
   and when it fails it logs one `W daily_log: daily snapshot unavailable,
   steps/activity rows skipped` instead of silently dropping everything. Header
   creation moved out of the gate too, so a new day's files are still valid when
   the IMU is dead. Verified live after the flash: `steps.csv` now writes
   `2026-10-13,16:32,700,82525` against its 4-field header and `activity.csv`
   matches its 3-field one. What this fix does **not** explain is the 12 Oct
   zombie boot: that boot wrote two battery rows (uptime 4 s and 14 s) and then
   nothing for the remaining 56 minutes, so either `housekeeping_task()` stopped
   reaching `daily_log_tick()` or the tick's clock/SD-session preconditions
   failed - neither of which ungating changes.

## Medium

4. `[done]` **The AXP2101's own low-battery warnings are decoded and
   ignored.** `AXP_IRQ_SOCWL1`/`SOCWL2` (`main/system/power_mgmt.c:105-107`)
   only ever reach `axp_irq_describe()` for the log string; the sole action on
   an AXP IRQ is `AXP_IRQ_PEK_LONG` (941). The decisive line:
   `AXP IRQ wake: irq=0x000090 (lowsoc socwl2)` at uptime 217,937,338 ms —
   22:03, 59 minutes before the cell hit 0 — and the firmware did nothing.
   Across the whole 8 MB syslog these warnings appear 5 times (`socwl2` ×4,
   `socwl1` ×1) out of ~250 AXP IRQ lines.

   The chip already provides the whole mechanism, and the firmware uses none of
   it (all register references from `Datasheets/AXP2101_datasheet.pdf`):
   - `REG1AH` sets the two thresholds: bits 7:4 warning level 2 (5–20 %, 1 % per
     step, POR `1010b` = **15 %**), bits 3:0 warning level 1 (0–15 %, POR
     `0001b` = 1 %). The 22:03 `socwl2` landing at ~15 % is that POR default
     firing exactly as configured — the PMIC warned on time, in the log, and was
     ignored.
   - `REG40` bits 7/6 enable `socwl2_irq`/`socwl1_irq` (both enabled out of
     reset); `REG48` bits 7/6 are the RW1C status flags the log decodes. `REG48`
     bit 7 is "SOC drop to **Warning** Level IRQ" and bit 6 is "SOC drop to
     **Shutdown** Level IRQ", so level 2 is the warning and level 1 the shutdown
     trip. The lone `irq=0x000050 (lowsoc socwl1)` line is the PMIC reaching its
     shutdown level — 1 %, the POR default — and the firmware treated it as
     another log line. That session cannot be dated from the syslog (see the
     corrections below), so do not claim which cycle it belongs to; the flag
     itself is unambiguous.
   - `REG17` resets the fuel gauge, `REGA4H` is the SOC the driver already reads.

   `AXP_IRQ_LOWSOC` (bit 4) is the *gauge new-SOC* flag, which is what
   `axp2101_disable_gauge_soc_irq()` masks at boot; the warning bits 6/7 are
   separate, were never masked, and were never acted on.

   **Fixed (13. Oct):** `axp2101_set_low_soc_levels()` /
   `axp2101_get_low_soc_levels()` write `REG1A` with level 2 = 15 % and level
   1 = 3 %, armed by `power_mgmt_init()` after `axp2101_disable_gauge_soc_irq()`
   followed by `axp2101_clear_irq()`. Level 1 is lifted off the 1 % POR default
   because the gauge reports 0 % while VBAT is still 3496 mV; the firmware wants
   the decision made at 3 % and the guard's two-sample rule covers the gauge's
   optimism. `debug_cmd_rails()` prints the armed levels next to the live gauge
   SOC, so the write is verifiable over the console instead of inferred from the
   driver source.
5. `[pending]` **There is no current measurement path on this PMIC, so a drain
   cannot be attributed to a rail.** `battery.csv` has `chg_ma`, which is the
   *configured* charge current, not a measurement. Unlike the AXP192/AXP202, the
   AXP2101 has **no IBAT channel at all**: its 14-bit SAR ADC covers only VBAT,
   VBUS, VSYS, TS and die temperature (datasheet Table 6-5), and `REG36H/37H` —
   the addresses that carry IBAT on the older parts — are `ts_h`/`ts_l` here.
   `components/drivers/axp2101/axp2101.c:179-182` enables BATT-V, VBUS, SYS-V
   and TEMP, which is the full useful set. So do not go looking for an
   `axp2101_get_battery_ma()`; it cannot be written against this chip.
   Attributing the drain needs either the e-gauge's own bookkeeping (SOC +
   VBAT over time, which is what this section's tables do) or a bench
   measurement in series with the cell.
6. `[done]` **No syslog rotation, and the RAM mirror is tiny.**
   `main/system/syslog_capture.c:130` opens `SYSLOG_FILE` with `"a"` and never
   checks size, renames or truncates, so the file is now 8,020,377 bytes and
   still growing; a full-file `synclog <substring>` scan costs ~200 s at
   ~40 KB/s, and the newest data is the most expensive to read. The RAM mirror
   is `SYSLOG_BUF_SIZE 4096` (line 18) with a 3 KB high-water mark, so a boot
   where the SD session fails keeps ~1 s of history — which is exactly the
   zombie-boot case.

   **Fixed (13. Oct):** the buffer is now a ping-pong pair of 8 KB halves
   (`SYSLOG_BUF_SIZE 8192`), so the RAM window is 16 KB instead of 4 KB and a
   flush costs no copy — the flusher keeps the batch it is writing and hands
   capture the other half, which is why the old `static char flush_copy[]` is
   gone. Capture is never blocked on SD I/O, and the buffers stay in internal
   DRAM because the `vprintf` hook can run while the cache is disabled during a
   flash operation. The SD mirror rotates at `SYSLOG_MAX_BYTES` 2 MB into
   `syslog.1.txt`, two generations total, and drops the old generation with a
   one-line warning below `SYSLOG_MIN_FREE_BYTES` 8 MB free so the log mirror is
   never what fills the card. `synclog` now also reports `syslog.1.txt`'s size,
   otherwise the older half of a post-mortem looks like it was never recorded.
   Rotation is not yet exercised on hardware — check it with `synclog` after the
   next flash.
7. `[done]` **BHI260AP soft recovery never sticks, and the step counter
   froze.** `W lvgl_app: BHI260AP data stale but chip responsive; soft
   recovery` appears 880 times and `bhi260ap: sensors re-enabled (soft
   recovery)` 879 times, in bursts at 24 h (65), 30 h (45), 49 h (15), 53 h
   (44) and 57 h (103) of uptime. The trigger is
   `bhi260ap_get_data_age_ms() > BHI_STALE_MS` at `main/lvgl_app.c:814`, and
   the recovery only re-applies the sensor enables — it never escalates to a
   full re-init. `lifetime_steps` froze at **81,825 at 2026-10-10 20:05** and
   never advanced again through the end of the log on 12 Oct; `day_steps` was 0
   on 12 Oct. Both late boots report `persisted steps 81825`.

   **Fixed (13. Oct):** `bhi260_task()` now counts consecutive stale-but-
   responsive periods and escalates after `BHI_SOFT_RECOVERY_LIMIT` (3, about
   15 s of dead stream) to `bhi260ap_deinit()` plus a full re-init, which
   re-uploads the RAM firmware image. Counting attempts rather than acting on
   age is the only way to tell the two cases apart, because
   `bhi260ap_reenable_sensors()` calls `bhi260ap_mark_data()` and so resets the
   staleness clock on every "recovery" — that is exactly why 879 re-enables
   changed nothing. The escalation waits `BHI_REINIT_BACKOFF_MAX_MS` (30 s)
   before the next pass so a genuinely wedged chip does not re-upload its
   firmware every 15 s. This makes the freeze recoverable, not explained: why
   the virtual-sensor configuration is lost in the first place is still open.
8. `[pending]` **The percentage on the screen cannot answer "how much time is
   left", and the gauge has never been told which cell it is driving.** The band
   table above spans 1 %/h to 16 %/h inside a single discharge, so "42 %" carries
   no time information and any threshold scheme has to sit where the gauge falls
   fastest rather than be spaced evenly. Two reasons this is a configuration gap
   rather than a bug:

   - The AXP2101's E-Gauge (§6.11) is a voltage/profile estimator, not a coulomb
     counter. It exports SOC (`REGA4H`) and VBAT (`REG34H`/`REG35H`) and nothing
     else — no capacity-remaining or time-remaining register to read instead, and
     no current channel to integrate (finding 5).
   - Its accuracy comes from a battery parameter table: `REGA1H` selects a ROM
     profile, `REGA2H` bit 4 picks ROM vs. a user SRAM table and bit 0 is a
     "brom writer". §6.11 states programming the battery module is "highly
     recommended to achieve better Fuel Gauge accuracy". **The driver never
     writes `REG1AH`, `REGA1H`, `REGA2H` or `REG17H`**, so the gauge runs on
     whatever ROM profile it booted with, against an 1100 mAh cell whose real
     curve nobody has measured. The datasheet contradicts itself here: §6.13.2.86
     marks `REGA1H` read-only, the register map marks it writable.

   The datasheet does not document SOC non-linearity and gives no SOC-vs-voltage
   or SOC-vs-time curve, so the knee measured above is measurement, not spec;
   what the datasheet supplies is the mechanism that makes it expected.

   Two spec facts that bear on the shutdown design:
   - `VBAT_UVLOZ` is 2.4 V (Table 5-5), while DCDC1 — which feeds the ESP32-S3 —
     is specified 2.6–5.5 V in, 1.5–3.4 V out, with no documented dropout. The
     PMIC's own battery cutoff is therefore below the point where the 3.3 V rail
     sags, which is consistent with the brownout reboot on 11 Oct, and leaves the
     firmware's shutdown level as the only protection that actually works. The
     rail collapse point stays an inference, not a datasheet number.
   - Reaching the shutdown level only sets `REG48` bit 6; nothing in the
     datasheet says the PMIC cuts power on SOC by itself. The level is a
     notification and the action is firmware's.

   Options, in rough order of value: program the battery parameters for the
   fitted cell (needs its datasheet, or a measured discharge curve — `battery.csv`
   already logs VBAT next to SOC, so the data to fit one exists); or stop
   presenting `pct` as if it were linear and show time remaining from the
   measured band table instead of the raw number.

## Low

9. `[done]` **`steps.csv` and `activity.csv` headers declare more columns
    than the rows write.** Headers are `date,time,day_steps,lifetime_steps`
    (`main/services/daily_log.c:98`) and `date,time,activity` (line 104), but the
    rows use `"%s %s,%lu,%lu\n"` (129) and `"%s %s,%s\n"` (138) — date and time
    are merged into one field, so the files are 3- and 2-column. `battery.csv` is
    the only one that matches its header. The comment near daily_log.c:145
    already acknowledges this and deliberately leaves it; either fix the header
    or the rows.

    **Fixed (13. Oct):** the rows were split into `date,time,...` so they match
    the declared headers — changing the headers instead would have silently
    reinterpreted every existing row. This is a format break for those two files:
    rows written before **2026-10-13** have one merged date+time field, later rows
    have two, so any analysis must normalise by field count, as it already does
    for `battery.csv`'s pre-2026-09-02 rows. The date is in the code comment next
    to the `fprintf` calls, because this is the kind of thing that only matters
    six weeks later.
10. `[done]` **Wrong power-management comment.** `main/system/power_mgmt.c:1298`
   said "DFS + automatic light sleep (tickless)". Tickless idle is not the
   mechanism here: `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y` is a no-op on the SMP
   kernel (IDF-4986; `portSUPPRESS_TICKS_AND_SLEEP` is empty in
   `FreeRTOS-Kernel-SMP/include/freertos/FreeRTOS.h:2694`). The light sleep
   comes from IDF's own `esp_pm` auto-sleep path driven by the
   `esp_pm_configure()` call on the next line.

   **Fixed (12. Oct):** comment replaced in place; no behaviour change, and
   `UWatch.bin` is byte-identical in size after the rebuild.
11. `[done]` **Idle boots look like hangs in the log.** Once the watch is
    asleep, the only line emitted is `twatch_board: system clock synced from
    RTC (UTC)` every 60 s — 18,801 of them across the syslog, the single
    largest source after `power_mgmt` and `mesh_log`. A 2.7 h sleep episode is
    indistinguishable from a dead device unless you diff the timestamps.

    **Fixed (13. Oct):** `sync_system_time()` keeps the first sync at INFO and
    logs later ones at DEBUG, promoting to WARN only when the system clock
    actually moved — `|delta| > 2 s` measured before `settimeofday()`. That is
    the case worth keeping: it is the only in-field sign of the RTC drifting or
    being reset while running. Steady state is the RTC and the system clock
    agreeing, which is not news 18,801 times — which is 17 % of the syslog's
    lines and 15 % of its bytes. Removing them is also what makes the new 2 MB
    rotation worth having: the 8 MB file covered 42 days, so one generation is
    roughly 10 days of history instead of an unbounded file that costs 200 s to
    scan.

    Note this also removes the heartbeat the 12 Oct post-mortem was built from:
    the zombie boot was identified by 57 minutes of *only* that line. The
    equivalent signal now is `battery.csv` rows, which finding 3's fix puts back
    on that path, plus the WARN line if the clock is being corrected.

## Corrections to earlier reasoning (do not re-raise)

- `esp_timer_get_time()`, and therefore every `ESP_LOG` uptime and
  `battery.csv`'s `uptime_s`, **does** advance through light sleep on this
  build — rows stay exactly 604-605 s apart per 10 minutes across the sleeping
  evening. Uptime is a usable wall-clock proxy.
- The watch **does** sleep. From uptime 208,284 s (19:21) to 217,937 s (22:02)
  the log contains 157 copies of the 60 s RTC-sync line and nothing except the
  sleep-entry bookkeeping in the first 25 s: one ~2.7 h sleep episode. The "light sleep is never entered" hypothesis is wrong, and
  `causes=0x1` (`ESP_SLEEP_WAKEUP_UNDEFINED`) in the wake log is the normal
  cause for `esp_pm` micro-sleeps between task gaps, not evidence against
  sleeping. `slept=<ms>` is only wall time between `power_mgmt` "entering
  sleep"/"waking" lines, so a `sleep_ratio` computed from it is meaningless.
- `charge_state=stop` during the whole 11 Oct drain is **normal**: `stop` is
  `STATUS2` bits 2:0 == 5, "not charging" (`AXP2101_CHG_STOP`). `chg_enabled=1
  chg_ma=500` are configured settings, not a stuck charger, and the final 61 h
  session contains no `vbus_insert` IRQ at all — the watch was unplugged the
  entire time.
- The absence of `bhi260ap: step counter:` lines is a log-level artifact
  (`ESP_LOGD` at `components/drivers/bhi260ap/bhi260ap.c:313`), not a stalled
  counter. The counter freeze in finding 7 was established from `steps.csv`
  values, not from missing lines.
- The 638 `st25r3916: wait_irq(want=0x00000010) timed out` /
  `no response: txe=yes, timer/nfc irq=0x80` pairs (637) are NFC bench sessions
  (`nfcprobe`), not runtime traffic — none of them fall in the 11 Oct window.
- The 8 MB syslog is not a reliable wall-clock index. Only 71 lines carry a date,
  and both `battery.csv`'s `uptime_s` and the log's uptimes repeat across boot
  sessions, so a log line's date can only be interpolated between anchors. The
  one AXP warning that matters *is* anchored — `irq=0x000090` at uptime
  217,937,338 ms sits 9 s from the `battery.csv` row at 22:02 on 11 Oct, 15 % —
  but the other four warnings cannot be dated, so do not read a per-cycle story
  into them.
- Other high-count lines are noise, not findings: `draw-complete request
  dropped: controller queue full` ×1048, `gpio_install_isr_service(537): GPIO
  isr service already installed` ×339, `scan_start: ESP_ERR_WIFI_NOT_INIT` ×68
  paired with `pcf85063a_get_time(90): read time` ×68, and `PPS cal aborted:
  only 0 edges (need 10)` ×60 (no GPS fix in view, expected indoors). Counts are
  for the whole 8 MB syslog at the time of the dump.
- "`battery.csv` stopped recording five hours before the cell went empty" and
  "the 57-minute post-brownout boot produced zero rows" are both wrong, and so
  is the claim in the finding 3 fix note that ungating would have produced 57
  rows there. Re-dumping the card on 13 Oct: 11 Oct has one row a minute right
  up to 23:02, where the last pre-brownout row (uptime 221,502 s, 0 %, 3496 mV)
  sits 6 s after the one before it, and the zombie boot wrote two rows (uptime
  4 s and 14 s) before going silent. The only gap on those dates is 620 minutes,
  10-11 23:02 to 10-12 09:22, and that is the dead watch, not a logging fault.
  Finding 3's fix is still the right change - the gate really does couple power
  telemetry to the IMU - but it was never the explanation for 12 Oct.

## Suggested next steps (not implemented)

Findings 1-4, 6-7 and 9-11 are fixed on `ui-redesign`; findings 5 and 8 are the
ones that need a bench or another week of logging rather than a code change.
What is left is ordered by how much each one would change the next low-battery
outcome.

1. Find out what the 3 % shutdown level is actually worth. It is a guess derived
   from one cycle's 10 %→1 % in 35 minutes. `debug_cmd_rails()` now prints the
   armed levels next to the live gauge SOC, so the next discharge can show how
   much runtime is left after the guard's first sample. Raise the level if the
   watch still browns out before the guard fires; lower it if it fires with an
   hour to spare.
2. Program the gauge for the fitted cell, or stop treating `pct` as linear
   (finding 8). `battery.csv` already records VBAT beside SOC, which is enough to
   fit a curve and turn the displayed number into a usable time estimate.
3. Root-cause why BHI260AP soft recovery never sticks — finding 7's fix makes
   the freeze recoverable, not explained.
4. Settle `CAP_SEL` by measurement rather than datasheet: `rtcdump` plus host
   NTP/PPS drift for `CAP_SEL=0` vs `1` (see the load-capacitance subsection in
   `docs/hardware.md`).
5. If discharge current is ever needed, measure it on the bench — there is no
   IBAT register on this PMIC (finding 5).
6. Find out why `battery.csv` alone loses rows. The 13 Oct dump has 55 gaps
   longer than 2 minutes that are not boot boundaries — 393 minutes of power
   telemetry missing, usually 6-10 minutes at a stretch — and in every one
   checked `steps.csv` and `activity.csv` hold the rows for those same minutes.
   All three files were opened inside a single `append_lines()` call, so the
   battery `fopen()`/`fclose()` failed on its own, and that call checks neither
   return, so it left no trace: for the 11 Oct 20:01-20:06 gap (uptime
   210,609-211,033 s) the syslog holds nothing but the 60 s RTC-sync line.
   Finding 3's fix does not touch this; checking the open and logging the
   failure would at least make the next one visible.
7. Work out why the 12 Oct zombie boot wrote two `daily_log_tick()` rows and then
   none for 56 minutes while the 60 s RTC-sync task kept running. Either
   `housekeeping_task()` stopped reaching the tick — it takes `s_log_mutex` with
   `portMAX_DELAY`, and `sd_log_session_begin()` blocks the same way — or
   `local_date()`/the SD session failed and the tick exited quietly.
   `syslog_capture.c` drops every `sd_log:` line, so an SD failure in that boot
   is invisible by design.

# Boot forensics and Ultra-Sparmodus logging (14. Oct)

24 h wear/drain run on the Batch 1-5 build, 13 Oct 15:32 to 14 Oct 10:51, read-only
console checks only (`rails`/`rtcdump`/`crashls`/`dailycat`/`synclog`/`sparmodus`).
It answered the drain question and raised a worse one: the watch rebooted three
times in the night and the firmware has no way to say why.

## What the run showed

SOC fell 100 % → 91 % across the wear window, then rose to 99 % on the charger.
No brownout, and the 15 % warning and 3 % guard from Batch 1 never came close to
firing, so those two levels are still uncalibrated on a normal-wear run.

Five boot sessions, not one:

| wall clock | uptime | rows | SOC | note |
| --- | --- | --- | --- | --- |
| 13 Oct 15:32 → 20:26 | 0 → 17,658 s | 370 | 100 → 97 % | worn, mostly long light sleeps |
| 13 Oct 20:27 | 4, 19 s | 2 | 97 % | then 171 min of nothing |
| 13 Oct 23:18 | 4, 19 s | 2 | 95 % | then 288 min of nothing |
| 14 Oct 04:06 | 4, 19 s | 2 | 93 % | then 245 min of nothing |
| 14 Oct 08:11 → 10:51 | 0 → 9,614 s | 404 | 91 → 99 % | on the charger |

Inside all three gaps **no CSV file has a row for any minute** — not just
`battery.csv`, so this is not finding 3's per-file failure. `esp_timer_get_time()`
advances through light sleep on this build, so a reset `uptime_s` means a real
boot: the device was powered off or in deep sleep, not merely idle. `temp_c10`
supports that: 35.3 °C at 20:26, 31.6 °C at 23:18, 25.7 °C at 04:06 — the watch
cooled to ambient, so it was off a wrist and not self-heating.

## Findings

1. [done] **High: any boot session shorter than 60 s leaves no log at all.**
   `syslog.txt` holds only two of the five sessions above; the three 2-row sessions
   produced zero lines. `syslog_capture.c` buffers in the RAM ping-pong pair and
   writes only at the 6 KB high-water mark or when
   `now - last_flush >= pdMS_TO_TICKS(60000)` in `syslog_capture_service()`.
   `last_flush` starts at 0, so the first flush of a boot cannot happen before
   tick 60,000 — 60 s of uptime. A boot that dies at 20 s has its whole log in
   RAM when the power goes. The 13 Oct 20:26 session proves the loss is real:
   its last flushed line is `co5300: panel re-initialized (display off)` at
   uptime 17,599,996 ms, i.e. the mirror stops mid-wake, and everything after
   that — including whatever ended the session — was never written.
   **Fixed (14. Oct):** `syslog_capture_flush()` is public now, and called at the
   end of `app_main()`, so every boot writes its whole bring-up sequence to
   `syslog.txt` immediately instead of waiting for the 60 s tick. A session that
   dies in a brownout still loses its tail, but it always leaves its boot banner
   and reset reason behind, which is what makes a short session identifiable.
   The 60 s periodic cadence is deliberately unchanged: at the measured ~9 KB/h
   log rate a shorter interval would mean thousands more SD open/write/close
   cycles a day for little extra evidence.
2. [done] **Medium: the Ultra-Sparmodus switch is never logged.**
   `power_mgmt_set_sparmodus_active()` (`main/system/power_mgmt.c`) returns
   early when the value is unchanged and otherwise sets the flag and calls
   `pm_config_save()` without logging, so the Settings toggle
   (`main/screens/settings_screen.c:822`) and `sparmodus set on|off` both change
   persisted behaviour in total silence. The value loaded from NVS key
   `PM_NVS_KEY_SPARMODUS` in `pm_config_load()` is not logged at boot either.
   Only the entry paths log (`"Ultra-Sparmodus: entering deep sleep"`, the
   PWRKEY-IRQ and BOOT-held lines, the mesh-radio stop), and those lines are
   exactly the ones finding 1 throws away. Reading the state requires asking for
   it: `sparmodus` with no argument prints `persisted flag=%d`, which is how this
   was checked - it reads 0 now, and since the flag is NVS-persisted and nothing
   else writes it except the low-battery auto-activate in `pm_wake_task()`
   (SOC < 10 %, never reached), it was off all night.
   **Fixed (14. Oct):** the setter logs `Ultra-Sparmodus enabled`/`disabled` on
   every real change, `pm_config_load()` logs `Ultra-Sparmodus disabled at boot
   (NVS)` once per boot, and `app_main()` logs `Ultra-Sparmodus: explicit wake,
   staying awake` for a deep-sleep wake that is not a timer resleep;
   `power_mgmt_sparmodus_handle_wake()` only logged the alarm-wake case before.
   Together with finding 3's reset line this makes the mode's state and every
   wake from it visible in `syslog.txt`.
3. [done] **Medium: no reset reason or wake cause is logged at boot.**
   Nothing in `main/` calls `esp_reset_reason()`; `esp_sleep_get_wakeup_causes()`
   is only used inside `power_mgmt.c` to decide whether to resleep. So a boot
   leaves no record of whether it came from power-on, a brownout, a watchdog,
   `esp_restart()` or a deep-sleep wake. This is the single missing datum for the
   three night reboots, and it is cheap: one line after `syslog_capture_init()`
   in `app_main()`.
   **Fixed (14. Oct):** `power_mgmt_log_boot_reason()` logs
   `boot: reset=power-on` (or `brownout`, `deep sleep`, `software restart`,
   `task WDT`, …) plus `wake_causes=0x…` when the wake-cause bitmap is non-zero,
   and `app_main()` calls it immediately after `syslog_capture_init()`.
4. [done] **Medium: the paths that end a session flush the CSV outbox but not
   the log mirror.** `power_mgmt_shutdown()` (the >4 s PWRKEY path, also
   `pwroff`) calls `daily_log_flush()` and `sd_log_unmount()` but nothing flushes
   the syslog buffer, so the `"shutting down"` line it logs is itself lost.
   `power_mgmt_sparmodus_enter_sleep()` reaches `esp_deep_sleep_start()` the same
   way. `sd_log_unmount()` flushing "the RAM ring" refers to the sd_log ring, not
   the syslog mirror.
   **Fixed (14. Oct):** both call `syslog_capture_flush()` before
   `daily_log_flush()`/`sd_log_unmount()`, so a clean power-off and a Sparmodus
   deep-sleep entry now write their own last lines. The silent-resleep path needs
   no flush because it logs nothing.
5. [pending] **Medium: the three night reboots are unexplained.** What is ruled
   out: Ultra-Sparmodus (flag 0, and its 60 s tick would have produced a row every
   minute, not a 3 h gap), a firmware crash (`crashls` still shows no dump after
   31 Aug), the Batch 1 low-SOC guard (97 %), and a brownout (97 %, 4125 mV).
   What is left is the power path: a PWRKEY press long enough to hit the 4 s
   OFFLEVEL, which runs `power_mgmt_shutdown()` and cuts power, or the AXP's own
   PEK handling. A watch in a pocket or case pressing its own side button fits
   the pattern — boot, ~20 s, off, hours later boot again — and findings 1-4
   mean the firmware would record none of it. Distinguishing them needs finding
   3's reset-reason line plus finding 1's flush.
   Findings 1-4 are now fixed, so the next 24 h run answers this: a short session
   that starts with `boot: reset=power-on` and ends with `shutting down` was a
   PWRKEY power-off, `reset=brownout` is the power path, and `reset=deep sleep`
   with `wake_causes=0x8` is a Sparmodus timer wake.

## Next steps

- [pending] Re-run 24 h of normal wear on this build and read the new boot lines
  for every session, including the short ones; that is what closes finding 5 and
  also gives the first normal-wear pass at the Batch 1 15 %/3 % levels.
- [pending] Check the `fopen()`/`fclose()` results in `append_battery_line()` and
  its steps/activity equivalents and log a failure — the 55 in-session
  `battery.csv`-only gaps of the 13 Oct dump are still unexplained, and the
  unchecked open is the only candidate the code offers.
- [pending] Root-cause the 12 Oct zombie boot, which wrote two rows and then went
  silent for the rest of its session while `steps.csv`/`activity.csv` kept going.

## Corrections to earlier reasoning (do not re-raise)

- "The overnight 2-row sessions were Ultra-Sparmodus deep sleep" is withdrawn.
  It was wrong twice: the persisted flag reads 0, and `PM_SPARMODUS_TICK_US` is
  60 s, so an armed Sparmodus wakes every minute and a silent-resleep cycle would
  show up as a row every 1-2 min, not three boots in a night. The `uptime_s`
  reset plus all-three-CSVs-silent reasoning still identifies whole-device-off
  windows; only the mode attribution was wrong.
- "The syslog shows nothing happened at 20:26" is withdrawn and is the basis of
  finding 1. The absence of lines is the buffer never being flushed, not the
  absence of events. Any `syslog.txt` from before the 14 Oct flush fix carries
  gaps that say nothing about that time window; from that build on, a session
  always leaves at least its boot banner.
- `I (…) sd_log: SD mounted: 59 MB` appearing mid-session (uptime 12,242,970 ms
  in the pre-flash session, again at 16,849,631 ms on 14 Oct) is the normal
  re-mount after light sleep cut ALDO1, not a boot marker and not an SD fault.
  It is also the only line that survived from some sessions, which is how the
  session count in the old log reading came out wrong.

## Step and activity counters (14. Oct review)

Checked against the 14 Oct dump rather than only the code. The day model is
correct where it matters: `s_daily_total`/`s_daily_id`/`s_activity_ms` are
`#define` aliases into `s_days.current` (`bhi260ap.c:69`, `101-103`), so there is
no second stale copy; steps are a monotonic BHI lifetime counter read as
`base + delta` with the sub-ms remainder kept and a `now_us < since_us` guard;
`activity_days_advance()` refuses backward dates and a full outbox instead of
dropping a completed day; the lock order `s_bhy2_mux` → `s_persist_mux` →
`s_step_mux` is never inverted. The live proof: `steps.csv` reads
`day_steps=1146, lifetime=82971` at 13 Oct 23:18 and
`day_steps=0, lifetime=82971` at 14 Oct 04:06 — the daily counter rolled at the
next boot and the lifetime total survived all three night reboots.

1. [fixed, awaiting device validation] **Medium: light sleep was credited as
   activity time.** The activity clock now closes at AP-suspend and restarts at
   AP-resume; host tests cover a sleep gap. `UNKNOWN` retains its existing
   meaning for awake intervals where the classifier has no active class.
2. [pending] **Low: dead legacy NVS keys.** `step_base_load()` reads
   `STEP_NVS_DAY`, `STEP_NVS_TOTAL`, `STEP_NVS_DAY_START` and `STEP_NVS_DAY_LAST`,
   and nothing in the tree writes any of them; `STEP_NVS_DAY_ID` is neither read
   nor written. Harmless, but they read like live state and are not.

# Clean-card check of the boot forensics build (15. Oct)

The SD card was formatted so the next wear run starts with no pre-existing log,
and the watch was read back ~40 min into the first session on the fresh card.
Read-only set: `synclog`, `rtcdump`, `rails`, `crashls`, `dailycat`.

## What the build now records (all four hooks confirmed on hardware)

- `I (1718) power_mgmt: boot: reset=power-on wake_causes=0x1` is the first line
  of `syslog.txt`. The session was at 1,302 s when read and the file ended at
  49 s, which is the point of the end-of-`app_main()` flush: a session that dies
  after a few seconds now leaves its boot banner instead of nothing.
- The 60 s mirror cadence still works: the `rtcdump` line logged at uptime
  1,013 s appeared in `syslog.txt` on a later read (file 3,244 B to 3,367 B).
  Nothing else was logged between 49 s and 1,013 s because the watch sat still
  on USB power, so there was nothing to flush - not a stalled flush.
- `Ultra-Sparmodus disabled at boot (NVS)` is present, so the persisted flag is
  now visible in the log without asking the device.
- `rails` prints the armed gauge levels beside the live SOC, which is what
  calibrating the 3 % shutdown level needs.
- `crashls: no crash dir` on the fresh card, as expected after a format.
- CSVs are continuous at one row/minute for the whole session (22 battery rows,
  uptime 4 s to 1,270 s, 54 % to 69 % while charging), and the boot wrote two
  rows at uptime 4 s and 19 s. That double row is the same signature the 12 Oct
  zombie boot wrote, so it is normal boot behaviour, not a fault.
- `steps.csv`/`activity.csv` behave while the watch lies still: `day_steps=1866,
  lifetime=91103` unchanged, activity `unknown` on the first row then `still`.

## Findings

1. [done] **Low: the Ultra-Sparmodus boot line is printed twice per boot**
   (`uptime 1766` and `2233` in the same session). `power_mgmt_load_config()`
   runs from `lvgl_app_start()` (`lvgl_app.c:911`, needed early for the
   brightness) and again from `power_mgmt_init()` (`lvgl_app.c:982`), so the NVS
   read and its log line both happen twice. A doubled boot line is exactly the
   kind of thing that makes a post-mortem read "two boots" where there was one.
   **Fixed (15. Oct):** the line is logged once per boot by a static guard in
   `pm_config_load()`; the reload itself stays, it is idempotent.
2. [done] **Low: `wake_causes=0x1` on a cold boot reads like a fault.** See the
   correction below - the raw bitmap is meaningless to whoever reads the log
   after the fact, and the interesting case (a Sparmodus timer wake) is buried
   in hex.
   **Fixed (15. Oct):** `power_mgmt_log_boot_reason()` now masks out the
   `UNDEFINED` bit and names the rest, so a cold boot logs `boot: reset=power-on`
   and a Sparmodus timer wake logs `boot: reset=deep sleep wake_causes=timer`.
3. [pending] **Low: three AXP2101 I2C read failures in the first 4 s of boot** -
   `axp2101_is_rail_enabled(336): read ldo_onoff0` at 2,889 ms and
   `axp2101_set_bit(137): read reg 0x90` at 3,526 and 3,626 ms. Reg 0x90 is
   `LDO_ONOFF0`, so these are rail enable/disable read-modify-write attempts
   failing; the callers log and continue, so one rail toggle at boot may simply
   not have taken effect. The preceding 24 h of `syslog.txt` contain zero of
   these lines, so this is either new or tied to this boot (fresh card, VBUS
   just applied). Needs a second boot to see whether it reproduces.

## Corrections to earlier reasoning (do not re-raise)

- "`wake_causes=0x8` is TIMER and `0x1` is EXT0/GPIO" is only half right.
  `esp_sleep_get_wakeup_causes()` returns `BIT(esp_sleep_source_t)`, so 0x8 is
  `BIT(ESP_SLEEP_WAKEUP_TIMER)` = 3, correct. But 0x1 is
  `BIT(ESP_SLEEP_WAKEUP_UNDEFINED)` - the bit IDF sets for any boot that was not
  a sleep wake. So `boot: reset=power-on wake_causes=0x1` is the normal cold-boot
  reading, and the `waking: src=0x100 causes=0x1` line on a light-sleep wake
  means the cause was not attributed, not that EXT0/GPIO8 fired.
