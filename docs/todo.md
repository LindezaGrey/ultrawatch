# UWatch resolved engineering review log

The 2026-10-10 section below has open `[pending]` items. All earlier sections
are resolved history; their previously open items were also tracked in
`docs/issues.md`.

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
  measurement artifact: `esp_timer` pauses during light sleep and the first
  sample of a run can be stale. Continuous single-session sampling disproved it.
