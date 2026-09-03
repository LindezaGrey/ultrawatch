# UWatch TODO: docs/m10Q.md vs Code Implementation Differences

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

## Build & Flash Status
- ✅ Build: Successful (Docker espressif/idf:v6.0.2)
- ✅ Flash: 1,398,656 bytes written to ESP32-S3 via /dev/ttyACM0
- ✅ Verified: Data verified after write, hard reset performed

---

# Code Review Findings (16. Aug)

Status labels: `[pending]` not started, `[in-progress]`, `[done]`.

## High
1. `[pending]` **power_mgmt.c:261-271 — RTC/PWRKEY/BOOT GPIO ISR not re-enabled at sleep entry.**
   `button_isr` disables all pins on every wake (power_mgmt.c:132-135), but `pm_arm_gpio_wakeup` only calls `gpio_intr_enable()` for the IMU. After a non-RTC wake, the alarm/snooze light-sleep wake stops being armed until a future RTC-attributed wake re-enables it.
   **Fix:** `gpio_intr_enable(PM_GPIO_PWRKEY | PM_GPIO_BOOT | PM_GPIO_RTC)` alongside the `gpio_wakeup_enable()` calls in `pm_arm_gpio_wakeup`. Currently masked by the new 5s level-based safety nets; without it the worst-case alarm latency is ~5s.
   **Update (16. Aug):** partial fix applied while debugging the IMU — the settling logic in `pm_arm_gpio_wakeup` now retries sampling the IMU line over 250ms instead of a single 50ms check (bhi260ap_ap_suspend() drains leftover WU-FIFO events first). The IMU/gesture wake path is now clean; RTC/PWRKEY/BOOT ISR re-enable is still pending (see #1 unchanged).
2. `[done]` **power_mgmt.c — `s_wake_gpio` last-writer-wins with simultaneous wake sources.** (2026-08-27)
   Gesture + alarm waking together attribute to one source; the other's selective ISR re-arm is skipped. Impact limited by the 1 Hz `alarm_check` polling while awake, but compounds with #1.
   **Fix:** replaced `s_wake_gpio` with a `s_wake_sources` bitmask (`PM_WAKE_PWRKEY`/`BOOT`/`IMU`/`RTC`), OR'd in from `button_isr` under a critical section; `pm_wake_task` now processes every set bit in one pass instead of only the last writer. See ADR [0001](docs/adr/0001-wake-source-bitmask.md).

## Medium
3. `[done]` **tracking.c — tracking task held `s_mux` across I2C reads + `esp_lv_adapter_report_activity()`.** (2026-09-03)
   The task now samples activity and step count outside the state lock, then rechecks the session before applying results.
4. `[pending]` **m10q.c — data mutex taken inside ubxlib rx callbacks (`pos_cb`, `nav_status_cb`).**
   Correct, but verify ubxlib task priority vs UI priority for priority inversion.
5. `[pending]` **`imon` busy-polls GPIO8 for 30s** — debug-only, low CPU impact, minor.
6. `[pending]` **Display TE/VSync flush sync (co5300).** Panel TE is already enabled (DCS 0x35, co5300.c:26) and routed to `TWATCH_PIN_DISP_TE` GPIO6 (twatch_board.h:52), but GPIO6 is unused — LVGL flush is not gated on VSync. Before pursuing: confirm GPIO6 actually toggles at VSync (pin-watch/scope). If it does, an IRAM ISR + holding each 48-row band flush until the TE edge is the integration path. Enhancement, not a correctness fix — current partial-band QSPI DMA flush already avoids vTaskDelay pacing.

## Low
- Mixed brace styles + magic numbers remain (already tracked in issues.md).
- `ring_task` final `heap_caps_free()` closes the leak from issues.md (alarm.c:279-281) — verify on review.

## Process / Validation (blocked)
- **Wrist-tilt wake must be validated on battery**, not USB: sleep is skipped while VBUS present (`power_mgmt_enter_sleep` returns ESP_ERR_NOT_SUPPORTED). Steps: `sdclear` → unplug USB → wait ~5s for sleep → wrist-tilt → reconnect → `sdin`, look for `entering sleep` + `AP suspend ... gpio8=1` then `wake: gpio=8` + `Auto sleep exited`. SD log currently shows zero sleep entries (29x `Auto sleep enter callback failed`).
- `[done]` **BHI260AP gyroscope orientation matrix** (2026-08-27) — `bhi260ap_init()` sets `BHY2_PHYS_SENSOR_ID_GYROSCOPE`'s orientation matrix to `diag(-1,-1,-1)` (`bhi260ap.c`), replacing the app-side quaternion-conjugate hack previously in `lvgl_app.c`'s `bhi_cube_update()`. Fixes the rotation-sense mismatch at the source (before on-chip fusion) instead of only patching the cube widget, so gesture/activity detection benefit too. **Confirmed working on hardware**: cube tracks physical rotation correctly on all axes.

---

# Code Review Findings (17. Aug full re-review)

Full read-only review of main/, components/ (Bosch vendor lib at integration points only). Verified closed: `ring_task` buffer free (alarm.c:279-284), coredump erase-on-CRC (crash_dump.c:184-194, confirmed live on watch), m10q UART driver delete + ttfs persistence, tracking mutex.

## Medium
1. `[done]` **GNSS rail (BLDO1) never turns off when GPS is disabled** (2026-08-27) — `enter_sleep()` had since grown a `lvgl_gps_enabled()` check, but `exit_sleep()` still restored BLDO1 unconditionally on every wake, desyncing m10q's `s_powered` from the physical rail (`m10q_power(false)` then permanently no-ops) and leaving the rail stuck on (~25-30 mA) after the first wake with GPS off.
   **Fix:** made `exit_sleep()`'s BLDO1 restore conditional on `lvgl_gps_enabled()`, symmetric with `enter_sleep()`.
2. `[pending]` **Battery runtime estimate diverges after the first 5 min** — sensor_cache.c:78-85: when the gauge window fills, `first_ms` is re-based forward but `first_pct` keeps the original window-start sample, so rate = (total % change since first sample) / ~5 min and grows wronger with uptime.
   **Fix:** keep a small sample history (or at least re-pair first_pct with the new first_ms).
3. `[pending]` **BLE vprintf hook recurses infinitely (dead-code landmine)** — ble_debug.c:364: `ESP_LOGI` inside `ble_debug_vprintf` while `s_capturing` routes through the same hook again → unbounded recursion → stack overflow the moment BLE is re-enabled and a command runs. `ble_debug_init()` is currently commented out (uwatch_main.c:914).
   **Fix:** remove that ESP_LOGI or emit via the chained `s_prev_vprintf`.

## Low
4. `[done]` **DST inconsistency** (2026-09-03) — superseded by UTC RTC storage and `pcf85063a_time_to_epoch()` in `twatch_board.c`; system-clock sync no longer uses local-time conversion or `tm_isdst`.
5. `[done]` **`parse_nav_sat()` wrote `s_fix` without `s_data_mux`** (2026-09-03) — satellite-list updates now use the same mutex as other receiver callbacks.
6. `[done]` **`m10q_get_stats()` read `s_stats` unlocked** (2026-09-03) — readers and callback updates now take a coherent mutex-protected snapshot.
7. `[done]` **Night-mode red filter off-by-one** (2026-09-03) — superseded by the adapter’s end-exclusive draw-bitmap API. `area_rounder_cb()` explicitly converts LVGL’s inclusive invalidation bounds before this callback computes its pixel count.
8. `[done]` **Alarm `ring_start()` reset race** (2026-09-03) — superseded by the current ring state machine: `s_ringing` remains true until a pending dismiss/snooze is consumed, so RTC handling cannot start a second ring in that window.
9. `[done]` **Watch-face month index unguarded** (2026-09-03) — superseded by `localtime_r()` formatting in `screens/watch_face.c`; no direct RTC month-array indexing remains.
10. `[done]` **System-log flush dropped its buffered batch on SD failure** (2026-09-03) — `syslog_capture.c` now restores a failed batch ahead of concurrently captured lines when mounting, opening, writing, or closing the log file fails.
11. `[pending]` **Alarm auto-snooze has no retry cap** — observed 15 auto-snoozes over 2.5 h (SD log, 17. Aug). Consider a max-snooze count or escalating volume.

## Trivial
- uwatch_main.c:798 comment says `alarm HH:MM`, parser wants `alarm <hh> <mm>`.
- `crashread` has no `../` sanitization (debug-only interface).
- uwatch_main.c:107: a >255-char line without newline wedges the console buffer.
- lvgl_app.c:111: back-to-back GNSS requests can collapse (single volatile int, no queue).
- lvgl_app.c:1890-1903: duplicated/stale screenshot comment block.
- alarm.c:305-310: semaphore/task creation unchecked.
- co5300_deinit() leaks s_panel_io (dead code path).
- Boot DISP_PWR pulse is 50 ms vs 200 ms in `disppwr` (twatch_board.c:198-203 vs uwatch_main.c:547-550) — match margins if a stuck panel ever survives the boot pulse.
- `disppwr` sends only SLPOUT+brightness after a real power cut; works only if the CO5300 reloads OTP config on power-up — otherwise it needs the full s_init_cmds sequence.
