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
2. `[pending]` **power_mgmt.c:121 — `s_wake_gpio` last-writer-wins with simultaneous wake sources.**
   Gesture + alarm waking together attribute to one source; the other's selective ISR re-arm is skipped. Impact limited by the 1 Hz `alarm_check` polling while awake, but compounds with #1.

## Medium
3. `[pending]` **tracking.c — tracking task holds `s_mux` across I2C reads + `esp_lv_adapter_report_activity()` (tracking.c:139-170).**
   Can add ms-scale blockage to UI-side tracking reads. Shrink the critical section (sample step count outside the lock).
4. `[pending]` **m10q.c — data mutex taken inside ubxlib rx callbacks (`pos_cb`, `nav_status_cb`).**
   Correct, but verify ubxlib task priority vs UI priority for priority inversion.
5. `[pending]` **`imon` busy-polls GPIO8 for 30s** — debug-only, low CPU impact, minor.
6. `[pending]` **Display TE/VSync flush sync (co5300).** Panel TE is already enabled (DCS 0x35, co5300.c:26) and routed to `TWATCH_PIN_DISP_TE` GPIO6 (twatch_board.h:52), but GPIO6 is unused — LVGL flush is not gated on VSync. Before pursuing: confirm GPIO6 actually toggles at VSync (pin-watch/scope). If it does, an IRAM ISR + holding each 48-row band flush until the TE edge is the integration path. Enhancement, not a correctness fix — current partial-band QSPI DMA flush already avoids vTaskDelay pacing.

## Low
- Mixed brace styles + magic numbers remain (already tracked in issues.md).
- `ring_task` final `heap_caps_free()` closes the leak from issues.md (alarm.c:279-281) — verify on review.

## Process / Validation (blocked)
- **Wrist-tilt wake must be validated on battery**, not USB: sleep is skipped while VBUS present (`power_mgmt_enter_sleep` returns ESP_ERR_NOT_SUPPORTED). Steps: `sdclear` → unplug USB → wait ~5s for sleep → wrist-tilt → reconnect → `sdin`, look for `entering sleep` + `AP suspend ... gpio8=1` then `wake: gpio=8` + `Auto sleep exited`. SD log currently shows zero sleep entries (29x `Auto sleep enter callback failed`).

---

# Code Review Findings (17. Aug full re-review)

Full read-only review of main/, components/ (Bosch vendor lib at integration points only). Verified closed: `ring_task` buffer free (alarm.c:279-284), coredump erase-on-CRC (crash_dump.c:184-194, confirmed live on watch), m10q UART driver delete + ttfs persistence, tracking mutex.

## Medium
1. `[pending]` **GNSS rail (BLDO1) never turns off when GPS is disabled** — axp2101.c:181 enables BLDO1 at boot; power_mgmt.c:480 re-enables it unconditionally on every wake; the only cut path `m10q_power(false)` early-returns when `s_powered == false` (m10q.c:603), which is always the case with the GPS switch off. Net: MIA-M10Q draws ~25-30 mA forever. Currently masked because the GPS switch is left on.
   **Fix:** cut BLDO1 directly in `power_mgmt_enter_sleep()` when `!lvgl_gps_enabled()`; restore it in `exit_sleep()` only when GPS is enabled.
2. `[pending]` **Battery runtime estimate diverges after the first 5 min** — sensor_cache.c:78-85: when the gauge window fills, `first_ms` is re-based forward but `first_pct` keeps the original window-start sample, so rate = (total % change since first sample) / ~5 min and grows wronger with uptime.
   **Fix:** keep a small sample history (or at least re-pair first_pct with the new first_ms).
3. `[pending]` **BLE vprintf hook recurses infinitely (dead-code landmine)** — ble_debug.c:364: `ESP_LOGI` inside `ble_debug_vprintf` while `s_capturing` routes through the same hook again → unbounded recursion → stack overflow the moment BLE is re-enabled and a command runs. `ble_debug_init()` is currently commented out (uwatch_main.c:914).
   **Fix:** remove that ESP_LOGI or emit via the chained `s_prev_vprintf`.

## Low
4. `[pending]` **DST inconsistency** — twatch_board.c:136 uses `struct tm tm = {0}` (tm_isdst=0) while lvgl_app.c:292 and m10q.c:165 use tm_isdst=-1; system clock ends up 1 h off during CEST (crash timestamps, time()).
5. `[pending]` **parse_nav_sat() writes s_fix without s_data_mux** (m10q.c:418, via nav_sat_cb) — inconsistent with pos_cb/nav_status_cb; torn satellite list reads possible.
6. `[pending]` **m10q_get_stats() reads s_stats unlocked** (m10q.c:787) while pos_cb mutates it.
7. `[pending]` **Night-mode red filter off-by-one** — lvgl_app.c:198: draw-bitmap coords are inclusive, count should be (x_end-x_start+1)*(y_end-y_start+1); last row/col of each flushed band stays unfiltered (faint fringe).
8. `[pending]` **alarm ring_start() resets s_ring_mode_pending unconditionally** (alarm.c:287-292) — a snooze-timer fire landing on a dismiss press eats the dismiss and re-rings. Guard with `if (s_ringing) return;`.
9. `[pending]` **Watch-face month index unguarded** — lvgl_app.c:307: `mon[t.month - 1]` with a garbled RTC month 0 reads out of bounds (weekday is range-checked, month isn't).
10. `[pending]` **sd_log_flush() drops ring content when fopen fails** (sd_log.c:354-359) — transient SD/MMC errors silently lose log lines; keep s_ring_len on open failure.
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