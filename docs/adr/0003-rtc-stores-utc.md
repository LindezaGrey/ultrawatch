# The PCF85063A RTC hardware stores UTC, not local time

Previously the RTC held local wall-clock time (with the intent that GNSS/system-clock code would convert to UTC as needed). We've switched the hardware itself to store UTC directly: `pcf85063a_time_t` fields read from and written to the chip are now a plain UTC calendar breakdown, never local. Local time is computed only at the point something needs to display or interpret it for a human (watch face main display, night-mode hour check, daily-log day-rollover, CSV timestamps), via the new `pcf85063a_time_to_epoch()`/`pcf85063a_epoch_to_time()` helpers (TZ-independent civil-date math) plus `localtime_r()`/`mktime()` and the process TZ (set once in `uwatch_main.c`).

This removes the local-time ambiguity around DST "fall back" (one wall-clock hour occurs twice, `mktime`'s `tm_isdst=-1` guess could pick either UTC offset) from every place that used to read the RTC and assume it was already local: `twatch_board.c`'s `sync_system_time()`, `m10q.c`'s GPS-vs-RTC sync and MGA-INI time seed, `power_mgmt.c`'s night-mode check, `daily_log.c`'s day-rollover and CSV log, and `lvgl_app.c`'s watch face. The GNSS sync path in particular gets simpler: comparing GPS UTC against the RTC, and writing GPS UTC back into the RTC, are now direct epoch operations with no TZ conversion at all.

## Alarm / DST consequence (accepted)

`alarm.c`'s `rtc_arm_alarm()` still lets the user set a LOCAL alarm time (`s_cfg.hour`/`min`), but now must convert that to UTC before writing it into the PCF85063A's hardware alarm-compare registers, since those registers match against the RTC's own UTC clock. The conversion uses **today's** date (at arm time) to pick the DST offset. Because the hardware alarm masks day/weekday (it fires at the same `hh:mm:00` UTC every day, not a single date), that offset is baked in until the alarm is next re-armed — which happens on every set/toggle and on every dismiss-then-re-arm-for-next-day cycle, but not automatically at the moment DST itself changes.

**Consequence**: an alarm armed shortly before a DST transition, left untouched, fires at the wrong local wall-clock time (off by the DST delta, typically 1 hour) until something re-arms it (the user touching the alarm screen, or the alarm firing and being dismissed).

## Considered Options

- **Re-arm the alarm automatically at each DST transition** (e.g. a scheduled recheck at 2/3 AM local on transition days): rejected as the more correct option, but adds real complexity (detecting the transition, a new scheduled task, races with an alarm that's mid-ring at the transition instant) for an edge case that affects at most one ring per DST transition (twice a year) and only when the alarm isn't touched or dismissed in between.
- **Accept the edge case, document it (chosen)**: the alarm is still correct the overwhelming majority of the time (any set/toggle/dismiss re-arms it with the then-current offset), and the failure mode is a known, bounded, once-or-twice-a-year mistiming rather than a crash or data-loss risk.
