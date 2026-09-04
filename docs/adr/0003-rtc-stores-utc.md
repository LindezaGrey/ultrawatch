# The PCF85063A RTC hardware stores UTC, not local time

Previously the RTC held local wall-clock time (with the intent that GNSS/system-clock code would convert to UTC as needed). We've switched the hardware itself to store UTC directly: `pcf85063a_time_t` fields read from and written to the chip are now a plain UTC calendar breakdown, never local. Local time is computed only at the point something needs to display or interpret it for a human (watch face main display, night-mode hour check, daily-log day-rollover, CSV timestamps), via the new `pcf85063a_time_to_epoch()`/`pcf85063a_epoch_to_time()` helpers (TZ-independent civil-date math) plus `localtime_r()`/`mktime()` and the process TZ (configured by `CONFIG_UWATCH_TIMEZONE` and set once in `uwatch_main.c`).

This removes the local-time ambiguity around DST "fall back" (one wall-clock hour occurs twice, `mktime`'s `tm_isdst=-1` guess could pick either UTC offset) from every place that used to read the RTC and assume it was already local: `twatch_board.c`'s `sync_system_time()`, `m10q.c`'s GPS-vs-RTC sync and MGA-INI time seed, `power_mgmt.c`'s night-mode check, `daily_log.c`'s day-rollover and CSV log, and `lvgl_app.c`'s watch face. The GNSS sync path in particular gets simpler: comparing GPS UTC against the RTC, and writing GPS UTC back into the RTC, are now direct epoch operations with no TZ conversion at all.

## Alarm scheduling amendment

The original single-alarm implementation converted a local time using the
current day's UTC offset and programmed a repeating UTC hour. That created a
one-hour error across a DST transition until the alarm was re-armed.

The later multi-alarm scheduler supersedes that behavior. For every enabled
weekday entry it constructs the actual next local calendar occurrence with
`tm_isdst = -1`, converts that future occurrence to UTC, and programs the
PCF85063A for its specific UTC weekday/hour/minute. It recomputes after every
list change and after every completed alarm. Consequently the upcoming alarm
uses the DST offset applicable when it will fire; the previously accepted DST
edge case no longer applies.
