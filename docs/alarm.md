# Alarm Clock — Plan

Daily alarm with selectable ring mode (beep / vibration / both), snooze (10 min).

## Config

Persisted in NVS namespace `"alarm"` / key `"cfg"`:

```c
typedef struct {
    bool  enabled;
    uint8_t hour, min;      /* daily alarm time */
    uint8_t ring_mode;      /* 0=beep, 1=vibration, 2=beep+vibration */
} alarm_config_t;
```

`alarm_set(h, m, enabled, ring_mode)` — ring mode selectable per alarm.

## New module: `main/alarm.c` + `alarm.h`

- `alarm_init()` — load config, register GPIO1 ISR + wake source.
- `alarm_arm()` / `alarm_disarm()` — enable/disable `AIE` + arm/disarm GPIO1 wake.
- `alarm_is_armed()`.
- `alarm_handle_wake()` — fires the ring if the RTC `AF` flag is set.
- `alarm_start_ring()` / `alarm_stop_ring()`.
- `alarm_dismiss()` — stop ring, clear AF + AIE, re-arm for next day.
- `alarm_snooze()` — RTC countdown timer 600 s (10 min), wakes on `TIE`.

## Ringing (respects `ring_mode`)

- **beep**: enable `BLDO2` amp rail, `max98357a_write()` repeats a 3× beep melody
  (reuse the `tone` sine approach).
- **vibration**: `drv2605_play(wave)` pulses on the same cadence.
- **both**: beep + haptic synchronized.
- Loops until Dismiss/Snooze; `esp_lv_adapter_report_activity()` keeps the watch
  awake.

## Integration

1. **power_mgmt.c** — arm `TWATCH_PIN_RTC_INT` (GPIO1) as a LOW_LEVEL light-sleep
   wake source while an alarm is armed (mirror the IMU wake pattern); route a
   wake to `alarm_handle_wake()`.
2. **lvgl_app.c** — alarm **set screen** (swipe **up** from the watch face):
   HH:MM +/- buttons, on/off toggle, ring-mode selector (Beep / Vibration /
   Both), "Set" confirm. Exempt from the menu inactivity timeout.
3. **lvgl_app.c** — **ringing screen**: "ALARM HH:MM", **Snooze 10 min** +
   **Dismiss** buttons; auto-shown on wake via `esp_lv_adapter_request_wake()`.
4. **uwatch_main.c** — console commands for headless testing:
   - `alarm <hh> <mm> [beep|vib|both]`
   - `alarm off`
   - `alarm` (status)

## Edge cases

- Daily repeat: clear `AF` + re-arm after each ring (RTC battery-backed, survives
  reboots).
- Wake from light sleep via GPIO1; ring detection while awake via the 1 Hz
  `watch_face_update` timer checking `AF`.
- Snooze = 10 min via the RTC countdown timer (`pcf85063a_set_timer_minutes(10, true)`,
  using the 1/60 Hz clock `TCF=11`, so the value is in minutes). The PCF85063A
  automatically re-loads and loops a countdown, so `alarm_check()` calls
  `pcf85063a_timer_stop()` (TE=0 + clear TF) the moment it observes TF, making
  the snooze a one-shot at the software level.
- INT routing: the RTC pulls GPIO1 LOW while AF/TF is set (TI_TP=0, "INT follows
  flag") and holds it until the flag is cleared. power_mgmt registers a GPIO1
  edge ISR (like PWRKEY) and arms GPIO1 as a LOW_LEVEL light-sleep wake source
  while `alarm_is_armed()`. A falling edge wakes `pm_wake_task`, which calls
  `alarm_handle_wake()` (reads AF/TF → `ring_start()`) and restores the edge ISR.
- `BLDO2` restored before playback, off after the ring stops.
