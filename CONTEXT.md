# UWatch

FreeRTOS/ESP-IDF smartwatch firmware for the LilyGO T-Watch Ultra (ESP32-S3): sensors, GNSS, alarms, and light-sleep power management driven from `main/` and `components/`.

## Language

### Power management

**Wake source**:
One of the four hardware-interrupt-attributed reasons a light-sleep wake occurred: Power key (PWRKEY), Boot button (BOOT), Gesture (IMU), or Alarm/Snooze (RTC). Each drives source-specific post-wake behavior (e.g. Alarm/Snooze wake runs `alarm_handle_wake()`). Tracked as a bitmask so simultaneous sources are each still individually handled, not just the last one to fire.
_Avoid_: wake gpio, wake reason

**Wake-capable GPIO**:
Any pin armed as a light-sleep wake trigger (`gpio_wakeup_enable`), a hardware-level fact distinct from Wake source. Touch is wake-capable but is *not* a Wake source: it has no source-specific post-wake behavior, so it isn't ISR-attributed and can't be distinguished from other wakes.
_Avoid_: wake pin (when a Wake source is meant)

**Armed** (of a wake source):
A wake source is armed when it is currently configured to trigger a light-sleep wake — e.g. gesture wake is armed only outside night mode and only once the BHI260AP's wake-up FIFO is confirmed drained; alarm/snooze wake is armed only while an alarm is set (`alarm_is_armed()`).
_Avoid_: enabled, active
