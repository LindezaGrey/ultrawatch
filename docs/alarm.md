# Alarms and countdown timers

The watch supports up to eight independent alarms plus one countdown timer.
Alarm configuration is stored as an NVS blob in namespace `alarm`, key `cfg2`.

## Alarm entries

Each alarm contains:

- enabled/disabled state;
- local hour and minute;
- ring mode: beep, vibration, or both;
- weekday mask (`bit0=Sunday` through `bit6=Saturday`).

The PCF85063A provides one hardware alarm slot. `alarm.c` therefore computes
the next occurrence across all enabled entries and programs that specific UTC
weekday/time. It recomputes after every add, edit, delete, enable change, and
completed ring. Local-to-UTC conversion is performed for the actual future
occurrence, including its applicable DST offset.

## User interface

The Alarms screen is part of the horizontal navigation ring, after Bluetooth
and before NFC.

- `+ Alarm` opens the alarm editor.
- Hours change in one-hour steps and minutes in 30-minute steps.
- Any weekday combination can be selected.
- Each row has an enable switch; tapping the row edits or deletes it.
- `+ Timer` offers 1/5/10/15/30/60/90/120-minute presets.
- An active countdown and Cancel button are shown on the list screen.

## Ringing and snooze

The shared ring engine serves both alarms and countdown timers. Beep playback
uses the MAX98357A, vibration uses the DRV2605, and `both` synchronizes them.
The global alarm-sound setting can mute both outputs.

Alarm rings offer Dismiss and a fixed 10-minute Snooze. Countdown timers offer
Dismiss only. PWR and BOOT also dismiss a ring. An unanswered alarm is
automatically snoozed at most six times; the generated speaker amplitude rises
from roughly 10% to 50%, then the final ring continues until dismissed.

The RTC interrupt on GPIO1 wakes the watch for an alarm or snooze expiry.
While already awake, `alarm_check()` polls the RTC flags once per second.

## Console commands

| Command | Effect |
|---|---|
| `alarm` | Show ring/armed state and command help. |
| `alarmls` | List configured alarms and slot indices. |
| `alarmadd <hh> <mm> [beep\|vib\|both] [wmask]` | Add an alarm. |
| `alarmrm <idx>` | Remove an alarm. |
| `alarmen <idx> <0\|1>` | Disable or enable an alarm. |
| `alarmring` | Start a test ring. |
| `alarmdismiss` | Dismiss the active ring. |
| `alarmsnooze` | Snooze the active alarm. |
| `timer <seconds>` | Start a countdown. |
| `timer off` | Cancel the countdown. |

Implementation: `main/services/alarm.c`, `main/services/cd_timer.c`,
`main/screens/alarm_screen.c`, and `main/screens/ring_screen.c`.
