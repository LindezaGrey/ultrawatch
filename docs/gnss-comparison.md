# GNSS implementation summary

This document previously compared the current UWatch driver with an older
`main/gps.c` implementation from another checkout. Those files are not part of
this repository, so the stale comparison has been replaced with a summary of
the implementation that can be verified here.

## Receiver and protocol

- u-blox MIA-M10Q on UART, opened at the module's factory 38400 baud. This
  driver does not probe alternate rates or change the module baud.
- UBX NAV-PVT provides position, time, speed, course, fix quality and accuracy;
  NAV-SAT supplies skyplot data.
- MON-VER identifies the module. MON-RF is unsupported by this receiver, so AGC
  is unavailable.
- Fix validity comes from the NAV-PVT fix type/flags. Position is stored in
  degrees, altitude in metres, and horizontal accuracy in metres.

## Configuration and aiding

- Constellation selection, dynamic model, and minimum satellite count are not
  configured by this driver; receiver defaults or persisted module settings apply.
- RTC UTC time and the persisted last-known position are sent as MGA-INI aiding
  when useful.
- TTFF, fix counts and the last-known position are persisted in NVS.

## Timekeeping

The PCF85063A stores UTC. A valid GNSS fix can update it directly without a
timezone conversion. Local time is derived only for display, alarms and
local-date logs through `CONFIG_UWATCH_TIMEZONE`.

GNSS 1PPS on GPIO13 measures RTC drift and updates the PCF85063A OFFSET
register. The calibration is persisted in NVS and can be repeated with
`rtccal`.

## Power and UI

BLDO1 is off by default. The receiver is powered for the GPS screen, the
boot-time position check, explicit diagnostics, and step-gated tracking fixes.
The VRTC backup rail remains active for warm/hot starts.

The GPS screen shows a satellite skyplot, position in decimal and DMS forms,
altitude, speed, course, satellite count and accuracy. GPX logging records a
valid point every 30 seconds while its session is active. Distance tracking is
a separate step-gated feature that requests a fix every 50 steps.

Implementation: `components/drivers/m10q/`, `main/screens/gps_screen.c`,
`main/services/gpx_log.c`, and `main/services/tracking.c`.
