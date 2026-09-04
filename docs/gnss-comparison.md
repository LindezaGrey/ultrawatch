# GNSS implementation summary

This document previously compared the current UWatch driver with an older
`main/gps.c` implementation from another checkout. Those files are not part of
this repository, so the stale comparison has been replaced with a summary of
the implementation that can be verified here.

## Receiver and protocol

- u-blox MIA-M10Q on UART, with baud probing at 38400/115200/9600 and normal
  operation at 115200 baud.
- NMEA GGA/RMC/GSV provide fix, speed, course and per-satellite skyplot data.
- UBX NAV-PVT supplies measured horizontal accuracy; MON-VER identifies the
  module; MON-RF exposes AGC diagnostics.
- Validity requires the expected NMEA fix status. Position is stored in
  degrees, altitude in metres, and horizontal accuracy in millimetres.

## Configuration and aiding

- GPS, Galileo, BeiDou B1I, QZSS and SBAS are enabled; B1C and GLONASS are
  disabled.
- The receiver uses automatic 2D/3D fix mode with a three-satellite minimum.
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
