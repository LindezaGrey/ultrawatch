# GNSS Feature Comparison — UWatch vs. ultrawatch

Comparison of the GPS/GNSS (u-blox MIA-M10Q) integration in the two projects:
**UWatch** (this repo, `components/drivers/m10q/`) and **ultrawatch**
(`projekte/ultrawatch`, `main/gps.c` + `main/gps_screen.c`).

## Protocol & data source

| Aspect | UWatch | ultrawatch |
|---|---|---|
| Primary protocol | **NMEA-0183** (GGA/RMC/GSV) | **UBX binary** (NAV-PVT only) |
| NMEA enabled? | Yes (default) | No — only UBX-NAV-PVT is enabled |
| UBX used | NAV-PVT for hAcc only; MON-VER for baud probe | NAV-PVT (fix), MON-VER, MON-RF, MGA-INI, CFG-VALSET/GET |
| Satellite skyplot data | Yes — GSV (per-sat az/el/SNR) | No skyplot (text screen only) |
| Fix validity | GGA quality>0 + RMC "A" | UBX flags bit0 + fixType 2/3 |

## Position / accuracy data

| Feature | UWatch | ultrawatch |
|---|---|---|
| Latitude / longitude | Yes (double, deg) | Yes (e-7 int) |
| Altitude | Yes (m) | Yes (mm) |
| Speed | Yes (km/h, from RMC) | No |
| Course / heading | Yes (deg, from RMC) | No |
| Satellite count | Yes (used + in-view) | Yes (SV count only) |
| HDOP | Yes (×10) | No |
| **Horizontal accuracy (hAcc)** | **Measured, from UBX-NAV-PVT (mm)** | No |
| Per-satellite az/el/SNR | Yes (GSV) | No |

## Fix-time / statistics

| Feature | UWatch | ultrawatch |
|---|---|---|
| TTFF measurement | Yes (avg / best, ms, NVS-persisted) | Yes (avg / best, ms, NVS-persisted) |
| Fix counter (total / today) | Yes (NVS) | Yes (NVS) |
| RTC sync from GPS | Yes (PCF85063A, UTC->local via TZ) | Yes (PCF85063A, UTC->local, <2 s guard) |
| **RTC drift calibration** | **Yes — 1PPS (GPIO13) measures drift, writes PCF85063A OFFSET (±64 ppm), NVS-persisted, re-applied at boot** | No |
| `rtccal` console command | Yes (re-run calibration on demand) | — |

## Config & aiding (fast lock)

| Feature | UWatch | ultrawatch |
|---|---|---|
| Baud probe (38400/115200/9600) | Yes | Yes |
| Raise UART to 115200 | Yes (CFG-VALSET) | Yes |
| Constellation config (GPS+GAL+BDS B1I+QZSS+SBAS) | Yes (CFG-VALSET) | Yes (CFG-VALSET, ordered, read-back-checked) |
| Fix mode / min SVs | Yes (auto 2D/3D, min 3 SV) | Yes (auto 2D/3D, min 3 SV) |
| **MGA-INI time aiding** (UTC epoch -> module) | Yes | Yes |
| **MGA-INI position aiding** (last fix from NVS) | Yes | Yes |
| RF diagnostics (MON-RF AGC, LNA mode) | Yes (AGC) | Yes |
| Cyclic power-save (CFG-PM2) | Yes | No (uses backup mode instead) |

## Power management

| Feature | UWatch | ultrawatch |
|---|---|---|
| Rail control | On-demand BLDO1 (off by default; on only on GPS screen) | BLDO1 always on after boot |
| Soft standby on power-down | UBX-RXM-PMREQ | UBX-CFG-PWR (flags 0x0005, UART-RX wake) |
| UART driver lifecycle | Installed once, kept (rail toggled) | Installed once, kept |
| Wake on touch | — (relies on rail power-up) | Single UART byte + re-aid |
| GPS keeps running during display sleep | No (off when screen closed) | Yes (display sleeps, GPS continues) |

## UI

| Feature | UWatch | ultrawatch |
|---|---|---|
| Screen | **Skyplot** (rings + sat dots by SNR, PRN) + position/speed/alt/sats/hAcc/UTC | Text: constellations, fix, position, TTFF stats |
| On-demand power on screen open | Yes | No (always powered) |
| Auto-sleep-while-acquiring / sleep-after-fix | Yes (report_activity gating) | Fixed 30 s inactivity |
| Debug console command | `gnss` (state, fix, per-sat) | None (screen only) |

## Strengths & gaps

**UWatch strengths:** satellite skyplot, per-sat az/el/SNR, measured hAcc,
speed/course, on-demand rail power (best battery), USB console debugging,
time+position aiding, constellation tuning, **PPS-disciplined RTC**, TTFF
stats, RF AGC diagnostics.

**UWatch gaps (vs ultrawatch):** none significant remain — the follow-ups below
have been implemented.

**ultrawatch strengths:** faster fixes via time+position aiding, constellation
tuning, RTC sync, TTFF stats, RF diagnostics.

**ultrawatch gaps:** no skyplot, no per-sat data, no speed/course/hAcc, GPS
always powered (higher idle current).

## UWatch follow-ups (now implemented)

1. **MGA-INI time + position aiding** on power-on — UTC epoch + last known
   lat/lon (from NVS) sent to the module for a fast first fix.
2. **RTC sync** from NAV-PVT on first fix (UTC -> local via configured TZ;
   PCF85063A).
3. **RTC drift calibration via 1PPS** — first fix also starts a PPS-measured
   drift correction written to the PCF85063A OFFSET register; persisted in NVS
   and re-applied at boot (`rtccal` re-runs it).
4. **TTFF + fix statistics** persisted in NVS (total, today, avg, best).
5. **Constellation config** — GPS + GAL + BDS B1I + QZSS + SBAS, B1C/GLO off.
6. **MON-RF AGC** debug (0 = weak, 8191 = saturated), exposed in `gnss`.
