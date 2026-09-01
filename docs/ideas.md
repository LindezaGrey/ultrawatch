# LilyGo T-Watch Ultra - new ideas

Notes below keep the original idea first, then what checking it against the
code/hardware turned up. Verified facts are marked with the file or datasheet
id they came from, so they can be re-checked rather than trusted.

## Logging

**Idea.** Maybe needs a main switch. Logging of activities is always on: if the
BHI sends an activity interrupt, the ESP should wake up and process the data,
and if other devices are already powered, check them too for updated data, then
sleep again.

**Blocker: activity recognition cannot raise a wake interrupt.**
`BHY2_SENSOR_ID_AR` is id 63 and has **no `_WU` variant** (bhy2_defs.h:430).
Only ACC, GYRO, MAG, GRA, LACC, STC and STD have wake-up twins, so AR can never
assert the wake-up-FIFO interrupt. Enabling the non-wakeup FIFO interrupt
instead is not an option - it fires on every ACC/GYRO sample at 12.5 Hz.

Options, best first:

1. **Process activity on the existing once-a-minute housekeeping wake.**
   power_mgmt.c now arms an RTC timer wake (`PM_HOUSEKEEPING_WAKE_US`) purely
   so housekeeping keeps running through long sleeps. Draining the FIFO there
   costs nothing extra, and `parse_activity()` already credits duration per
   event in FIFO order, so no transition is lost between drains - that is
   exactly what it was written for.
2. `BHY2_SENSOR_ID_STD_WU` (step detector wake up, id 94) as a proxy wake: if
   steps are happening the activity class is probably changing, so wake on that
   and read AR from the normal FIFO.
3. Significant-motion as a coarser trigger.

**Main switch.** Worth having, and it should gate the *SD writes* specifically:
the card mount around each write is the expensive part, not the sampling.

**Storage.** Time-series belongs on the SD card (see the NVS note below), which
is what steps.csv / activity.csv / battery.csv already do. Keep NVS for small
state that has to survive a missing card.

## NVS size - applies to every idea below

The `nvs` partition is **0x6000 = 24 KB** (partitions.csv) and is already shared
by ten namespaces: alarm, ble_scan, drv2605, gps, haptic, m10q, mesh, pm, step,
track.

A MAC + lat + lon + RSSI record is ~50 bytes of payload, but NVS stores in
32-byte entries with per-blob overhead, so ~200 records would fit *if the whole
partition were free*, which it is not. One walk will pass 200 WiFi BSSIDs. And
"update only if stronger" means rewriting entries repeatedly - flash wear on the
same partition the step counter and alarms live in.

**So: these datasets go to the SD card as append-only CSV**, same shape as
battery.csv. Unlimited in practice, no wear concern, and post-processable on a
laptop. NVS only for small persistent state.

## Node-List

**Idea.** If a nodeinfo is received with 0 hops - only nodeinfos heard directly
from the originating node - store it persistently, so the table is split:
indirectly heard nodes in RAM, directly heard nodes persisted. If GNSS is active
with a good fix, add the coordinates so it is later visible where the node was
heard. RF parameters are already logged. If a coordinate is already stored,
update it only when the RF signal is stronger, so the strongest-signal position
is kept.

**The 0-hop definition is correct.** Meshtastic's packet header carries
`hop_start` and `hop_limit`; `hops_away = hop_start - hop_limit`, so directly
heard means the two are equal. The header is readable even though the payload is
encrypted.

**Two gaps in the current code.** `mesh_node_t` (mesh_log.h:86) holds `id`,
`name[32]`, `last_rssi_dbm`, `last_snr_db` and **no hop field** - the parser
needs extending to surface it. `MESH_NODE_TABLE_MAX` is 32, which is fine for
the RAM half but is the reason the persistent half needs its own store.

The RAM/persistent split is a good design. Persist to SD, not NVS.

## Bluetooth

**Idea.** If GNSS is on with a good fix, store MAC + coordinates + RF strength
per station; if the MAC already exists, update the coordinates only when the
signal is stronger.

Feasible as written. Same storage rule: SD, not NVS.

## WiFi

**Idea.** If GNSS is on with a good fix, store BSSID + SSID + coordinates + RF
strength; if the MAC already exists, update the coordinates only when the signal
is stronger.

Feasible as written. Note WiFi scanning draws heavily in bursts, so it wants to
be opt-in rather than always-on.

## Caveat shared by the three geo ideas

GNSS is now **off by default** - the BLDO1 rail was being left powered from boot
and was ~25-30 mA of the overnight drain (see the axp2101.c comment). So these
only collect data during deliberate GPS sessions. Design them as opportunistic
enrichment - a record with no coordinates is normal, not a failure - rather than
expecting the coordinate column to be populated.

## Super-Sparmodus

**Idea.** In Super-Sparmodus only the RTC and the buttons can wake the ESP from
deep sleep.

Feasible as written. All the relevant pins are RTC-capable on the ESP32-S3 (RTC
GPIOs are 0-21): BOOT = GPIO0, PWRKEY / AXP IRQ = GPIO7, RTC INT = GPIO1
(power_mgmt.c:38-42), so an `ext1` wake mask over those three works from deep
sleep. Mostly a narrowing of the existing path rather than new machinery -
`power_mgmt_sparmodus_enter_sleep()` already arms a timer wake.

Worth being explicit about what the S3 can and cannot do here, since it came up:
the chip wakes itself from deep sleep on its **internal** RTC timer, no external
part needed - the PCF85063A exists for wall-clock alarms that must survive power
off. What deep sleep *cannot* do is wake on arbitrary GPIOs or UART; that is
light-sleep only, which is where the touch/IMU wakes work today.
