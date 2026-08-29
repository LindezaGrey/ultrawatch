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

**Power rail**:
One of the AXP2101 PMU's 7 switchable outputs (ALDO1-4, BLDO1-2, DLDO1), each dedicated to one device: ALDO1=SD card, ALDO2=display, ALDO3=LoRa, ALDO4=sensor, BLDO1=GNSS, BLDO2=speaker, DLDO1=NFC (`docs/hardware.md`'s AXP2101 power tree table is canonical). DLDO2 exists on the chip but isn't routed to anything on this board.

The one rail that is *not* only its named device's: **ALDO1 is also the SPI2 bus rail whenever an SD card is seated**, because a seated-but-unpowered card clamps the shared MISO net and every read on SPI2 then returns `0x00` — including the SX1262's and the ST25R3916's. So ALDO1 stays on while a card is present, at the cost of that card's idle draw, and is cut only when the socket is empty. Don't reason about it as "the SD card's rail" when deciding what to power down.
_Avoid_: channel, LDO

**Init a rail** (`axp2101_init_rail`):
One-time bring-up: configure a rail's voltage and turn it on. Called once per rail at boot (`axp2101_set_default_power`); rewrites the voltage register every call, so it's not a cheap way to just toggle a rail.
_Avoid_: set a rail

**Enable a rail** (`axp2101_enable_rail`):
Runtime on/off at whatever voltage a rail was last Init'd with — never touches the voltage register. The only rail operation used for sleep/wake gating.
_Avoid_: set a rail, toggle a rail

**RTC backup battery**:
The Seiko MS621FE-FL11E rechargeable coin cell on the AXP2101's VBACKUP pin (per the T-Watch Ultra schematic), distinct from the main Li-ion Cell battery. Keeps the PCF85063A RTC and BHI260AP VBACKUP time alive across power loss. Its charge-enable bit (AXP2101 REG 0x18 bit 2) defaults disabled and resets to disabled on every system reset — `axp2101_set_default_power` explicitly enables it, since without that write it's silently never charged.
_Avoid_: button battery (datasheet's term, kept here as a synonym since it names the same REG 0x18 bit; prefer "RTC backup battery" in prose since "button battery" reads as generic)
