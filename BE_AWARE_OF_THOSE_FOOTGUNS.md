# Be aware of those footguns

This note records failures that occurred during UltraWatch development. Read it
before you add a feature that uses more than one subsystem.

## Failures that caused resets or crashes

### RTC alarm interrupt loop

The alarm could cause an interrupt watchdog reset. The PCF85063A holds its INT
line low until firmware clears the alarm flag. GPIO interrupt settings can also
remain active after an ESP32-S3 software reset. A retained RTC, touch, or PMIC
interrupt could therefore run while the system was still starting.

The fix has two parts:

- Mask all shared wake sources before the ISR service starts.
- Let the ISR only disable its GPIO source and notify the alarm task. The task
  clears the RTC flag. It enables the GPIO interrupt again only after INT is
  high.

Do not access I2C, start audio, drive the haptic motor, allocate memory, or draw
the screen from an ISR.

### Weather fetch exhausted internal memory

Opening Weather started Wi-Fi, TLS, HTTP, JSON parsing, SD access, and a full
screen update in a short period. TLS and Wi-Fi need internal RAM. Display DMA
buffers also need internal DMA-capable RAM. The JSON tree and response body made
the remaining internal heap small and fragmented. A later allocation or SPI
queue operation then failed.

The fixes were:

- Store HTTP response data in PSRAM.
- Install the cJSON PSRAM allocator before any service can create a JSON tree.
- Keep FreeRTOS task stacks in internal RAM. Do not put an active task stack in
  PSRAM.
- Delete the response tree before the display refresh.
- Keep Wi-Fi in `WIFI_PS_NONE` during the transfer. Restore
  `WIFI_PS_MIN_MODEM` after all transient network data is released.
- Load a valid cache first. Refresh it no more than once per hour.
- Log free internal heap and the largest internal block before TLS and before a
  display refresh.

Free heap alone is not sufficient. A large total can still have no block large
enough for TLS or a DMA transaction.

### Recoverable display errors caused aborts

`ESP_ERROR_CHECK` is suitable for a required boot dependency. It is dangerous
in a runtime path. A failed SPI queue operation during memory pressure used to
abort the process and restart the watch.

The display queue now reports the error, releases the SPI bus, and stops the
current frame transfer. The next refresh can recover. New runtime code must
return a status or publish an error state. It must not convert a temporary I/O
or allocation error into a reboot.

## Failures that looked like crashes

### Several clients controlled the SD card

Icons, maps, Wi-Fi profiles, and weather cache all use the same SD card and
power rail. Separate mount and unmount code let one client remove power while
another client still had files open.

All clients now use `sd_storage_acquire()` and `sd_storage_release()`. The
service owns its mutex, mount state, reference count, SPI bus, and ALDO1 power.
Only the last release can unmount the card and remove power. Every successful
acquire must have one release on every exit path.

Never format the card after a mount error. Report the error and keep the Watch
screen usable.

### Android rejected concurrent BLE operations

Android Web Bluetooth rejects overlapping GATT procedures more often than
desktop Chrome. Starting notifications and reading all characteristics in
parallel made the connection slow or caused it to fail.

The web page now sends all GATT work through one promise queue. Profile
administration uses 20-byte packets and waits for one indication before it
sends the next packet. Keep this serialization for each new characteristic.

When BLE is switched off, stop advertising and terminate the active connection.
These are separate operations.

### Partial display updates left old pixels

The large clock glyphs extended outside the first update rectangle. Padding did
not fix the fault because the transfer was still too small and was not aligned
with the display band geometry. Scaled 72 px glyphs also distorted their lower
edges.

The clock now uses native 120 px Cascadia Code glyphs. Its update is a
full-width, TE-synchronized band on `DISPLAY_BAND_ROWS` boundaries. Redraw each
object that overlaps that band before the transfer.

### The degree glyph was present but invisible

The text renderer uses one-byte character codes. The degree glyph used byte
`0xB0`, but the lookup table had only 128 entries. The renderer selected the
blank glyph.

The lookup now has 256 entries. The font generator writes non-ASCII byte values
as fixed three-digit octal escapes. Do not put a UTF-8 character directly into
a string for this renderer. UTF-8 would produce two bytes and two glyph cells.

### The RTC does not charge its backup cell

The PCF85063A uses the board's `VBACKUP` supply, but it has no battery charger.
The AXP2101 charges the MS621FE cell. Register `0x18`, bit 2 enables its 100 µA
charger. Register `0x6A` sets the termination voltage. Set it to 3.1 V for the
MS621FE and verify both registers after each write. The main-cell charger uses a
different bit.

## Rules for a feature that uses several subsystems

Define ownership before implementation:

- The UI task is the only display owner.
- An ISR only records or signals an event.
- The SD service owns the SD SPI bus and card power.
- The Wi-Fi manager owns the Wi-Fi driver and power-save state.
- BLE operations in the browser use one queue.
- A temporary sensor user takes a lease and releases it when the app closes.

Keep startup small. Draw the Watch face before SD mounting, Wi-Fi startup,
network requests, asset loading, or sensor initialization that the clock does
not need.

For each asynchronous operation, specify:

1. Who owns the task and its data.
2. Which mutex, queue, or lease protects each shared resource.
3. What happens if the screen closes during the operation.
4. What releases memory, files, power rails, and temporary sensor leases.
5. What the user sees for loading, cached data, and an error.

Use internal RAM only for data that requires it, such as DMA buffers, task
stacks, and some radio or TLS allocations. Put large frames, response bodies,
atlases, and JSON trees in PSRAM. Release temporary objects before a full-screen
transfer.

Do not hold an SD lease, I2C lock, SPI bus, or state mutex while you wait for
Wi-Fi, BLE, a touch event, or another task. This can block an unrelated service
and can create a deadlock.

Write SD configuration and caches through a temporary file and a backup. Flush
the temporary file before rename. Never publish a partial record. Do not log
passwords or API keys.

## Hardware test checklist

Test more than the normal path:

- Cold boot and software reset.
- Boot with no SD card and with a damaged configuration file.
- Repeated app open and close operations.
- Wi-Fi loss during TLS and SD writes.
- BLE reads while Wi-Fi and GPS are active.
- Alarm while another app owns SD or GPS.
- Touch wake and full Watch redraw.
- Low-memory behavior with internal free heap and largest-block logs.

For a crash, save the complete panic output and backtrace first. Do not diagnose
from the last visible screen. That screen can be unrelated to the failing task.
