# Preferred JTAG debugging workflow

The T-Watch Ultra exposes the ESP32-S3 USB JTAG interface alongside its CDC
serial port. Use JTAG for breakpoints, task inspection, register reads, and
control-flow checks during development; reserve flashing for firmware changes.

Validated tool versions on this host:

- OpenOCD `v0.12.0-esp32-20260424`
- Xtensa GDB `17.1_20260402`
- USB JTAG device VID/PID `303a:1001`

After installing ESP-IDF tools, load the environment:

```bash
cd /home/peter/esp/esp-idf
source ./export.sh
```

Start the ESP32-S3 built-in JTAG server (the command is intentionally kept
running):

```bash
OPENOCD=/home/peter/.espressif/tools/openocd-esp32/v0.12.0-esp32-20260424/openocd-esp32
"$OPENOCD/bin/openocd" \\
  -s "$OPENOCD/share/openocd/scripts" \\
  -f board/esp32s3-builtin.cfg
```

In a second terminal, attach GDB using the matching application ELF:

```bash
GDB=/home/peter/.espressif/tools/xtensa-esp-elf-gdb/17.1_20260402/xtensa-esp-elf-gdb/bin/xtensa-esp32s3-elf-gdb
"$GDB" build/twatch_ultra_phase1.elf
(gdb) target extended-remote :3333
(gdb) info threads
(gdb) thread apply all bt 3
```

Useful BHI260AP breakpoints:

```text
(gdb) break bhi_irq_isr
(gdb) break bhi_irq_task
(gdb) break accel_callback
(gdb) continue
```

The first GDB attach may reset and halt both cores because ESP-IDF memory
protection requires a controlled reset. This is expected and does not erase
flash. To catch early boot behavior, start GDB before releasing the reset and
set breakpoints before `continue`. Always detach cleanly with `detach` before
stopping OpenOCD so the USB JTAG interface is released.

JTAG cannot prove sensor FIFO contents by itself; combine breakpoints with the
task-local `bhy2_get_and_process_fifo()` return value and BHI register reads.
Use the serial port only for continuous logs after the debug session is
detached. Do not run OpenOCD and a serial flasher against the same USB JTAG
interface at the same time.

## One-shot I2C semaphore-timeout capture (d08530f only)

`tools/i2c_lock_timeout.gdb` verifies the preserved ELF SHA256 before connecting.
It arms one hardware breakpoint on the optimized failed-semaphore-take branch,
prints current/all-task stacks on the first hit, then disables breakpoints and
detaches to resume the target. Output: `/tmp/ultra-i2c-lock-gdb.log`. It makes no
target function calls. Starting it is a hardware-control action; follow the
normal debugger authorization and single-owner procedure. Stop serial capture
first. Attach may trigger the documented initial memory-protection reset.

With OpenOCD already running and the watch still on d08530f:

```bash
/home/peter/.espressif/tools/xtensa-esp-elf-gdb/17.1_20260402/xtensa-esp-elf-gdb/bin/xtensa-esp32s3-elf-gdb \
  -q .cache/firmware/i2c-contention/twatch_ultra_phase1.elf \
  -x tools/i2c_lock_timeout.gdb
```

Offline disassembly confirms failed take branches to function+168, address
0x4216d37c, instruction `movi a5,0x107`; it then returns ESP_ERR_TIMEOUT.
The nearby source line 1010 is unsuitable: its debug location points into the
success path. Do not reuse offsets after rebuilding. The script verifies the
host ELF, not the flashed image; verify deployment identity independently.
If no timeout occurs, interrupt GDB, remove breakpoints and detach explicitly,
then stop OpenOCD. If the script errors after attaching, likewise detach before
closing the server. Halting changes timing; task stacks may reveal contenders
but the binary semaphore does not record a mutex owner, so do not assume a
single snapshot proves who held it throughout the wait.

Prepared and disassembly-reviewed only; this script has not yet run on-device.

### Cleanup correction from live capture

The first live timeout trap captured main blocked behind an LVGL I2C event wait.
Disabling breakpoints then detaching was insufficient: the watch subsequently
remained halted at the diagnostic PC with no serial output. A fresh server had
no breakpoint bookkeeping but both cores were halted. Recovery, with both cores
halted, used `targets esp32s3.cpu0; reg ibreakenable 0`, then the same for cpu1,
then `targets esp32s3.cpu0; resume`. This intentionally clears all hardware
instruction-breakpoint enables; do this only when this diagnostic session owns
the debugger. Verify both targets running, stop OpenOCD, and confirm serial
health. No firmware reset or flash was needed for this recovery.

The script now deletes breakpoints, issues monitor resume, then detaches. It
also records 16 stack frames and bus/requester address/status. This revised
script is prepared, not yet live-validated. Treat missing serial output after
cleanup as a possible debugger halt before diagnosing a firmware crash.
