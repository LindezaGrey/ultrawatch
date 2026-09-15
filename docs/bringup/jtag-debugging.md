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
