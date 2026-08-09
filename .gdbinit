# Attach to OpenOCD (run on-chip USB-JTAG) started in the esp-idf container
target remote :3333
monitor reset halt
set remotetimeout 15
load
