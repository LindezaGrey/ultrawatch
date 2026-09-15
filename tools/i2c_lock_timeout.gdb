# Only for flashed d08530f and its preserved ELF. Start OpenOCD separately,
# with serial monitor stopped. Loading this script controls the live target.
set pagination off
set confirm off
python
import hashlib
import gdb
expected = 'e3a391537b870134574b574bf329ad293d80265fad1da35362962e531f386e83'
filename = gdb.current_progspace().filename
if not filename:
    raise gdb.GdbError('Load the preserved i2c-contention ELF first')
with open(filename, 'rb') as source:
    digest = hashlib.file_digest(source, 'sha256').hexdigest()
if digest != expected:
    raise gdb.GdbError('ELF mismatch: do not use this breakpoint with another build')
end
set logging file /tmp/ultra-i2c-lock-gdb.log
set logging overwrite on
set logging enabled on
target extended-remote :3333
# Audited failure branch: movi a5,0x107 after failed xQueueSemaphoreTake.
# Source line 1010 maps incorrectly to success-path code under optimization.
hbreak *s_i2c_synchronous_transaction+168
commands
silent
printf "I2C_LOCK_TIMEOUT breakpoint: semaphore acquisition failed\n"
info registers pc
bt 12
printf "Failed caller address and active bus transfer address/status:\n"
p/x i2c_dev->device_address
p/x i2c_dev->master_bus->i2c_trans.device_address
p i2c_dev->master_bus->status
info threads
thread apply all bt 16
# Delete (not merely disable) debugger breakpoints and explicitly resume.
# Still verify running state/serial after detach; cleanup failures are visible.
delete breakpoints
monitor resume
detach
set logging enabled off
quit
end
continue
