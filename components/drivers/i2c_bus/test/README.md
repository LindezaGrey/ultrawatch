# i2c_bus_frame tests

Host tests for `i2c_bus_frame.c`'s pure write-frame construction
(`i2c_bus_build_frame`). No ESP-IDF dependency - runs as a plain native
binary, no flashing needed. `i2c_bus_read`/`i2c_bus_write` themselves (the
actual I2C transactions) aren't tested here - only the byte-layout logic
they build on.

Plain C, no test framework: each check is a hand-rolled `assert`-style macro
that prints and counts failures; `main()` returns non-zero if any failed.

## Run

Fastest loop (no cmake configure step):

```sh
cd components/drivers/i2c_bus
gcc -Wall -Wextra -std=c11 -I include test/test_i2c_bus_frame.c i2c_bus_frame.c -o /tmp/test_i2c_bus_frame && /tmp/test_i2c_bus_frame
```

Or via CMake/CTest:

```sh
cd components/drivers/i2c_bus/test
cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
```
