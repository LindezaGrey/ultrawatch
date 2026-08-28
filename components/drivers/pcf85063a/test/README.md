# pcf85063a_time tests

Host tests for `pcf85063a_time.c`'s UTC calendar<->epoch conversion
(`pcf85063a_time_to_epoch`/`pcf85063a_epoch_to_time`). No ESP-IDF dependency -
runs as a plain native binary, no flashing needed.

Plain C, no test framework: each check is a hand-rolled `assert`-style macro
that prints and counts failures; `main()` returns non-zero if any failed.

## Run

Fastest loop (no cmake configure step):

```sh
cd components/drivers/pcf85063a
gcc -Wall -Wextra -std=c11 -I include test/test_pcf85063a_time.c pcf85063a_time.c -o /tmp/test_pcf85063a_time && /tmp/test_pcf85063a_time
```

Or via CMake/CTest:

```sh
cd components/drivers/pcf85063a/test
cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
```
