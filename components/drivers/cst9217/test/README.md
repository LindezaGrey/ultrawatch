# CST9217 touch wrapper logic host tests

Tests the project-owned raw-coordinate bounds helper. These tests do not
exercise the vendored I2C controller driver or hardware.

Run from the repository root:

```sh
gcc -Wall -Wextra -Werror -std=c11 \
  -I components/drivers/cst9217 \
  components/drivers/cst9217/test/test_cst9217_logic.c \
  components/drivers/cst9217/cst9217_logic.c \
  -o /tmp/test_cst9217_logic && /tmp/test_cst9217_logic
```
