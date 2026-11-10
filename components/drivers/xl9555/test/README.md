# XL9555 pin mapping host tests

Tests pure pin-to-register/bit mapping and rejects invalid pins/output pointers.
No ESP-IDF or hardware required.

Run from the repository root:

```sh
gcc -Wall -Wextra -Werror -std=c11 \
  -I components/drivers/xl9555 \
  components/drivers/xl9555/test/test_xl9555_logic.c \
  components/drivers/xl9555/xl9555_logic.c \
  -o /tmp/test_xl9555_logic && /tmp/test_xl9555_logic
```
