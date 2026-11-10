# AXP2101 pure-logic host tests

Exercises voltage-code bounds, sparse charging-current encodings and PMIC die
ADC conversion without ESP-IDF or hardware.

Run from the driver directory:

```sh
gcc -Wall -Wextra -Werror -std=c11 -I . test/test_axp2101_logic.c axp2101_logic.c -o /tmp/test_axp2101_logic && /tmp/test_axp2101_logic
```
