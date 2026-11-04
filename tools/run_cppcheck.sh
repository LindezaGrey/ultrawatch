#!/usr/bin/env bash
set -euo pipefail

common=(
    --enable=warning,performance,portability
    --error-exitcode=1
    --inline-suppr
    --language=c
    --std=c11
    --suppress=missingIncludeSystem
    --suppress=syntaxError:*/bhy2_defs.h
    --quiet
)

mapfile -t firmware_sources < <(
    # cppcheck 2.13.0 (Ubuntu Noble) crashes inside ValueFlow::setValues()
    # when analyzing this translation unit, even with default options. Keep
    # it covered by the ESP-IDF compiler; remove this exclusion when cppcheck
    # is upgraded and the translation unit passes independently.
    find main components -type f -name '*.c' \
        ! -path 'components/ubxlib/*' \
        ! -path 'components/ui_fonts/*' \
        ! -path 'components/drivers/bhi260ap/bhy2*' \
        ! -path 'components/drivers/bhi260ap/bhi260ap.c' \
        | sort
)

firmware_includes=(
    -Imain
    -Imain/connectivity
    -Imain/controllers
    -Imain/debug
    -Imain/screens
    -Imain/services
    -Imain/system
)
for include_dir in components/*/include components/drivers/*/include; do
    firmware_includes+=("-I${include_dir}")
done

cppcheck "${common[@]}" "${firmware_includes[@]}" "${firmware_sources[@]}"

mapfile -t simulator_sources < <(find sim -maxdepth 1 -type f -name '*.c' | sort)
cppcheck "${common[@]}" -Isim/compat -Isim -Imain/screens "${simulator_sources[@]}"
