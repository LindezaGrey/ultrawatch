/*
 * twatch_board.h - sim-only compat stub. main/screens/*.c include the real
 * driver headers by their real names so the same .c files compile
 * unchanged in the firmware build; this directory (sim/compat/, listed
 * ahead of main/ on the sim's include path - see sim/CMakeLists.txt)
 * intercepts those includes for the sim build and forwards to the mock
 * implementations in sim/mock_hw.h instead of the real ESP-IDF-dependent
 * header of the same name.
 */
#pragma once
#include "../mock_hw.h"
