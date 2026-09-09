# LVGL PC simulator scaffold. Host-only, separate from the ESP-IDF build
# (build with plain cmake, not idf.py). Reuses the same LVGL checkout the
# firmware build pulls in via the component manager, so run `idf.py build`
# in the repo root at least once before building this if
# managed_components/lvgl__lvgl doesn't exist yet.
cmake_minimum_required(VERSION 3.12.4)
project(uwatch_sim C)

set(CMAKE_C_STANDARD 11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

set(LV_BUILD_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h CACHE PATH "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/lvgl__lvgl lvgl)

add_executable(uwatch_sim
    main.c
    nav.c
    navigation_test.c
    screenshot.c
    mock_hw.c
    ../main/screens/watch_face.c
    ../main/screens/nfc_screen.c
    ../main/screens/navigation.c
    ../main/screens/activity_screen.c
    ../main/screens/status_bar.c
    ../main/screens/status_icons.c
    ../main/screens/gps_screen.c
    ../main/screens/mesh_screen.c
    ../main/screens/wifi_screen.c
    ../main/screens/ble_screen.c
    ../main/screens/settings_screen.c
    ../main/screens/alarm_screen.c
    ../main/screens/ring_screen.c
    ../main/screens/bhi_screen.c
    ../components/ui_fonts/cascadia_88.c
    ../components/ui_fonts/cascadia_72.c
    ../components/ui_fonts/cascadia_36.c
    ../components/ui_fonts/cascadia_22.c
    ../components/ui_fonts/mesh_unicode_18.c
    ../components/ui_fonts/mesh_text_22.c
    ../components/ui_fonts/cascadia_18.c
)
target_link_libraries(uwatch_sim PRIVATE lvgl ${SDL2_LIBRARIES} m)
# compat listed BEFORE ../main so shared main/screens/*.c files' quote-includes
# of real driver header names (sensor_cache.h, alarm.h, ...) resolve to the
# sim's mocks (sim/compat/*.h -> mock_hw.h) instead of the real ESP-IDF headers.
target_include_directories(uwatch_sim PRIVATE ${SDL2_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/compat ${CMAKE_CURRENT_SOURCE_DIR}/../main ${CMAKE_CURRENT_SOURCE_DIR}/../components/ui_fonts/include ${CMAKE_CURRENT_SOURCE_DIR}/../components/drivers/m10q/include ${CMAKE_CURRENT_SOURCE_DIR}/../components/sd_log/include)
target_compile_options(uwatch_sim PRIVATE ${SDL2_CFLAGS_OTHER})
target_compile_definitions(uwatch_sim PRIVATE UWATCH_ASSETS_UI_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../assets/ui" UWATCH_SIM=1)

enable_testing()
add_test(NAME navigation COMMAND uwatch_sim)
set_tests_properties(navigation PROPERTIES ENVIRONMENT "UWATCH_SIM_TEST=1")

add_executable(test_gps_reception
    ../main/services/test/test_gps_reception.c
    ../main/services/gps_reception.c)
target_include_directories(test_gps_reception PRIVATE ../main/services ../components/drivers/m10q/include)
target_compile_options(test_gps_reception PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME gps_reception COMMAND test_gps_reception)

add_executable(test_m10q_decode
    ../components/drivers/m10q/test/test_decode.c
    ../components/drivers/m10q/m10q_decode.c)
target_include_directories(test_m10q_decode PRIVATE ../components/drivers/m10q/include)
target_compile_options(test_m10q_decode PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(test_m10q_decode PRIVATE m)
add_test(NAME m10q_decode COMMAND test_m10q_decode)

add_executable(test_activity_days
    ../components/drivers/bhi260ap/test/test_activity_days.c
    ../components/drivers/bhi260ap/activity_days.c
    ../main/services/daily_archive.c)
target_include_directories(test_activity_days PRIVATE ../components/drivers/bhi260ap/include ../main/services)
target_compile_options(test_activity_days PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME activity_days COMMAND test_activity_days)
