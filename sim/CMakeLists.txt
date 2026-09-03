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
    screenshot.c
    mock_hw.c
    ../main/screens/watch_face.c
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
    ../components/ui_fonts/cascadia_18.c
)
target_link_libraries(uwatch_sim PRIVATE lvgl ${SDL2_LIBRARIES} m)
# compat listed BEFORE ../main so shared main/screens/*.c files' quote-includes
# of real driver header names (sensor_cache.h, alarm.h, ...) resolve to the
# sim's mocks (sim/compat/*.h -> mock_hw.h) instead of the real ESP-IDF headers.
target_include_directories(uwatch_sim PRIVATE ${SDL2_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/compat ${CMAKE_CURRENT_SOURCE_DIR}/../main ${CMAKE_CURRENT_SOURCE_DIR}/../components/ui_fonts/include)
target_compile_options(uwatch_sim PRIVATE ${SDL2_CFLAGS_OTHER})
target_compile_definitions(uwatch_sim PRIVATE UWATCH_ASSETS_UI_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../assets/ui" UWATCH_SIM=1)
