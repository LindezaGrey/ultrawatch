idf_component_register(
    SRCS "uwatch_main.c" "debug/debug_audio.c" "debug/debug_bhi.c" "debug/debug_gnss.c" "debug/debug_cmds.c" "display_controller.c" "touch_controller.c" "lvgl_app.c" "system/power_mgmt.c" "system/crash_dump.c" "services/tracking.c" "services/sensor_cache.c" "connectivity/ble_debug.c" "alarm.c" "services/cd_timer.c" "services/daily_log.c" "services/gpx_log.c" "connectivity/mesh_log.c" "connectivity/wifi_scan.c" "haptic.c" "system/syslog_capture.c" "services/housekeeping.c" "connectivity/ble_scan.c" "screens/status_bar.c" "screens/status_icons.c" "screens/watch_face.c" "screens/activity_screen.c" "screens/gps_screen.c" "screens/mesh_screen.c" "screens/wifi_screen.c" "screens/ble_screen.c" "screens/settings_screen.c" "screens/alarm_screen.c" "screens/ring_screen.c" "screens/bhi_screen.c"
    INCLUDE_DIRS "." "connectivity" "debug" "services" "system"
    REQUIRES
        twatch_board
        axp2101
        xl9555
        cst9217
        co5300
        bhi260ap
        m10q
        drv2605
        max98357a
        t3902
        sd_log
        pcf85063a
        sx1262
        st25r3916
        spi2_power
        ndef
        esp_lvgl_adapter
        lvgl
        ui_fonts
        spiffs
        esp_app_format
        esp_pm
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_usb_serial_jtag
        esp_common
        esp_timer
        log
        nvs_flash
        bt
        espcoredump
        esp_partition
        esp_wifi
        esp_netif
        esp_event
)

# Bake the git commit hash into the firmware for boot-screen identification.
# `-c safe.directory=*` keeps this working when the build container runs as
# root against a host-owned repo. Version is read at runtime from the app
# descriptor (esp_app_get_description), which ESP-IDF resolves correctly.
execute_process(
    COMMAND git -c safe.directory=* rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE UPROJECT_GIT_HASH
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT UPROJECT_GIT_HASH)
    set(UPROJECT_GIT_HASH "unknown")
endif()
target_compile_definitions(${COMPONENT_LIB} PUBLIC "UWATCH_GIT_HASH=\"${UPROJECT_GIT_HASH}\"")

# Pack the assets/ folder (UI overlay, BHI firmware) into the SPIFFS partition.
spiffs_create_partition_image(assets ../assets FLASH_IN_PROJECT)
