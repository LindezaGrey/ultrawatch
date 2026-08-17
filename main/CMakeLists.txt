idf_component_register(
    SRCS "uwatch_main.c" "lvgl_app.c" "power_mgmt.c" "crash_dump.c" "tracking.c" "sensor_cache.c" "ble_debug.c" "alarm.c" "cascadia_72.c" "cascadia_36.c" "cascadia_22.c" "cascadia_18.c"
    INCLUDE_DIRS "."
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
        esp_lvgl_adapter
        lvgl
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
