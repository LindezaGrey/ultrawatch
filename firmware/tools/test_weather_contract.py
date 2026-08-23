#!/usr/bin/env python3
import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
WEATHER_C = (ROOT / "main/weather.c").read_text()
WEATHER_H = (ROOT / "main/weather.h").read_text()
APP_C = (ROOT / "main/app_main.c").read_text()
SDKCONFIG_DEFAULTS = (ROOT / "sdkconfig.defaults").read_text()
WEATHER_ATLAS = ROOT / "sdcard/ultrawatch/weather/images.rgb565"


class WeatherContractTest(unittest.TestCase):
    def test_sd_configuration_has_no_committed_key(self):
        config = json.loads((ROOT / "sdcard/ultrawatch/config.txt").read_text())
        self.assertEqual(config["openweathermap_api_key"], "")

    def test_daily_result_is_limited_to_seven_records(self):
        self.assertIn("#define WEATHER_DAY_COUNT 7", WEATHER_H)
        self.assertIn("snapshot->day_count == WEATHER_DAY_COUNT", WEATHER_C)

    def test_request_uses_one_call_4_metric_daily_endpoint(self):
        self.assertIn("/data/4.0/onecall/timeline/1day", WEATHER_C)
        self.assertIn("units=metric", WEATHER_C)

    def test_subscription_error_is_not_reported_as_network_failure(self):
        self.assertIn("WEATHER_API_ACCESS_ERROR", WEATHER_H)
        self.assertIn("http_status == 401 || http_status == 403", WEATHER_C)
        self.assertIn('"ONE CALL NICHT AKTIV"', APP_C)

    def test_cache_records_position_and_is_written_atomically(self):
        self.assertIn('"latitude_e7"', WEATHER_C)
        self.assertIn('"longitude_e7"', WEATHER_C)
        self.assertIn('WEATHER_CACHE_TMP_PATH', WEATHER_C)
        self.assertIn('rename(WEATHER_CACHE_TMP_PATH, WEATHER_CACHE_PATH)',
                      WEATHER_C)
        self.assertIn('WEATHER_CACHE_BACKUP_PATH', WEATHER_C)
        self.assertLess(
            WEATHER_C.index(
                'rename(WEATHER_CACHE_PATH, WEATHER_CACHE_BACKUP_PATH)'),
            WEATHER_C.index(
                'rename(WEATHER_CACHE_TMP_PATH, WEATHER_CACHE_PATH)'))

    def test_cache_is_published_first_and_refreshed_at_most_hourly(self):
        self.assertIn("#define WEATHER_CACHE_REFRESH_SECONDS 3600", WEATHER_C)
        self.assertIn('cJSON_AddNumberToObject(root, "updated_at"', WEATHER_C)
        self.assertIn("snapshot.state = WEATHER_READY_CACHE", WEATHER_C)
        self.assertIn("load_cache(&snapshot)", WEATHER_C)
        self.assertNotIn("load_matching_cache", WEATHER_C)
        self.assertLess(WEATHER_C.index("snapshot.state = WEATHER_READY_CACHE"),
                        WEATHER_C.index("fetch_weather(key"))
        self.assertIn("using fresh weather cache; network refresh deferred",
                      WEATHER_C)

    def test_mini_apps_keep_the_display_awake(self):
        self.assertIn("screen_keeps_display_awake", APP_C)
        for screen in ("UI_SETTINGS", "UI_ALARM", "UI_MAP", "UI_WEATHER"):
            self.assertIn(f"screen == {screen}", APP_C)

    def test_weather_launcher_is_interactive(self):
        self.assertIn(
            "{337, 241, 42, ICON_WEATHER, LAUNCHER_ACTION_WEATHER}", APP_C)
        self.assertIn("launcher_action_at(x, y)", APP_C)
        self.assertIn("bubble->radius", APP_C)
        self.assertIn("active_screen = UI_WEATHER", APP_C)

    def test_weather_starts_wifi_without_restarting_an_active_connection(self):
        self.assertIn("watch_wifi_ensure_enabled()", APP_C)
        self.assertIn("wifi.state == WATCH_WIFI_OFF", WEATHER_C)
        self.assertIn("watch_wifi_wait_for_status_change()", WEATHER_C)

    def test_tls_uses_psram_but_weather_task_stack_stays_internal(self):
        self.assertIn('xTaskCreate(weather_task, "weather"', WEATHER_C)
        self.assertNotIn("xTaskCreateWithCaps(weather_task", WEATHER_C)
        self.assertIn("CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y",
                      SDKCONFIG_DEFAULTS)
        self.assertIn("# CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC is not set",
                      SDKCONFIG_DEFAULTS)

    def test_json_trees_use_psram_before_any_application_service_starts(self):
        allocator = APP_C.index("initialize_cjson_allocator();")
        rtc = APP_C.index("ble_rtc_initialize()")
        self.assertLess(allocator, rtc)
        self.assertIn("cJSON_InitHooks(&hooks)", APP_C)
        self.assertIn("MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT", APP_C)

    def test_weather_keeps_the_phy_awake_until_transient_data_is_released(self):
        manager = (ROOT / "main/wifi_manager.c").read_text()
        self.assertIn("watch_wifi_set_transfer_active(true)", WEATHER_C)
        self.assertIn("watch_wifi_set_transfer_active(false)", WEATHER_C)
        self.assertIn("WIFI_PS_NONE", manager)
        self.assertIn("WIFI_PS_MIN_MODEM", manager)
        success = WEATHER_C.index(
            "if (response != NULL && parse_days(response, &online))")
        failure = WEATHER_C.index("} else {", success)
        success_path = WEATHER_C[success:failure]
        self.assertLess(success_path.index("cJSON_Delete(response)"),
                        success_path.index(
                            "watch_wifi_set_transfer_active(false)"))

    def test_online_response_is_released_before_display_refresh(self):
        success = WEATHER_C.index(
            "if (response != NULL && parse_days(response, &online))")
        failure = WEATHER_C.index("} else {", success)
        success_path = WEATHER_C[success:failure]
        self.assertLess(success_path.index("cJSON_Delete(response)"),
                        success_path.index("publish(&online)"))
        self.assertIn("heap before weather display refresh", success_path)

    def test_display_queue_memory_failure_does_not_abort(self):
        self.assertNotIn(
            "ESP_ERROR_CHECK(spi_device_queue_trans(display_spi",
            APP_C)
        self.assertIn("display command queue failed", APP_C)
        self.assertIn("display update stopped at row", APP_C)

    def test_settings_has_wifi_control(self):
        self.assertIn("draw_settings_wifi(settings_frame)", APP_C)
        self.assertIn("settings_wifi_dirty = true", APP_C)

    def test_weather_picture_atlas_and_condition_mapping(self):
        self.assertEqual(WEATHER_ATLAS.stat().st_size, 9 * 128 * 128 * 2)
        self.assertIn('#define WEATHER_PICTURE_COUNT 9', APP_C)
        self.assertIn('draw_weather_picture(frame, day->condition_id)', APP_C)
        expected = {
            'condition_id >= 200 && condition_id < 300': 'return 6',
            'condition_id >= 300 && condition_id < 400': 'return 4',
            'condition_id == 511': 'return 7',
            'condition_id >= 500 && condition_id <= 504': 'return 5',
            'condition_id >= 520 && condition_id < 600': 'return 4',
            'condition_id >= 600 && condition_id < 700': 'return 7',
            'condition_id >= 700 && condition_id < 800': 'return 8',
            'condition_id == 800': 'return 0',
            'condition_id == 801': 'return 1',
            'condition_id == 802': 'return 2',
            'condition_id == 803 || condition_id == 804': 'return 3',
        }
        for condition, result in expected.items():
            self.assertIn(f'if ({condition}) {result};', APP_C)

    def test_temperature_uses_single_byte_degree_glyph(self):
        font_header = (ROOT / "main/cascadia_code_72.h").read_text()
        font_tool = (ROOT / "tools/generate_cascadia_font.py").read_text()
        self.assertIn('"%d\\260"', APP_C)
        self.assertIn('"MIN %d\\260  MAX %d\\260"', APP_C)
        self.assertNotIn('"%dC"', APP_C)
        self.assertIn("?°", font_tool)
        self.assertIn("\\260", font_header)
        self.assertIn("static uint8_t glyph_indices[256]", APP_C)


if __name__ == "__main__":
    unittest.main()
