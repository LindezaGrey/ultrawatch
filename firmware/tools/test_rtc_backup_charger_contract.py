#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
BLE_C = (ROOT / "main/ble_rtc.c").read_text()
WEB = (ROOT.parent / "web/index.html").read_text()
README = (ROOT.parent / "README.md").read_text()


class RtcBackupChargerContractTest(unittest.TestCase):
    def test_axp2101_uses_button_battery_charger_at_3_1_volts(self):
        self.assertIn(
            "#define AXP_BACKUP_CHARGE_VOLTAGE_REGISTER 0x6a", BLE_C)
        self.assertIn("#define AXP_BACKUP_CHARGE_ENABLE_BIT 2", BLE_C)
        self.assertIn("#define AXP_BACKUP_CHARGE_VOLTAGE_MV 3100", BLE_C)
        self.assertIn("#define AXP_BACKUP_CHARGE_VOLTAGE_CODE 5", BLE_C)
        self.assertIn("axp_set_backup_charger(true)", BLE_C)

    def test_setting_is_written_and_read_back(self):
        function = BLE_C[
            BLE_C.index("static esp_err_t axp_set_backup_charger"):
            BLE_C.index("static esp_err_t axp_initialize_measurement")
        ]
        self.assertIn("axp_write_masked_register", function)
        self.assertGreaterEqual(function.count("axp_read_registers"), 2)
        self.assertIn("ESP_ERR_INVALID_RESPONSE", function)

    def test_ble_payload_reports_and_accepts_backup_setting(self):
        self.assertIn('"%u,%u,%u,%u,%u"', BLE_C)
        self.assertIn('"%u,%u,%u,%u%c"', BLE_C)
        self.assertIn("backup_setting_present", BLE_C)
        self.assertIn("axp_set_backup_charger(backup_charger_enabled)", BLE_C)

    def test_web_has_a_separate_rtc_backup_toggle(self):
        self.assertIn('id="rtcBackupChargerEnabled"', WEB)
        self.assertIn("100 µA · 3.1 V · MS621FE", WEB)
        self.assertIn("values.length !== 5", WEB)
        self.assertIn("ui.rtcBackupChargerEnabled.checked", WEB)

    def test_web_cannot_apply_unread_charge_defaults(self):
        self.assertIn("let powerConfigLoaded = false", WEB)
        self.assertIn("Read from watch first", WEB)
        self.assertIn("ui.writePowerConfig.disabled = !editable", WEB)
        self.assertIn("if (!powerConfigLoaded)", WEB)
        self.assertIn("powerConfigLoaded = true", WEB)

    def test_documentation_states_restart_behavior(self):
        self.assertIn("enables this charger after each restart", README)
        self.assertIn("three-field write leaves the RTC backup", README)


if __name__ == "__main__":
    unittest.main()
