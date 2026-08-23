#!/usr/bin/env python3
"""Host checks for the UltraWatch Wi-Fi profile and asset contract."""

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware"
WEB = ROOT / "web" / "index.html"
ATLAS = FIRMWARE / "sdcard" / "ultrawatch" / "ui" / "icons.rgb565"
CONFIG = FIRMWARE / "sdcard" / "ultrawatch" / "config.txt"

PACKET_SIZE = 20
DATA_SIZE = 10
APPEND_INDEX = 0xFFFFFFFF


def valid_password(password: str) -> bool:
    encoded = password.encode("utf-8")
    return (
        not encoded
        or 8 <= len(encoded) <= 63
        or (len(password) == 64 and all(char in "0123456789abcdefABCDEF"
                                         for char in password))
    )


def validate_config(value: object) -> None:
    if not isinstance(value, dict) or value.get("version") != 1:
        raise ValueError("version")
    networks = value.get("networks")
    if not isinstance(networks, list):
        raise ValueError("networks")
    seen: set[str] = set()
    for network in networks:
        if not isinstance(network, dict):
            raise ValueError("network")
        ssid = network.get("ssid")
        password = network.get("password")
        if not isinstance(ssid, str) or not isinstance(password, str):
            raise ValueError("types")
        if not 1 <= len(ssid.encode("utf-8")) <= 32:
            raise ValueError("ssid")
        if not valid_password(password):
            raise ValueError("password")
        if ssid in seen:
            raise ValueError("duplicate")
        seen.add(ssid)


def record(ssid: str, password: str) -> bytes:
    ssid_bytes = ssid.encode("utf-8")
    password_bytes = password.encode("utf-8")
    return bytes((len(ssid_bytes), len(password_bytes))) + ssid_bytes + password_bytes


def put_packets(request_id: int, index: int, payload: bytes) -> list[bytes]:
    packets = []
    for offset in range(0, len(payload), DATA_SIZE):
        data = payload[offset:offset + DATA_SIZE]
        packet = bytearray(PACKET_SIZE)
        packet[0] = 3
        packet[1] = request_id
        packet[2] = (1 if offset == 0 else 0) | (
            2 if offset + len(data) == len(payload) else 0)
        packet[3] = len(data)
        packet[4:8] = index.to_bytes(4, "little")
        packet[8:10] = offset.to_bytes(2, "little")
        packet[10:10 + len(data)] = data
        packets.append(bytes(packet))
    return packets


class WifiContractTests(unittest.TestCase):
    def test_committed_config_is_valid(self) -> None:
        value = json.loads(CONFIG.read_text(encoding="utf-8"))
        validate_config(value)

    def test_json_escaping_and_utf8_byte_limits(self) -> None:
        value = {
            "version": 1,
            "networks": [{"ssid": 'Lab "Nord"', "password": "größer-123"}],
        }
        validate_config(json.loads(json.dumps(value, ensure_ascii=False)))
        with self.assertRaises(ValueError):
            validate_config({"version": 1, "networks": [
                {"ssid": "ä" * 17, "password": "12345678"}]})

    def test_credentials_duplicates_and_version(self) -> None:
        invalid = [
            {"version": 2, "networks": []},
            {"version": 1, "networks": [{"ssid": "", "password": ""}]},
            {"version": 1, "networks": [{"ssid": "Lab", "password": "short"}]},
            {"version": 1, "networks": [
                {"ssid": "Lab", "password": "12345678"},
                {"ssid": "Lab", "password": "abcdefgh"},
            ]},
        ]
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_config(value)
        validate_config({"version": 1, "networks": [
            {"ssid": "Open", "password": ""},
            {"ssid": "PSK", "password": "a" * 64},
        ]})

    def test_order_and_full_length_transport(self) -> None:
        networks = ["first", "second", "third"]
        networks.insert(0, networks.pop(2))
        self.assertEqual(networks, ["third", "first", "second"])
        payload = record("S" * 32, "f" * 64)
        self.assertEqual(len(payload), 98)
        packets = put_packets(19, APPEND_INDEX, payload)
        self.assertTrue(all(len(packet) == PACKET_SIZE for packet in packets))
        self.assertEqual(b"".join(packet[10:10 + packet[3]] for packet in packets),
                         payload)
        self.assertEqual([int.from_bytes(packet[8:10], "little")
                          for packet in packets], list(range(0, 98, 10)))
        self.assertEqual(packets[0][2], 1)
        self.assertEqual(packets[-1][2], 2)

    def test_request_id_and_stale_offset_are_detectable(self) -> None:
        packets = put_packets(7, 2, record("Network", "password"))
        self.assertTrue(all(packet[1] == 7 for packet in packets))
        self.assertNotEqual(int.from_bytes(packets[-1][8:10], "little"), 3)
        self.assertEqual(int.from_bytes(packets[-1][4:8], "little"), 2)

    def test_web_uuids_queue_and_result_codes(self) -> None:
        html = WEB.read_text(encoding="utf-8")
        for suffix in ("000e", "000f"):
            self.assertIn(f"7a1e{suffix}-7a1e-4b6c-8d9e-001122334455", html)
        self.assertIn("queueGatt", html)
        self.assertIn("wifiProfilePending", html)
        for code in ("0x80", "0x81", "0x82", "0x83", "0x84"):
            self.assertIn(code, html)

    def test_storage_recovery_and_ram_only_wifi_are_present(self) -> None:
        manager = (FIRMWARE / "main" / "wifi_manager.c").read_text()
        for value in ("config.tmp", "config.bak", "fflush", "fsync", "rename"):
            self.assertIn(value, manager)
        self.assertIn("WIFI_STORAGE_RAM", manager)
        defaults = (FIRMWARE / "sdkconfig.defaults").read_text()
        self.assertIn("CONFIG_ESP_WIFI_NVS_ENABLED=n", defaults)
        log_lines = [line for line in manager.splitlines() if "ESP_LOG" in line]
        self.assertTrue(all("password" not in line.lower() for line in log_lines))

    def test_wifi_atlas(self) -> None:
        self.assertEqual(ATLAS.stat().st_size, 25 * 48 * 48 * 2)
        source = (FIRMWARE / "tools" / "build_ui_assets.py").read_text()
        order = [source.index(f'"wifi_{state}"') for state in
                 ("off", "connecting", "connected", "error")]
        self.assertEqual(order, sorted(order))


if __name__ == "__main__":
    unittest.main()
