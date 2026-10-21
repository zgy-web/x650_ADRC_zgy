from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("x650_parameter_evidence", ROOT / "simulations/x650_hover_v1/parameter_evidence.py")
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


class Message:
    def __init__(self, name, value, kind=6, source=(1, 1), raw=None, version=2, signed=False):
        self.param_id = name
        self.param_type = kind
        self.source = source
        value_bytes = raw if raw is not None else struct.pack("<i" if kind == 6 else "<f", value)
        self.payload = value_bytes + struct.pack("<HH", 101, 0) + name.encode("ascii").ljust(16, b"\0") + bytes([kind])
        if version == 2:
            header = bytes([0xFD, len(self.payload), int(signed), 0, 17, *source, 22, 0, 0])
        else:
            header = bytes([0xFE, len(self.payload), 17, *source, 22])
        checksum = 0xFFFF
        for byte in header[1:] + self.payload + bytes([220]):  # PARAM_VALUE CRC extra.
            tmp = byte ^ (checksum & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            checksum = ((checksum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        self.frame = header + self.payload + struct.pack("<H", checksum) + (bytes(13) if signed else b"")

    def get_type(self): return "PARAM_VALUE"
    def get_srcSystem(self): return self.source[0]
    def get_srcComponent(self): return self.source[1]
    def get_payload(self): return self.frame[6:-2]  # Reproduce deployed MAVLink 2 bug.
    def get_msgbuf(self): return self.frame


class Vehicle:
    def __init__(self, values):
        self.target = (1, 1)
        self.values = values
        self.sent = []
        self.pending = []
        self.pump_count = 0
        self.other_hook = lambda link, message: None
        self.link = SimpleNamespace(message_hooks=[self.other_hook], mav=SimpleNamespace(param_request_read_send=self.send))

    def send(self, system, component, name, index):
        name = name.decode("ascii")
        self.sent.append((system, component, name, index))
        if name in self.values:
            value, kind = self.values[name]
            self.pending.append(Message(name, value, kind, source=(2, 1)))
            self.pending.append(Message(name, value, kind))

    def wait(self, predicate, timeout_s, description):
        self.pump_count += 1
        for message in self.pending:
            for hook in list(self.link.message_hooks):
                hook(self.link, message)
        self.pending.clear()
        if not predicate():
            raise RuntimeError("Timeout: " + description)


class ParameterEvidenceTests(unittest.TestCase):
    def test_integer_bytes_are_not_float_cast_and_nan_bits_are_preserved(self):
        for version in (1, 2):
            for value in (4999, -1, -8388607, -2147483648, 2147483647):
                message = Message("SYS_AUTOSTART", value, version=version)
                decoded = MOD.decode_param_value(message)
                self.assertEqual(decoded["value"], value)
                self.assertEqual(decoded["raw_value_hex"], struct.pack("<i", value).hex())

    def test_real32_and_signed_mavlink2_header(self):
        for version in (1, 2):
            self.assertAlmostEqual(MOD.decode_param_value(Message("MPC_THR_HOVER", .3888, 9, version=version))["value"], .3888, places=6)
        self.assertEqual(MOD.decode_param_value(Message("SYS_AUTOSTART", 4999, signed=True))["value"], 4999)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            MOD.decode_param_value(Message("MPC_THR_HOVER", float("nan"), 9))

    def test_mavlink2_regression_header_bytes_are_not_parameter_value(self):
        message = Message("SYS_AUTOSTART", 4999)
        self.assertEqual(message.get_payload()[:4].hex(), "01160000")
        self.assertEqual(MOD.decode_param_value(message)["value"], 4999)

    def test_wrong_message_id_and_truncated_frame_rejected(self):
        message = Message("SYS_AUTOSTART", 4999)
        message.frame = message.frame[:7] + bytes([23]) + message.frame[8:]
        with self.assertRaisesRegex(ValueError, "wire message ID 22"):
            MOD.decode_param_value(message)
        message = Message("SYS_AUTOSTART", 4999)
        message.frame = message.frame[:-1]
        with self.assertRaisesRegex(ValueError, "frame length"):
            MOD.decode_param_value(message)

    def test_native_pymavlink_pack_parse_frames_when_installed(self):
        try:
            from pymavlink.dialects.v10 import common as mav1
            from pymavlink.dialects.v20 import common as mav2
        except ImportError:
            self.skipTest("pymavlink not installed locally; run this test in server amsim-venv")
        for version, dialect in ((1, mav1), (2, mav2)):
            parser = dialect.MAVLink(None)
            for value in (4999, -1, -8388607, -2147483648, 2147483647):
                # Feed actual CRC-valid wire frames, including integer bit patterns
                # that Python float unpack/repack would canonicalize as NaN.
                wire = Message("SYS_AUTOSTART", value, version=version).frame
                parsed = parser.parse_char(wire)
                self.assertIsNotNone(parsed)
                self.assertEqual(MOD.decode_param_value(parsed)["value"], value)
            sender = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
            packet = sender.param_value_encode(b"MPC_THR_HOVER", .3888, 9, 101, 0).pack(sender)
            parsed = parser.parse_char(packet)
            self.assertAlmostEqual(MOD.decode_param_value(parsed)["value"], .3888, places=6)

    def test_hook_reuses_normal_pump_filters_source_and_cleans_up(self):
        vehicle = Vehicle({"SYS_AUTOSTART": (4999, 6), "MPC_USE_HTE": (1, 6)})
        values, errors, attempts = MOD.read_parameters(vehicle, ["SYS_AUTOSTART", "MPC_USE_HTE"])
        self.assertEqual(values["SYS_AUTOSTART"]["value"], 4999)
        self.assertEqual(values["MPC_USE_HTE"]["source_system"], 1)
        self.assertFalse(errors)
        self.assertEqual(attempts, {"SYS_AUTOSTART": 1, "MPC_USE_HTE": 1})
        self.assertEqual(vehicle.pump_count, 2)
        self.assertEqual(vehicle.link.message_hooks, [vehicle.other_hook])
        self.assertTrue(all(item[-1] == -1 for item in vehicle.sent))

    def test_missing_response_retries_and_keeps_original_hooks(self):
        vehicle = Vehicle({})
        values, errors, attempts = MOD.read_parameters(vehicle, ["MPC_USE_HTE"], attempts=2)
        self.assertFalse(values)
        self.assertIn("No fresh PARAM_VALUE", errors["MPC_USE_HTE"])
        self.assertEqual(attempts["MPC_USE_HTE"], 2)
        self.assertEqual(vehicle.link.message_hooks, [vehicle.other_hook])

    def test_airframe_requires_literal_names_and_matches_float32_tolerance(self):
        expected = MOD.parse_airframe("param set CA_ROTOR0_PX 0.2282554293\nparam set SIM_GZ_EC_MAX1 628\n")
        values = {"CA_ROTOR0_PX": MOD.decode_param_value(Message("CA_ROTOR0_PX", .2282554293, 9)), "SIM_GZ_EC_MAX1": MOD.decode_param_value(Message("SIM_GZ_EC_MAX1", 628))}
        self.assertFalse(MOD.contract_mismatches(values, expected))
        values["SIM_GZ_EC_MAX1"]["value"] = 629
        self.assertIn("SIM_GZ_EC_MAX1", MOD.contract_mismatches(values, expected))
        with self.assertRaisesRegex(ValueError, "nonnumeric"):
            MOD.parse_airframe("param set SIM_GZ_EC_MAX1 ${VALUE}")

    def test_failed_contract_is_saved_before_arming_can_proceed(self):
        with tempfile.TemporaryDirectory(prefix="x650-params-") as directory:
            folder = Path(directory)
            source = folder / "airframe_source"
            source.write_bytes(b"param set SIM_GZ_EC_MAX1 628\n")
            evidence = MOD.ParameterEvidence(folder, source, source, 4999, "a"*64)
            vehicle = Vehicle({"SIM_GZ_EC_MAX1": (900, 6), "SYS_AUTOSTART": (4999, 6), "MPC_USE_HTE": (1, 6)})
            with self.assertRaisesRegex(RuntimeError, "arming refused"):
                evidence.capture(vehicle, "before", enforce_contract=True)
            data = json.loads((folder / "px4_parameters_before.json").read_text())
            self.assertEqual(data["contract_mismatches"]["SIM_GZ_EC_MAX1"]["actual"], 900)
            self.assertTrue(evidence.reports["before"]["sha256"])
            self.assertEqual(vehicle.link.message_hooks, [vehicle.other_hook])

    def test_after_records_estimator_related_change_without_claiming_fixed_values(self):
        with tempfile.TemporaryDirectory(prefix="x650-params-") as directory:
            folder = Path(directory)
            source = folder / "airframe_source"
            source.write_bytes(b"param set MPC_THR_HOVER 0.3888\n")
            evidence = MOD.ParameterEvidence(folder, source, source, 4999, "a"*64)
            vehicle = Vehicle({"MPC_THR_HOVER": (.3888, 9), "SYS_AUTOSTART": (4999, 6), "MPC_USE_HTE": (1, 6)})
            evidence.capture(vehicle, "before", enforce_contract=True)
            vehicle.values["MPC_THR_HOVER"] = (.40, 9)
            report = evidence.capture(vehicle, "after", enforce_contract=False)
            self.assertFalse(report["parameters_unchanged"])
            self.assertEqual(report["changed_names"], ["MPC_THR_HOVER"])
            self.assertEqual(evidence.snapshots["before"]["parameters"]["MPC_USE_HTE"]["value"], 1)

    def test_all_requested_identifiers_fit_protocol_and_flight_contract_is_parsed(self):
        contract = MOD.parse_airframe((ROOT / "configs/vehicles/x650_no_arm/4999_gz_x650_mn4014_v1").read_text())
        self.assertGreater(len(contract), 40)
        self.assertTrue(all(len(name) <= 16 for name in (*contract, *MOD.CORE_CONTROL_PARAMETERS)))


if __name__ == "__main__":
    unittest.main()
