"""Guard the observational probe against sending actuator/configuration commands."""
import importlib.util
from pathlib import Path
import struct

from pymavlink.dialects.v20 import common as mav


spec = importlib.util.spec_from_file_location(
    "px4_usb_probe", Path(__file__).resolve().parents[2] / "scripts/px4_usb_readonly_remote.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def command(number, first=0, second=0):
    return mav.MAVLink_command_long_message(1, 1, number, 0, first, second, 0, 0, 0, 0, 0)


def test_query_allowlist_only():
    for message_id in probe.REQUESTABLE_MESSAGES:
        assert probe.query_allowed(command(512, message_id))
    assert probe.query_allowed(command(520, 1))
    for name in probe.PARAMETERS:
        assert probe.query_allowed(mav.MAVLink_param_request_read_message(1, 1, name.encode(), -1))


def test_arm_reboot_mode_rate_and_arbitrary_queries_rejected():
    for number in (400, 246, 176, 183, 511, 21, 22):
        assert not probe.query_allowed(command(number, 1))
    assert not probe.query_allowed(command(512, 999))
    assert not probe.query_allowed(command(512, 148, 1))
    assert not probe.query_allowed(command(520, 0))
    assert not probe.query_allowed(mav.MAVLink_param_request_read_message(1, 1, b"UNLISTED", -1))
    assert not probe.query_allowed(mav.MAVLink_param_request_read_message(1, 1, b"SYS_HITL", 0))
    assert not probe.query_allowed(mav.MAVLink_param_set_message(1, 1, b"SYS_HITL", 1, 6))
    assert not probe.query_allowed(mav.MAVLink_set_mode_message(1, 1, 0))


def test_capability_aware_parameter_decoding():
    raw = struct.unpack("<f", struct.pack("<i", 4001))[0]
    message = mav.MAVLink_param_value_message(b"SYS_AUTOSTART", raw, 6, 1, 0)
    assert probe.parameter_value(message, mav.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)["value"] == 4001
    assert probe.parameter_value(message, None)["value"] is None
    cast = mav.MAVLink_param_value_message(b"SYS_AUTOSTART", 4001.0, 6, 1, 0)
    assert probe.parameter_value(cast, mav.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)["value"] == 4001


def test_generated_requests_roundtrip_crc():
    encoder = mav.MAVLink(None, srcSystem=245, srcComponent=191)
    decoder = mav.MAVLink(None)
    messages = [command(512, 148), command(520, 1),
                mav.MAVLink_param_request_read_message(1, 1, b"SYS_HITL", -1)]
    for message in messages:
        parsed = decoder.parse_char(message.pack(encoder))
        assert parsed is not None
        assert probe.query_allowed(parsed)
        assert (parsed.get_srcSystem(), parsed.get_srcComponent()) == (245, 191)
        assert probe.wire_identity(parsed) == ((245, 191), 0)


def test_unknown_message_identity_uses_wire_header():
    message = mav.MAVLink_unknown(436, bytearray([253, 4, 0, 0, 91, 1, 1, 180, 1, 0, 0, 0, 0, 0, 0, 0]))
    assert probe.wire_identity(message) == ((1, 1), 91)


def test_report_sanitizes_nonfinite_values_and_bytes():
    assert probe.safe_json({"x": float("nan"), "b": b"\0\1"}) == {"x": "nan", "b": "0001"}
