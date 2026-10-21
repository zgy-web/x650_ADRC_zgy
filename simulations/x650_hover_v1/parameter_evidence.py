"""Read PX4 parameter evidence through the already-owned MAVLink connection.

This module never writes PX4 parameters and never opens another connection.
PX4's pinned parameter server byte-copies INT32 values into PARAM_VALUE's
float-sized wire slot. Decode its raw bytes, not the apparent Python float.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import struct
import time


CORE_CONTROL_PARAMETERS = (
    "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P", "MC_AIRMODE",
    "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_FF", "MC_ROLLRATE_K",
    "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_FF", "MC_PITCHRATE_K",
    "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D", "MC_YAWRATE_FF", "MC_YAWRATE_K",
    "MPC_XY_P", "MPC_Z_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC",
    "MPC_Z_VEL_P_ACC", "MPC_Z_VEL_I_ACC", "MPC_Z_VEL_D_ACC",
    "MPC_THR_MIN", "MPC_THR_MAX", "MPC_THR_HOVER", "MPC_USE_HTE",
    "MPC_TILTMAX_AIR", "MPC_TILTMAX_LND", "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN",
    "MPC_ACC_HOR_MAX", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX", "MPC_JERK_MAX", "MPC_TKO_SPEED", "MPC_LAND_SPEED",
)
TYPE_NAMES = {6: "INT32", 9: "REAL32"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_airframe(text):
    expected = {}
    for number, line in enumerate(text.splitlines(), start=1):
        tokens = shlex.split(line, comments=True)
        if tokens[:2] != ["param", "set"]:
            continue
        if len(tokens) != 4 or not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", tokens[2]):
            raise ValueError(f"Airframe line {number} is not a literal parameter assignment")
        try:
            value = float(tokens[3])
        except ValueError as exc:
            raise ValueError(f"Airframe line {number} has a nonnumeric parameter value") from exc
        if not math.isfinite(value):
            raise ValueError(f"Airframe line {number} has a nonfinite parameter value")
        if tokens[2] in expected:
            raise ValueError(f"Airframe repeats parameter {tokens[2]}")
        expected[tokens[2]] = value
    if not expected:
        raise ValueError("Airframe has no literal 'param set' assignments to verify")
    return expected


def decode_param_value(message):
    """Extract wire bytes from a complete MAVLink 1/2 frame, avoiding NaN loss.

    Do not use get_payload(): the deployed pymavlink version slices that helper
    at the MAVLink 1 header length even for MAVLink 2, exposing header bytes.
    CRC/signature verification remains pymavlink's parser's responsibility.
    """
    if message.get_type() != "PARAM_VALUE":
        raise ValueError("Expected PARAM_VALUE")
    frame = bytes(message.get_msgbuf())
    if len(frame) < 8:
        raise ValueError("PARAM_VALUE complete MAVLink frame is missing")
    if frame[0] == 0xFD:
        header_size = 10
        if len(frame) < header_size:
            raise ValueError("Truncated MAVLink 2 header")
        message_id = int.from_bytes(frame[7:10], "little")
        signature_size = 13 if frame[2] & 1 else 0
    elif frame[0] == 0xFE:
        header_size, message_id, signature_size = 6, frame[5], 0
    else:
        raise ValueError("Unsupported MAVLink frame magic")
    if message_id != 22:
        raise ValueError(f"Expected wire message ID 22 (PARAM_VALUE), got {message_id}")
    payload_size = frame[1]
    if len(frame) != header_size + payload_size + 2 + signature_size:
        raise ValueError("Truncated or inconsistent PARAM_VALUE frame length")
    if not 1 <= payload_size <= 25:
        raise ValueError("Invalid PARAM_VALUE payload length")
    payload = frame[header_size:header_size+payload_size].ljust(25, b"\0")
    if payload[24] != int(message.param_type):
        raise ValueError("PARAM_VALUE wire type disagrees with parsed type")
    # MAVLink 2 may truncate trailing zero bytes; reconstruct those wire zeros.
    # For ordinary PX4 PARAM_VALUE, nonzero param_type at offset 24 normally
    # means all four leading value bytes are already present.
    raw = bytes(payload[:4]).ljust(4, b"\0")
    kind = payload[24]
    if kind == 6:  # MAV_PARAM_TYPE_INT32
        value = struct.unpack("<i", raw)[0]
    elif kind == 9:  # MAV_PARAM_TYPE_REAL32
        value = struct.unpack("<f", raw)[0]
        if not math.isfinite(value):
            raise ValueError("PX4 returned nonfinite REAL32 parameter")
    else:
        raise ValueError(f"Unsupported PX4 parameter type {kind}; expected INT32 or REAL32")
    return {"value": value, "param_type": kind, "type_name": TYPE_NAMES[kind], "raw_value_hex": raw.hex()}


def parameter_name(message):
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii")
    name = str(value).split("\0", 1)[0]
    if not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", name):
        raise ValueError(f"Invalid parameter name {name!r}")
    return name


def read_parameters(vehicle, names, *, timeout_s=1.25, attempts=3):
    """Temporarily observe messages while VehicleAPI keeps its normal pump alive.

    pymavlink calls message_hooks(link, message) from post_message(), before
    recv_match() returns to VehicleAPI. Its existing pump can therefore keep
    handling ROS, GCS heartbeats and ACKs without discarding the evidence.
    """
    link = vehicle.link
    target = vehicle.target
    if target is None or not isinstance(getattr(link, "message_hooks", None), list):
        raise RuntimeError("Existing PX4 connection lacks target identity or pymavlink message_hooks")
    requested = list(dict.fromkeys(names))
    received, errors, retry_counts = {}, {}, {}
    waiting_for = [None]

    def hook(_link, message):
        if message.get_type() != "PARAM_VALUE" or (message.get_srcSystem(), message.get_srcComponent()) != tuple(target):
            return
        try:
            name = parameter_name(message)
        except (ValueError, UnicodeError):
            return  # Unrelated malformed parameter cannot satisfy this request.
        if name != waiting_for[0]:
            return
        try:
            received[name] = {**decode_param_value(message), "received_wall_time": time.time(),
                              "source_system": int(message.get_srcSystem()), "source_component": int(message.get_srcComponent())}
        except (ValueError, TypeError, struct.error) as exc:
            errors[name] = str(exc)

    link.message_hooks.append(hook)
    try:
        for name in requested:
            if not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", name):
                raise ValueError(f"Invalid requested parameter name {name!r}")
            waiting_for[0] = name
            for attempt in range(1, attempts+1):
                retry_counts[name] = attempt
                link.mav.param_request_read_send(*target, name.encode("ascii"), -1)
                try:
                    vehicle.wait(lambda: name in received or name in errors, timeout_s, "parameter " + name)
                except RuntimeError as exc:
                    if not str(exc).startswith("Timeout:"):
                        raise
                if name in received or name in errors:
                    break
            if name not in received and name not in errors:
                errors[name] = f"No fresh PARAM_VALUE after {attempts} read requests"
        return received, errors, retry_counts
    finally:
        # Remove our own hook only; preserve all pre-existing consumers.
        if hook in link.message_hooks:
            link.message_hooks.remove(hook)


def contract_mismatches(parameters, expected):
    differences = {}
    for name, reference in expected.items():
        actual = parameters.get(name)
        if actual is None:
            differences[name] = {"expected": reference, "actual": None, "reason": "missing_readback"}
            continue
        value = actual["value"]
        matches = (value == reference) if actual["param_type"] == 6 else math.isclose(value, reference, rel_tol=2e-6, abs_tol=1e-7)
        if not matches:
            differences[name] = {"expected": reference, "actual": value, "reason": "value_mismatch"}
    return differences


class ParameterEvidence:
    def __init__(self, folder, installed_airframe, reviewed_airframe, autostart, model_sha256):
        self.folder = Path(folder)
        self.airframe = Path(installed_airframe)
        installed = self.airframe.read_bytes()
        reference = Path(reviewed_airframe).read_text(encoding="utf-8").replace("\r\n", "\n").encode("utf-8")
        if installed != reference:
            raise RuntimeError("Installed PX4 airframe differs from the reviewed X650 airframe")
        self.airframe_sha256 = hashlib.sha256(installed).hexdigest()
        (self.folder / "px4_airframe.sh").write_bytes(installed)
        capture_source = Path(__file__).read_bytes()
        self.capture_source_sha256 = hashlib.sha256(capture_source).hexdigest()
        (self.folder / "parameter_evidence.py").write_bytes(capture_source)
        self.expected = parse_airframe(installed.decode("utf-8"))
        self.expected["SYS_AUTOSTART"] = int(autostart)
        self.required = set(self.expected) | {"MPC_USE_HTE"}
        self.requested = list(dict.fromkeys([*self.expected, *CORE_CONTROL_PARAMETERS]))
        self.model_sha256 = model_sha256
        self.snapshots = {}
        self.reports = {}

    def capture(self, vehicle, label, *, enforce_contract):
        if label not in {"before", "after"}:
            raise ValueError("Parameter snapshot label must be before or after")
        snapshot = {
            "schema_version": 1, "phase": label, "model_sha256": self.model_sha256,
            "airframe_source": str(self.airframe), "airframe_sha256": self.airframe_sha256,
            "capture_source_sha256": self.capture_source_sha256,
            "encoding": "PX4 INT32 bytewise / REAL32, decoded from original PARAM_VALUE payload bytes",
            "method": "PARAM_REQUEST_READ on existing VehicleAPI.link; temporary pymavlink message_hook; no parameter writes",
            "requested_names": self.requested, "required_names": sorted(self.required),
            "started_wall_time": time.time(), "parameters": {}, "read_errors": {},
        }
        failure = None
        try:
            if sha(self.airframe) != self.airframe_sha256:
                raise RuntimeError("Installed airframe changed during the experiment")
            parameters, errors, requests = read_parameters(vehicle, self.requested)
            snapshot.update({"parameters": parameters, "read_errors": errors, "request_attempts": requests})
            snapshot["contract_mismatches"] = contract_mismatches(parameters, self.expected)
            snapshot["missing_required"] = sorted(self.required - set(parameters))
            snapshot["unavailable_optional_control_parameters"] = sorted(set(CORE_CONTROL_PARAMETERS) - set(parameters) - self.required)
            snapshot["contract_matches"] = not snapshot["contract_mismatches"] and not snapshot["missing_required"]
            snapshot["control_readback_complete"] = all(name in parameters for name in CORE_CONTROL_PARAMETERS)
            if enforce_contract and not snapshot["contract_matches"]:
                failure = "PX4 physical/output parameter readback does not match the reviewed airframe; arming refused"
            if label == "after" and "before" in self.snapshots:
                before = self.snapshots["before"]["parameters"]
                snapshot["changed_since_before"] = {
                    name: {"before": before.get(name), "after": parameters.get(name)}
                    for name in self.requested
                    if (before.get(name, {}).get("raw_value_hex"), before.get(name, {}).get("param_type"))
                    != (parameters.get(name, {}).get("raw_value_hex"), parameters.get(name, {}).get("param_type"))
                }
                snapshot["parameters_unchanged"] = not snapshot["changed_since_before"]
        except Exception as exc:
            failure = str(exc) or exc.__class__.__name__
        snapshot["finished_wall_time"] = time.time()
        if failure:
            snapshot["error"] = failure
        destination = self.folder / f"px4_parameters_{label}.json"
        destination.write_text(json.dumps(snapshot, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        report = {"file": destination.name, "sha256": sha(destination), "airframe_sha256": self.airframe_sha256,
                  "parameter_count": len(snapshot["parameters"]), "contract_matches": snapshot.get("contract_matches", False),
                  "control_readback_complete": snapshot.get("control_readback_complete", False)}
        if label == "after":
            report["parameters_unchanged"] = snapshot.get("parameters_unchanged", False)
            report["changed_names"] = sorted(snapshot.get("changed_since_before", {}))
        self.snapshots[label], self.reports[label] = snapshot, report
        if failure:
            raise RuntimeError(failure + f"; evidence: {destination}")
        return report
