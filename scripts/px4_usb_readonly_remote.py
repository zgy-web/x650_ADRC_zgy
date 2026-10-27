"""In-memory Linux USB diagnostic. Only version/state/parameter queries may be sent.

This is an observational bring-up tool, not the MPC control loop or a real-time
latency benchmark. The launcher supplies its installed pymavlink common dialect
over stdin; this module uses no third-party installation and writes no files.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import select
import struct
import sys
import time
import types
import zlib


PARAMETERS = (
    "SYS_AUTOSTART", "SYS_HITL", "MAV_PROTO_VER", "MAV_USB_CFG",
    "MAV_0_CONFIG", "MAV_1_CONFIG", "MAV_2_CONFIG",
    "UXRCE_DDS_CFG", "UXRCE_DDS_DOM_ID", "UXRCE_DDS_PRT", "UXRCE_DDS_AG_IP",
    "COM_OF_LOSS_T", "COM_OBL_RC_ACT", "EKF2_EV_CTRL", "EKF2_GPS_CTRL",
    "EKF2_HGT_REF",
)
REQUESTABLE_MESSAGES = (148, 245, 230, 30, 32)  # version, extended state, estimator, attitude, local position


def text_id(value):
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).split("\0", 1)[0]


def wire_identity(message):
    """Older generated parsers leave UNKNOWN_n logical headers unset.

    Preserve the real frame identity for sequence statistics; an unknown message
    still has no schema CRC validation and must remain explicitly unidentified.
    """
    data = bytes(message.get_msgbuf())
    if len(data) >= 10 and data[0] == 253:
        return (data[5], data[6]), data[4]
    if len(data) >= 6 and data[0] == 254:
        return (data[3], data[4]), data[2]
    return (message.get_srcSystem(), message.get_srcComponent()), message.get_seq()


def query_allowed(message):
    """Explicit transmission allowlist; neither parameter writes nor setpoints."""
    name = message.get_type()
    if name == "PARAM_REQUEST_READ":
        return text_id(message.param_id) in PARAMETERS and message.param_index == -1
    if name != "COMMAND_LONG" or any(getattr(message, "param" + str(i)) != 0 for i in range(2, 8)):
        return False
    if message.command == 512:  # MAV_CMD_REQUEST_MESSAGE
        return message.param1 in REQUESTABLE_MESSAGES
    return message.command == 520 and message.param1 == 1  # request capabilities/version


def parameter_value(message, capabilities):
    raw = float(message.param_value)
    # PX4 specifies bytewise PARAM_VALUE encoding. Prefer explicit capability
    # bits when provided; without them retain both interpretations, unselected.
    formats = {1: "B", 2: "b", 3: "H", 4: "h", 5: "I", 6: "i"}
    result = {"type": int(message.param_type), "wire_float": raw if math.isfinite(raw) else str(raw)}
    if message.param_type in formats:
        fmt = formats[message.param_type]
        data = struct.pack("<f", raw)
        bytewise = struct.unpack("<" + fmt, data[:struct.calcsize(fmt)])[0]
        result["bytewise_interpretation"] = bytewise
        if capabilities is not None and capabilities & 16:  # PARAM_ENCODE_BYTEWISE
            result.update(encoding="bytewise", value=bytewise)
        elif capabilities is not None and capabilities & 131072:  # PARAM_ENCODE_C_CAST
            result.update(encoding="c_cast", value=int(raw) if math.isfinite(raw) else None)
        else:
            result.update(encoding="not_confirmed", value=None)
    else:
        result.update(encoding="float", value=result["wire_float"])
    return result


def safe_json(value):
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(v) for v in value]
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def remote_probe(payload):
    import fcntl
    import termios
    import tty

    duration = float(payload.get("duration_s", 25))
    if not 8 <= duration <= 120:
        raise ValueError("Duration must be between 8 and 120 seconds")
    device = str(payload["device"])
    real_device = os.path.realpath(device)
    if not real_device.startswith("/dev/ttyACM"):
        raise ValueError("Only a USB CDC ACM device is allowed")
    if not os.access(device, os.R_OK | os.W_OK):
        raise PermissionError("Current SSH session lacks serial read/write access")
    source = zlib.decompress(base64.b64decode(payload["dialect_zlib_base64"]))
    if hashlib.sha256(source).hexdigest() != payload["dialect_sha256"]:
        raise ValueError("Dialect source hash mismatch")
    os.environ.pop("MAV_IGNORE_CRC", None)
    dialect = types.ModuleType("flyarm_memory_mavlink_common")
    sys.modules[dialect.__name__] = dialect
    exec(compile(source, "<memory:mavlink_common>", "exec"), dialect.__dict__)
    mav = dialect.MAVLink(None, srcSystem=245, srcComponent=191)
    mav.robust_parsing = True
    report = {
        "schema": "flyarm.nx_px4_usb_readonly.v1", "device": device,
        "real_device": real_device, "requested_duration_s": duration,
        "dialect_sha256": payload["dialect_sha256"], "readonly_queries_only": True,
        "measurement_scope": "NX userspace serial receipt/query response; not one-way or hard-real-time latency",
        "nx_persistent_output_files": [], "transmissions": [], "parameters": {},
        "state_samples": {}, "command_acks": [], "statustext": [],
        "startup_fragment_events": [], "bad_data_after_sync": [],
        "unknown_messages": {}, "settled_window_start_s": 5.0,
        "sequence_events": [],
    }
    stats = {}
    settled_stats = {}
    param_messages = {}
    target = None
    capabilities = None
    schedule = []
    pending_parameters = {}
    last_seq = {}
    synchronized = False
    report["sequence"] = {"forward_gap_messages": 0, "duplicate_or_reordered": 0}
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    exclusive = False
    started = time.monotonic()
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        original = termios.tcgetattr(fd)
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200  # USB CDC line coding, not measured bus throughput
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        def send(message):
            if not query_allowed(message):
                raise ValueError("Rejected non-query MAVLink transmission")
            packet = message.pack(mav)
            sent = time.monotonic()
            offset = 0
            while offset < len(packet):
                if time.monotonic() - sent > 0.5:
                    raise TimeoutError("Serial query write timeout")
                _, writable, _ = select.select([], [fd], [], 0.05)
                if writable:
                    try:
                        offset += os.write(fd, packet[offset:])
                    except BlockingIOError:
                        pass
            mav.seq = (mav.seq + 1) & 255
            row = {"type": message.get_type(), "nx_elapsed_s": sent - started, "bytes": len(packet)}
            if message.get_type() == "PARAM_REQUEST_READ":
                key = text_id(message.param_id)
                row["parameter"] = key
                pending_parameters[key] = sent
            else:
                row.update(command=int(message.command), param1=float(message.param1))
            report["transmissions"].append(row)

        while time.monotonic() - started < duration:
            now = time.monotonic()
            if schedule and now >= schedule[0][0]:
                _, request = schedule.pop(0)
                send(request)
            readable, _, _ = select.select([fd], [], [], 0.02)
            if not readable:
                continue
            try:
                data = os.read(fd, 16384)
            except BlockingIOError:
                continue
            if not data:
                raise EOFError("USB serial endpoint closed")
            messages = mav.parse_buffer(data) or []
            received = time.monotonic()
            for message in messages:
                name = message.get_type()
                row = stats.setdefault(name, {"count": 0, "first_s": received-started, "last_s": received-started, "max_gap_ms": 0.0})
                if row["count"]:
                    gap = 1000 * (received-started-row["last_s"])
                    if gap > row["max_gap_ms"]:
                        row["max_gap_ms"] = gap
                        row["max_gap_end_s"] = received-started
                row["count"] += 1
                row["last_s"] = received-started
                if received-started >= report["settled_window_start_s"]:
                    settled = settled_stats.setdefault(name, {"count": 0, "first_s": received-started, "last_s": received-started, "max_gap_ms": 0.0})
                    if settled["count"]:
                        gap = 1000*(received-started-settled["last_s"])
                        if gap > settled["max_gap_ms"]:
                            settled.update(max_gap_ms=gap, max_gap_end_s=received-started)
                    settled["count"] += 1
                    settled["last_s"] = received-started
                if name == "BAD_DATA":
                    bucket = "bad_data_after_sync" if synchronized else "startup_fragment_events"
                    if len(report[bucket]) < 30:
                        report[bucket].append({"elapsed_s": received-started,
                                               "reason": str(getattr(message, "reason", "")),
                                               "prefix_hex": bytes(getattr(message, "data", b""))[:64].hex()})
                    continue
                if name.startswith("UNKNOWN_"):
                    report["unknown_messages"][name] = report["unknown_messages"].get(name, 0)+1
                else:
                    synchronized = True
                peer, seq = wire_identity(message)
                if peer in last_seq:
                    delta = (seq-last_seq[peer]) & 255
                    if 1 < delta < 128:
                        report["sequence"]["forward_gap_messages"] += delta-1
                    elif delta == 0 or delta >= 128:
                        report["sequence"]["duplicate_or_reordered"] += 1
                    if delta != 1 and len(report["sequence_events"]) < 100:
                        report["sequence_events"].append(dict(elapsed_s=received-started, peer=list(peer),
                                                               previous=last_seq[peer], current=seq, delta=delta,
                                                               message_type=name))
                last_seq[peer] = seq
                if name == "HEARTBEAT" and int(message.autopilot) == 12 and peer[1] == 1:
                    report["heartbeat"] = dict(message.to_dict(), system_id=peer[0], component_id=peer[1], armed=bool(message.base_mode & 128), hil_enabled=bool(message.base_mode & 32))
                    if target is None:
                        target = peer
                        requests = [dialect.MAVLink_command_long_message(*target, 512, 0, mid, 0, 0, 0, 0, 0, 0) for mid in REQUESTABLE_MESSAGES]
                        requests += [dialect.MAVLink_param_request_read_message(*target, key.encode("ascii"), -1) for key in PARAMETERS]
                        schedule = [(received+0.15*(i+1), req) for i, req in enumerate(requests)]
                        # Legacy version query is also observational; useful on older firmware.
                        schedule.append((received+4.0, dialect.MAVLink_command_long_message(*target, 520, 0, 1, 0, 0, 0, 0, 0, 0)))
                if target is None or peer != target:
                    continue
                if name == "AUTOPILOT_VERSION":
                    capabilities = int(message.capabilities)
                    version = int(message.flight_sw_version)
                    report["firmware"] = dict(message.to_dict(), semver=f"{version >> 24}.{(version >> 16) & 255}.{(version >> 8) & 255}", release_type=version & 255)
                elif name == "PARAM_VALUE":
                    key = text_id(message.param_id)
                    if key in PARAMETERS:
                        param_messages[key] = message
                        if key in pending_parameters:
                            report.setdefault("parameter_query_response_ms", {})[key] = 1000*(received-pending_parameters.pop(key))
                elif name == "COMMAND_ACK":
                    report["command_acks"].append(message.to_dict())
                elif name == "STATUSTEXT":
                    report["statustext"] = (report["statustext"] + [message.to_dict()])[-30:]
                elif name in ("ATTITUDE", "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "ODOMETRY", "ESTIMATOR_STATUS", "EXTENDED_SYS_STATE", "GPS_RAW_INT", "SYS_STATUS", "HIGHRES_IMU"):
                    report["state_samples"][name] = message.to_dict()
        report["elapsed_s"] = time.monotonic()-started
        for row in list(stats.values())+list(settled_stats.values()):
            span = row["last_s"]-row["first_s"]
            row["observed_rate_hz"] = (row["count"]-1)/span if span > 0 else None
        report["message_statistics"] = stats
        report["settled_message_statistics"] = settled_stats
        report["parameters"] = {key: parameter_value(message, capabilities) for key, message in param_messages.items()}
        report["unanswered_parameters"] = [key for key in PARAMETERS if key not in param_messages]
        report["checks"] = {
            "px4_autopilot_heartbeat": target is not None,
            "firmware_version_received": "firmware" in report,
            "parameter_read_roundtrip": bool(report.get("parameter_query_response_ms")),
            "attitude_received": "ATTITUDE" in report["state_samples"] or "ATTITUDE_QUATERNION" in report["state_samples"],
            "no_bad_data_after_stream_sync": not report["bad_data_after_sync"],
        }
        report["basic_link_pass"] = all(report["checks"].values())
        return safe_json(report)
    finally:
        if original is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, original)
            except OSError:
                pass
        if exclusive:
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError:
                pass
        os.close(fd)


if __name__ == "__main__":
    try:
        print(json.dumps(remote_probe(json.load(sys.stdin)), indent=2, allow_nan=False))
    except Exception as exc:
        print(json.dumps({"basic_link_pass": False, "error_type": type(exc).__name__, "error": str(exc)}, indent=2))
        raise SystemExit(2)
