"""One-time fixed RC mapping for the observed TX16S/6X Pro bench.

Default is plan only. Execute with ESC power physically disconnected, SF/CH5
low, SA/CH6 low, SB/CH7 high, throttle low and other axes centered. All logs
stay on the PC. No arming, motor, trajectory or NX command is implemented.
Partial failure keeps the actual applied settings; it never removes a kill
mapping automatically. This is not a generic PX4 parameter-writing utility.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import time

from px4_debug_console import DebugConsole, FIRMWARE, clean, status_fields
from px4_motor_preflight_readonly import parse_parameter
from px4_motor_single_pulse import require_zero
from px4_rc_preflight_readonly import ALLOWED as READ_ALLOWED


# Set kill first and verify its real actuator_armed flag before the other writes.
# The observed 3-position SA occupies PX4's six-mode slots 1, 4 and 6.
PLAN = (
    ("RC_MAP_KILL_SW", 7),
    ("COM_FLTMODE1", 1),
    ("COM_FLTMODE4", 2),
    ("COM_FLTMODE6", 7),
    ("RC_MAP_FLTMODE", 6),
    ("RC_MAP_ARM_SW", 5),
)
EXPECTED = {
    "RC_MAP_KILL_SW": 0, "RC_MAP_ARM_SW": 0, "RC_MAP_FLTMODE": 0,
    "RC_MAP_MODE_SW": 0, "RC_MAP_TERM_SW": 0, "RC_MAP_FLTM_BTN": 0,
    "RC_MAP_RETURN_SW": 0, "RC_MAP_OFFB_SW": 0, "RC_MAP_LOITER_SW": 0,
    "COM_ARM_SWISBTN": 0, "RC_ARMSWITCH_TH": 0.75, "RC_KILLSWITCH_TH": 0.75,
    "RC_MAP_ROLL": 1, "RC_MAP_PITCH": 2, "RC_MAP_THROTTLE": 3, "RC_MAP_YAW": 4,
    "RC_CHAN_CNT": 16,
    **{f"COM_FLTMODE{i}": -1 for i in range(1, 7)},
    **{f"RC{i}_REV": 1 for i in range(1, 8)},
    **{f"RC{i}_{name}": value for i in range(5, 8)
       for name, value in (("MIN", 1000), ("TRIM", 1500), ("MAX", 2000))},
}
WRITE_ALLOWED = frozenset(f"param set {name} {value}" for name, value in PLAN) | {"param save"}
# PX4 ManualControlSwitches.msg: ON=1, MIDDLE=2, OFF=3, NONE=0.
SWITCH_ON = 1
SWITCH_OFF = 3


class SetupConsole(DebugConsole):
    def __init__(self, port, *, execute=False):
        super().__init__(port)
        self.execute = execute

    def query(self, command):
        if self.active_listener or not (command in READ_ALLOWED or
                                       (self.execute and command in WRITE_ALLOWED)):
            raise ValueError("Command outside fixed RC setup scope")
        self.s.write((command + "\r").encode("ascii"))
        response = self.record(command, self.read(6))
        if "nsh>" not in response:
            raise RuntimeError("No prompt after " + command)
        return response


def field(text, name):
    matches = re.findall(r"^\s*" + re.escape(name) + r":\s*(\S+)", clean(text), re.M)
    if len(matches) != 1:
        raise ValueError("Missing or ambiguous field: " + name)
    return matches[0]


def boolean(text, name):
    value = field(text, name)
    if value not in ("True", "False"):
        raise ValueError("Invalid Boolean: " + name)
    return value == "True"


def number(text, name):
    value = float(field(text, name))
    if not math.isfinite(value):
        raise ValueError("Nonfinite field: " + name)
    return value


def fresh(text, max_age):
    matches = re.findall(r"^\s*timestamp:\s*(\d+)\s*\(([^ ]+) seconds ago\)", clean(text), re.M)
    if len(matches) != 1:
        raise ValueError("Missing or ambiguous timestamp age")
    stamp, age = int(matches[0][0]), float(matches[0][1])
    if stamp <= 0 or not math.isfinite(age) or not 0 <= age <= max_age:
        raise ValueError("Stale topic")
    return stamp


def validate_inputs(raw):
    stamp = fresh(raw, .1)
    last_signal = int(field(raw, "timestamp_last_signal"))
    if not 0 <= stamp - last_signal <= 100_000 or last_signal <= 0:
        raise ValueError("Stale RC signal")
    if boolean(raw, "rc_lost") or boolean(raw, "rc_failsafe"):
        raise ValueError("RC is lost or failsafe")
    if int(field(raw, "input_source")) != 14 or int(field(raw, "channel_count")) != 16:
        raise ValueError("Unexpected RC source/count")
    arrays = re.findall(r"^\s*values:\s*(\[[^\]\n]+\])", clean(raw), re.M)
    if len(arrays) != 1:
        raise ValueError("Missing or ambiguous channel array")
    values = json.loads(arrays[0])
    if len(values) < 16 or any(type(x) is not int for x in values):
        raise ValueError("Invalid RC channel array")
    if any(abs(values[i] - 1500) > 60 for i in (0, 1, 3)):
        raise ValueError("Roll/pitch/yaw must be centered")
    if not all(950 <= values[i] <= 1100 for i in (2, 4, 5)) or not 1900 <= values[6] <= 2050:
        raise ValueError("Require throttle low, SF low, SA low, SB high")
    return values


def snapshot(c, *, kill_required=False):
    state = status_fields(c.query("listener vehicle_status -n 1"))
    outputs = require_zero(c.query("dshot status"))
    battery = c.query("listener battery_status -n 1")
    fresh(battery, 1)
    if boolean(battery, "connected") or not 0 <= number(battery, "voltage_v") < 1:
        raise ValueError("Battery observation inconsistent with ESC power off")
    values = validate_inputs(c.query("listener input_rc -n 1"))
    manual = c.query("listener manual_control_setpoint -n 1")
    fresh(manual, .25)
    if not boolean(manual, "valid") or int(field(manual, "data_source")) != 1:
        raise ValueError("Manual input is not valid RC")
    if any(abs(number(manual, x)) > .05 for x in ("roll", "pitch", "yaw")) or not -1.001 <= number(manual, "throttle") <= -.95:
        raise ValueError("Manual input not centered with minimum throttle")
    switches = c.query("listener manual_control_switches -n 1")
    fresh(switches, 1.5)  # This topic republishes unchanged switches at about 1 Hz.
    sw = {name: int(field(switches, name)) for name in ("arm_switch", "kill_switch", "mode_slot")}
    armed = c.query("listener actuator_armed -n 1")
    fresh(armed, 1.5)
    if any(boolean(armed, key) for key in ("armed", "prearmed", "termination", "in_esc_calibration_mode")):
        raise ValueError("Unsafe actuator state")
    kill = boolean(armed, "kill")
    if kill_required and (not kill or sw["kill_switch"] != SWITCH_ON):
        raise ValueError("Kill has not been confirmed")
    return dict(state=state, outputs=outputs, channels=values, switches=sw, actuator_kill=kill)


def poll(c, predicate):
    deadline = time.monotonic() + 4
    while True:
        current = snapshot(c)
        if predicate(current):
            return current
        if time.monotonic() >= deadline:
            raise ValueError("Setting did not reach required physical readback")
        time.sleep(.1)


def read_parameter(c, name):
    # A console WARN may arrive between commands. Keep the strict parser and
    # retry only the read; never resend the preceding write on ambiguity.
    for _ in range(3):
        value = parse_parameter(c.query("param show -q " + name))
        if value is not None:
            return value
        time.sleep(.1)
    raise ValueError("Parameter remains unknown: " + name)


def applied_prefix(parameters, resume):
    targets = dict(PLAN)
    for name, expected in EXPECTED.items():
        if name not in targets and parameters.get(name) != expected:
            raise ValueError("Unexpected initial " + name)
    prefix = 0
    stopped = False
    for name, target in PLAN:
        actual = parameters.get(name)
        if resume and not stopped and actual == target:
            prefix += 1
        elif actual == EXPECTED[name]:
            stopped = True
        else:
            raise ValueError("Configuration is not an approved applied prefix: " + name)
    return prefix


def apply(port, output, *, operator_power_off=False, resume=False):
    if not operator_power_off:
        raise ValueError("Physical ESC power-off confirmation required")
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    c = SetupConsole(port, execute=True)
    report = dict(kind="fixed_px4_rc_switch_setup", utc_started=datetime.now(timezone.utc).isoformat(),
                  firmware=FIRMWARE, complete=False, plan=PLAN, operator_power_off=True, resume=resume,
                  parameter_attempts=[], before_parameters={}, after_parameters={},
                  arm_commands=0, motor_commands=0, explicit_mode_commands=0, nx_accessed=False,
                  note="RC mapping changes may generate internal kill/disarm/mode requests.",
                  powercycle_persistence_verified=False, rc_takeover_verified=False,
                  armed_failsafe_verified=False, physical_stop_verified=False, flight_ready=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in clean(c.query("ver all")):
            raise ValueError("Unexpected firmware")
        report["before"] = snapshot(c)
        for key in EXPECTED:
            report["before_parameters"][key] = read_parameter(c, key)
        prefix = applied_prefix(report["before_parameters"], resume)
        report["previously_applied_prefix"] = prefix
        if prefix:
            report["resumed_kill_confirmed"] = snapshot(c, kill_required=True)
        for index, (name, value) in enumerate(PLAN[prefix:], start=prefix):
            snapshot(c, kill_required=index > 0)
            attempt = dict(name=name, value=value, attempted=True, response_received=False, readback_verified=False)
            report["parameter_attempts"].append(attempt)
            answer = c.query(f"param set {name} {value}")
            attempt["response_received"] = True
            if "ERROR" in clean(answer):
                raise ValueError("PX4 reported parameter write error")
            actual = read_parameter(c, name)
            report["after_parameters"][name] = actual
            if actual != value:
                raise ValueError("Write readback mismatch: " + name)
            attempt["readback_verified"] = True
            if index == 0:
                report["kill_confirmed"] = poll(c, lambda s: s["actuator_kill"] and s["switches"]["kill_switch"] == SWITCH_ON)
        report["after"] = poll(c, lambda s: s["actuator_kill"] and s["switches"] ==
                               dict(arm_switch=SWITCH_OFF, kill_switch=SWITCH_ON, mode_slot=1) and s["state"]["nav_state"] == 1)
        for name, target in PLAN:
            report["after_parameters"][name] = read_parameter(c, name)
            if report["after_parameters"][name] != target:
                raise ValueError("Final parameter mismatch: " + name)
        save_reply = c.query("param save")
        report["save_reply"] = clean(save_reply)
        if "ERROR" in clean(save_reply):
            raise ValueError("PX4 reported save error")
        report["complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        c.close()
        report["port_closed"] = not c.s.is_open
        (out / "console.json").write_text(json.dumps(c.records, indent=2), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((out / "console.json").read_bytes()).hexdigest()
        (out / "setup_report.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--esc-power-off", action="store_true")
    parser.add_argument("--resume", action="store_true", help="Accept only an already applied prefix of this fixed plan")
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps(dict(plan=PLAN, execution=False), indent=2))
        return 0
    if not args.output:
        parser.error("--output is required when executing")
    result = apply(args.port, args.output, operator_power_off=args.esc_power_off, resume=args.resume)
    print(json.dumps(result, indent=2))
    return 0 if result["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
