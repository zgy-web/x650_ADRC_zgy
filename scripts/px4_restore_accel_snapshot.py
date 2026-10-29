"""Restore only the 18 accelerometer offsets/scales from an owned calibration session.

Used to recover from a completed calibration that fails physical acceptance.
Requires the same firmware, sensor IDs and current parameter readback. No arm,
motor, reboot, level or calibration command. Unknown writes are never retried.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from px4_debug_console import FIRMWARE, clean
from px4_imu_pole_check import ImuConsole, PARAMETERS, READS, guard
from px4_rc_switch_setup import read_parameter

NAMES = tuple(f"CAL_ACC{i}_{axis}{kind}" for i in range(3)
              for kind in ("OFF", "SCALE") for axis in "XYZ")


def restore_plan(session):
    r = session["report"]
    if r["firmware"] != FIRMWARE or not r["calibration_success"] or not r["post_readback_complete"]:
        raise ValueError("Requires a completed, fully read-back owned calibration session")
    before, current = r["before_parameters"], r["after_parameters"]
    if set(before) != set(PARAMETERS) or set(current) != set(PARAMETERS):
        raise ValueError("Incomplete parameter snapshot")
    for name in PARAMETERS:
        if name not in NAMES and before[name] != current[name]:
            raise ValueError("Unrelated parameter changed during calibration")
    for name in NAMES:
        for value in (before[name], current[name]):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("Invalid calibration number")
            if name.endswith("SCALE") and not .5 <= value <= 1.5:
                raise ValueError("Scale outside bounded recovery range")
            if name.endswith("OFF") and abs(value) > 3:
                raise ValueError("Offset outside bounded recovery range")
    return {name: before[name] for name in NAMES}


class RestoreConsole(ImuConsole):
    def __init__(self, port, plan):
        super().__init__(port)
        self.fixed_commands = {f"param set {name} {value:.9g}" for name, value in plan.items()} | {"param save"}

    def query(self, command):
        if command in READS:
            return super().query(command)
        if self.active_listener or command not in self.fixed_commands:
            raise ValueError("Outside fixed accelerometer restoration")
        self.s.write((command + "\r").encode("ascii"))
        result = self.record(command, self.read(6))
        if "nsh>" not in result:
            raise RuntimeError("No confirmation; write will not be retried")
        if any(word in clean(result).lower() for word in ("error", "failed", "not found")):
            raise RuntimeError("Parameter command reported failure")
        return result


def run(session_path, output, *, port="COM13", operator_power_off=False):
    if not operator_power_off:
        raise ValueError("Operator must confirm ESC power is physically off")
    source = Path(session_path).read_bytes()
    session = json.loads(source)
    plan = restore_plan(session)
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    c = RestoreConsole(port, plan)
    report = dict(complete=False, firmware=FIRMWARE, target=plan, attempts=[], verified=[],
                  session_sha256=hashlib.sha256(source).hexdigest(), operator_power_off=True,
                  utc_started=datetime.now(timezone.utc).isoformat(), motor_commands=0,
                  arm_commands=0, nx_accessed=False, power_cycle_verified=False,
                  baseline_is_rounded_console_readback=True, flight_ready=False)
    try:
        c.open()
        if FIRMWARE not in clean(c.query("ver all")):
            raise ValueError("Unexpected firmware")
        report["before"] = guard(c, allow_unpublished_battery=True)
        report["before_parameters"] = {p: read_parameter(c, p) for p in PARAMETERS}
        if report["before_parameters"] != session["report"]["after_parameters"]:
            raise ValueError("Actual parameters differ from the reviewed calibration result")
        report["prewrite"] = guard(c, allow_unpublished_battery=True)
        for name, value in plan.items():
            command = f"param set {name} {value:.9g}"
            report["attempts"].append(command)
            c.query(command)
            if abs(read_parameter(c, name) - value) > 0.000051:
                raise ValueError("Restored parameter did not read back: " + name)
            report["verified"].append(name)
        guard(c, allow_unpublished_battery=True)
        report["save_response"] = clean(c.query("param save"))
        report["after_parameters"] = {p: read_parameter(c, p) for p in PARAMETERS}
        expected = {**session["report"]["after_parameters"], **plan}
        if report["after_parameters"] != expected:
            raise ValueError("Final readback differs from the restoration plan")
        report["after"] = guard(c, allow_unpublished_battery=True)
        report["complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        c.close()
        report["port_closed"] = not c.s.is_open
        report["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (out / "console.json").write_text(json.dumps(c.records, ensure_ascii=False, indent=2), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((out / "console.json").read_bytes()).hexdigest()
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--session", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--esc-power-off", action="store_true")
    p.add_argument("--port", default="COM13")
    a = p.parse_args()
    r = run(a.session, a.output, port=a.port, operator_power_off=a.esc_power_off)
    print(json.dumps({k: r.get(k) for k in ("complete", "error", "verified", "after", "port_closed")}, indent=2))
    raise SystemExit(0 if r["complete"] else 1)
