"""Bounded PC-only observer of human RC actions with ESC power disconnected.

Only fixed read commands are allowed. No arming/disarming, mode, parameter,
motor, calibration or NX commands are implemented. A completed recording is
not a passed safety test. Hardware motion and flight failsafes need separate
evidence. Initial state must be disarmed; subsequent human actions are recorded.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from px4_debug_console import DebugConsole, FIRMWARE, clean
from px4_imu_pole_check import guard
from px4_rc_switch_setup import field, fresh, boolean, number
from px4_motor_single_pulse import dshot_channels


TOPICS = ("vehicle_status", "actuator_armed", "manual_control_switches",
          "input_rc", "action_request", "manual_control_setpoint", "failsafe_flags")
READS = frozenset(("ver all", "dshot status", "commander status") +
                 tuple("listener " + name + " -n 1" for name in TOPICS + (
                     "battery_status", "health_report", "vehicle_control_mode")))


class ObserverConsole(DebugConsole):
    def query(self, command):
        if command not in READS or self.active_listener:
            raise ValueError("Only fixed safety-observer reads are permitted")
        self.s.write((command + "\r").encode("ascii"))
        response = self.record(command, self.read(6))
        if "nsh>" not in response:
            raise RuntimeError("No prompt after read: " + command)
        return clean(response)


def fields(raw, names, conversion):
    return {k: conversion(raw, k) for k in names}


def summarize(raw):
    result = {}
    for name, text in raw.items():
        try:
            if name == "vehicle_status":
                fresh(text, 1.5)
                result[name] = fields(text, ("timestamp", "arming_state", "nav_state"), lambda t, k: int(field(t, k)))
                result[name]["failsafe"] = boolean(text, "failsafe")
            elif name == "actuator_armed":
                fresh(text, 1.5)
                result[name] = fields(text, ("armed", "prearmed", "ready_to_arm", "kill", "termination"), boolean)
            elif name == "manual_control_switches":
                # Invalid RC can leave an old published switch value. Keep age
                # information in raw text; never call it current without RC valid.
                result[name] = fields(text, ("timestamp", "arm_switch", "kill_switch", "mode_slot"), lambda t, k: int(field(t, k)))
            elif name == "input_rc":
                fresh(text, 1.5)
                result[name] = fields(text, ("timestamp", "timestamp_last_signal", "channel_count", "input_source"), lambda t, k: int(field(t, k)))
                result[name].update(fields(text, ("rc_lost", "rc_failsafe"), boolean))
            elif name == "action_request":
                result[name] = fields(text, ("timestamp", "action", "source", "mode"), lambda t, k: int(field(t, k)))
            elif name == "manual_control_setpoint":
                result[name] = fields(text, ("timestamp", "data_source"), lambda t, k: int(field(t, k)))
                result[name]["valid"] = boolean(text, "valid")
                result[name]["throttle"] = number(text, "throttle")
            elif name == "failsafe_flags":
                # Event-driven topic may legitimately have an older timestamp.
                result[name] = fields(text, ("manual_control_signal_lost", "offboard_control_signal_lost",
                                             "local_position_invalid", "local_velocity_invalid"), boolean)
            elif name == "dshot":
                result[name] = dshot_channels(text)
        except (ValueError, TypeError) as exc:
            result[name] = {"unknown": str(exc)}
    return result


def run(port, output, *, seconds, operator_power_off=False):
    if not operator_power_off or not 1 <= seconds <= 90:
        raise ValueError("Require confirmed ESC power off and 1..90 second window")
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    c = ObserverConsole(port)
    report = dict(kind="px4_human_rc_action_observation", firmware=FIRMWARE,
                  utc_started=datetime.now(timezone.utc).isoformat(), operator_power_off=True,
                  complete=False, rounds=[], arm_commands=0, motor_commands=0,
                  parameter_writes=0, mode_commands=0, nx_accessed=False,
                  physical_stop_verified=False, armed_failsafe_verified=False, flight_ready=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in c.query("ver all"):
            raise ValueError("Unexpected firmware")
        report["before"] = guard(c, allow_unpublished_battery=True)
        print("READONLY_OBSERVER_READY " + json.dumps(report["before"]), flush=True)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            sample = dict(pc_start_monotonic=time.monotonic(), raw={})
            for name in TOPICS:
                sample["raw"][name] = c.query("listener " + name + " -n 1")
            sample["raw"]["dshot"] = c.query("dshot status")
            sample["pc_end_monotonic"] = time.monotonic()
            sample["summary"] = summarize(sample["raw"])
            report["rounds"].append(sample)
            time.sleep(.1)
        report["final_raw"] = {name: c.query("listener " + name + " -n 1") for name in TOPICS}
        report["final_raw"]["dshot"] = c.query("dshot status")
        report["final_summary"] = summarize(report["final_raw"])
        report["commander_status"] = c.query("commander status")
        report["control_mode"] = c.query("listener vehicle_control_mode -n 1")
        report["health_report"] = c.query("listener health_report -n 1")
        report["complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        c.close()
        report["port_closed"] = not c.s.is_open
        (out / "console.json").write_text(json.dumps(c.records, ensure_ascii=False, indent=2), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((out / "console.json").read_bytes()).hexdigest()
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM13")
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--seconds", type=float, default=30)
    p.add_argument("--esc-power-off", action="store_true")
    a = p.parse_args()
    r = run(a.port, a.output, seconds=a.seconds, operator_power_off=a.esc_power_off)
    print(json.dumps({k: r.get(k) for k in ("complete", "error", "final_summary", "port_closed")}, ensure_ascii=False, indent=2))
    raise SystemExit(0 if r["complete"] else 1)
