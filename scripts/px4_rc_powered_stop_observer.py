"""Read-only, bounded observer for a human-operated props-off motor-stop test.

No actuator_test override: the operator owns SF arming and SB emergency stop.
This program does not enforce a motor-run timeout or stop a physical motor.
CLI requires an attended, secured, props-off bench and connected ESC power.
Results remain on the PC. Sampling is sequential, not a latency measurement.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from px4_debug_console import DebugConsole, FIRMWARE, clean, status_fields
from px4_rc_safety_observer import TOPICS, summarize
from px4_rc_switch_setup import boolean, field, fresh, number
from px4_motor_single_pulse import require_zero
from px4_motor_telemetry_check import telemetry_snapshot, assess_samples

READS = frozenset(("ver all", "dshot status", "commander status", "dmesg") + tuple(
    "listener " + name + " -n 1" for name in TOPICS + ("esc_status", "health_report", "battery_status")))
ATTESTATIONS = ("props_removed", "frame_secured", "operator_ready", "esc_power_connected")


class PoweredReadConsole(DebugConsole):
    def query(self, command):
        if command not in READS or self.active_listener:
            raise ValueError("Only fixed powered-bench reads are permitted")
        self.s.write((command + "\r").encode("ascii"))
        text = clean(self.record(command, self.read(6)))
        if "nsh>" not in text:
            raise RuntimeError("No prompt after " + command)
        return text


def snapshot(c):
    raw = {name: c.query("listener " + name + " -n 1") for name in
           ("vehicle_status", "actuator_armed", "manual_control_switches", "input_rc",
            "manual_control_setpoint", "action_request")}
    raw["dshot"] = c.query("dshot status")
    raw["esc_status"] = c.query("listener esc_status -n 1")
    raw["battery_status"] = c.query("listener battery_status -n 1")
    row = dict(raw=raw, summary=summarize(raw))
    try:
        row["telemetry"] = telemetry_snapshot(raw["dshot"], raw["esc_status"], require_stopped=False)
    except (ValueError, TypeError) as exc:
        row["telemetry"] = dict(unknown=str(exc))
    return row


def parked(row, power_profile="bench20"):
    if power_profile not in ("bench20", "6s_battery"):
        raise ValueError("Unknown power profile")
    raw = row["raw"]
    status_fields(raw["vehicle_status"])
    require_zero(raw["dshot"])
    fresh(raw["actuator_armed"], 1.5)
    for name in ("armed", "prearmed", "lockdown", "termination", "in_esc_calibration_mode"):
        if boolean(raw["actuator_armed"], name):
            raise ValueError("Unexpected initial actuator flag: " + name)
    if not boolean(raw["actuator_armed"], "kill"):
        raise ValueError("Initial actual kill must be set before powered recording")
    fresh(raw["input_rc"], .25)
    if (boolean(raw["input_rc"], "rc_lost") or boolean(raw["input_rc"], "rc_failsafe") or
            not 0 <= number(raw["input_rc"], "timestamp") - number(raw["input_rc"], "timestamp_last_signal") <= 100000):
        raise ValueError("RC signal unavailable")
    fresh(raw["manual_control_setpoint"], .3)
    if (not boolean(raw["manual_control_setpoint"], "valid") or
            number(raw["manual_control_setpoint"], "data_source") != 1 or
            number(raw["manual_control_setpoint"], "throttle") > -.95 or
            any(abs(number(raw["manual_control_setpoint"], x)) > .05 for x in ("roll", "pitch", "yaw"))):
        raise ValueError("Require valid RC with minimum throttle and centered axes")
    fresh(raw["manual_control_switches"], 1.5)
    if tuple(int(field(raw["manual_control_switches"], x)) for x in
             ("arm_switch", "mode_slot", "kill_switch")) != (3, 1, 1):
        raise ValueError("Require SF off, SA Altitude, SB kill")
    telem = telemetry_snapshot(raw["dshot"], raw["esc_status"])
    low, high = (18, 22) if power_profile == "bench20" else (22, 25.5)
    if any(not low <= esc["esc_voltage"] <= high for esc in telem["samples"].values()):
        raise ValueError("Voltage outside selected power profile")
    if power_profile == "6s_battery":
        from px4_rc_battery_metadata import validate_battery
        validate_battery(raw["battery_status"])
    return telem


def run(port, output, *, seconds, attestations, power_profile="bench20"):
    if (not 10 <= seconds <= 180 or power_profile not in ("bench20", "6s_battery") or
            any(attestations.get(x) is not True for x in ATTESTATIONS)):
        raise ValueError("Require attended props-off bench attestations, power profile and 10..180 seconds")
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    c = PoweredReadConsole(port)
    report = dict(kind="human_rc_powered_motor_stop_observation", firmware=FIRMWARE,
                  utc_started=datetime.now(timezone.utc).isoformat(), attestations=attestations,
                  power_profile=power_profile,
                  complete=False, rounds=[], arm_commands=0, motor_commands=0, mode_commands=0,
                  parameter_writes=0, nx_accessed=False, physical_stop_verified=False,
                  flight_ready=False, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in c.query("ver all"):
            raise ValueError("Unexpected firmware")
        report["before"] = snapshot(c)
        first = parked(report["before"], power_profile)
        time.sleep(.3)
        report["before_confirm"] = snapshot(c)
        second = parked(report["before_confirm"], power_profile)
        report["initial_telemetry_check"] = assess_samples([first, second])
        print("POWERED_READONLY_OBSERVER_READY " + json.dumps(report["before_confirm"]["summary"]), flush=True)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            start = time.monotonic()
            row = snapshot(c)
            row.update(pc_start_monotonic=start, pc_end_monotonic=time.monotonic())
            report["rounds"].append(row)
            s = row["summary"]
            rpm = {k: v["esc_rpm"] for k, v in row.get("telemetry", {}).get("samples", {}).items()}
            print(json.dumps(dict(event="sample", armed=s.get("actuator_armed"),
                                  outputs=s.get("dshot"), rpm=rpm)), flush=True)
        report["final"] = snapshot(c)
        report["commander_status"] = c.query("commander status")
        report["complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        c.close()
        report["port_closed"] = not c.s.is_open
        (out / "console.json").write_text(json.dumps(c.records, indent=2), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((out / "console.json").read_bytes()).hexdigest()
        (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM13")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=60)
    p.add_argument("--power-profile", choices=("bench20", "6s_battery"), default="bench20")
    for name in ATTESTATIONS:
        p.add_argument("--" + name.replace("_", "-"), action="store_true")
    a = p.parse_args()
    r = run(a.port, a.output, seconds=a.seconds, attestations={x: getattr(a, x) for x in ATTESTATIONS}, power_profile=a.power_profile)
    print(json.dumps({k: r.get(k) for k in ("complete", "error", "port_closed")}))
    raise SystemExit(0 if r["complete"] else 1)
