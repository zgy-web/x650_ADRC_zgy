"""Stationary IMU observer and fixed MN4014 pole-count correction on the PC.

No calibration, arm, mode, motor-test, reboot or NX commands are implemented.
The only writes are MOT_POLE_COUNT=24 and param save, behind an explicit CLI
flag and fresh disarmed/power-off/zero-output/kill checks. Raw reads are retained.
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
from px4_motor_single_pulse import require_zero
from px4_rc_switch_setup import boolean, field, fresh, number, read_parameter


PARAMETERS = (
    "MOT_POLE_COUNT", "SENS_IMU_MODE", "EKF2_MULTI_IMU", "EKF2_ABL_LIM",
    "EKF2_ABL_GYRLIM", "SENS_BOARD_ROT", "SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF",
    "SENS_BOARD_Z_OFF", "COM_ARM_IMU_ACC", "COM_ARM_IMU_GYR", "SYS_HAS_GPS",
    "DSHOT_BIDIR_EN", "DSHOT_TEL_CFG", "IMU_GYRO_DNF_EN",
) + tuple(f"CAL_ACC{i}_{name}" for i in range(3) for name in
          ("ID", "PRIO", "XOFF", "YOFF", "ZOFF", "XSCALE", "YSCALE", "ZSCALE"))
STATUS = (
    "ver all", "commander status", "sensors status", "ekf2 status", "dshot status",
    "listener vehicle_status -n 1", "listener actuator_armed -n 1",
    "listener battery_status -n 1", "listener sensor_selection -n 1",
    "listener estimator_selector_status -n 1", "listener vehicle_attitude -n 1",
    "listener vehicle_acceleration -n 1", "listener failsafe_flags -n 1",
    "listener health_report -n 1",
)
INSTANCE_TOPICS = ("estimator_sensor_bias", "estimator_status", "estimator_status_flags",
                   "sensor_accel", "vehicle_imu_status")
READS = frozenset(STATUS + tuple(f"param show -q {p}" for p in PARAMETERS) + tuple(
    f"listener {topic} -i {i} -n 1" for topic in INSTANCE_TOPICS for i in range(9)))
WRITES = frozenset(("param set MOT_POLE_COUNT 24", "param save"))


class ImuConsole(DebugConsole):
    def __init__(self, port, *, allow_pole_write=False):
        super().__init__(port)
        self.allow_pole_write = allow_pole_write

    def query(self, command):
        if self.active_listener or not (command in READS or
                                       (self.allow_pole_write and command in WRITES)):
            raise ValueError("Command outside fixed IMU/pole scope")
        self.s.write((command + "\r").encode("ascii"))
        response = self.record(command, self.read(6))
        if "nsh>" not in response:
            raise RuntimeError("No prompt after " + command)
        return response


def vector(raw, name, length=3):
    matches = re.findall(r"^\s*" + re.escape(name) + r":\s*(\[[^\]\n]*\])", clean(raw), re.M)
    if len(matches) != 1:
        raise ValueError("Missing or ambiguous vector: " + name)
    values = json.loads(matches[0])
    if len(values) != length or any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
        raise ValueError("Invalid vector: " + name)
    return values


def assess_bias(raw):
    stamp = fresh(raw, 2)
    result = dict(timestamp=stamp, accel_device_id=int(field(raw, "accel_device_id")),
                  gyro_device_id=int(field(raw, "gyro_device_id")))
    for sensor in ("accel", "gyro"):
        valid = boolean(raw, sensor + "_bias_valid")
        bias = vector(raw, sensor + "_bias")
        variance = vector(raw, sensor + "_bias_variance")
        limit = number(raw, sensor + "_bias_limit")
        if limit <= 0:
            raise ValueError("Invalid bias limit")
        # Same per-axis 0.75 * limit + 3 sigma check as the pinned FC firmware.
        thresholds = [.75 * limit + 3 * math.sqrt(max(v, 0)) for v in variance]
        result[sensor] = dict(valid=valid, stable=boolean(raw, sensor + "_bias_stable"),
                              bias=bias, variance=variance, limit=limit, thresholds=thresholds,
                              exceeds=[abs(b) > t for b, t in zip(bias, thresholds)] if valid else None)
    return result


def guard(c, *, require_kill=False, allow_unpublished_battery=False):
    state_raw = c.query("listener vehicle_status -n 1")
    fresh(state_raw, 1.5)
    state = status_fields(state_raw)
    outputs = require_zero(c.query("dshot status"))
    armed = c.query("listener actuator_armed -n 1")
    fresh(armed, 1.5)
    if any(boolean(armed, k) for k in ("armed", "prearmed", "termination", "in_esc_calibration_mode")):
        raise ValueError("Unexpected actuator state")
    kill = boolean(armed, "kill")
    if require_kill and not kill:
        raise ValueError("Kill must remain engaged for the fixed write")
    battery = c.query("listener battery_status -n 1")
    if allow_unpublished_battery and re.search(r"^never published$", clean(battery), re.M) and "TOPIC:" not in battery:
        # Cold USB-only boot may have no battery publisher. Physical ESC power-off
        # confirmation remains required by the caller; missing telemetry is NOT 0 V.
        return dict(state=state, outputs=outputs, kill=kill, battery_voltage=None,
                    battery_observation="never_published_operator_confirmed_off")
    fresh(battery, 1.5)
    if boolean(battery, "connected") or not 0 <= number(battery, "voltage_v") < 1:
        raise ValueError("Battery observation inconsistent with operator-confirmed power off")
    return dict(state=state, outputs=outputs, kill=kill, battery_voltage=number(battery, "voltage_v"))


def write_report(out, c, report):
    c.close()
    report["port_closed"] = not c.s.is_open
    (out / "console.json").write_text(json.dumps(c.records, ensure_ascii=False, indent=2), encoding="utf-8")
    report["console_sha256"] = hashlib.sha256((out / "console.json").read_bytes()).hexdigest()
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def run(port, output, *, operator_power_off=False, apply_poles=False, rounds=13, interval=5):
    if not operator_power_off:
        raise ValueError("Operator confirmation of ESC power off required")
    if not 1 <= rounds <= 25 or not 0 <= interval <= 10:
        raise ValueError("Bounded observation required")
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    c = ImuConsole(port, allow_pole_write=apply_poles)
    report = dict(kind="px4_stationary_imu_and_fixed_motor_poles", utc_started=datetime.now(timezone.utc).isoformat(),
                  firmware=FIRMWARE, complete=False, parameters={}, samples=[], status_text={},
                  operator_power_off=True, apply_poles=apply_poles, parameter_write_attempts=0,
                  save_attempts=0, motor_commands=0, calibration_commands=0, arm_commands=0,
                  nx_accessed=False, power_cycle_persistence_verified=False, flight_ready=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in clean(c.query("ver all")):
            raise ValueError("Unexpected firmware")
        report["before"] = guard(c, require_kill=apply_poles, allow_unpublished_battery=not apply_poles)
        for p in PARAMETERS:
            try:
                report["parameters"][p] = read_parameter(c, p)
            except ValueError:
                report["parameters"][p] = None
        for cmd in STATUS[1:]:
            report["status_text"][cmd] = clean(c.query(cmd))
        if apply_poles:
            if report["parameters"]["MOT_POLE_COUNT"] not in (14, 24):
                raise ValueError("Unexpected original pole count")
            report["prewrite"] = guard(c, require_kill=True)
            if report["parameters"]["MOT_POLE_COUNT"] == 14:
                report["parameter_write_attempts"] += 1
                report["write_response"] = clean(c.query("param set MOT_POLE_COUNT 24"))
            report["poles_readback"] = read_parameter(c, "MOT_POLE_COUNT")
            if report["poles_readback"] != 24:
                raise ValueError("Pole correction did not read back")
            guard(c, require_kill=True)
            report["save_attempts"] += 1
            report["save_response"] = clean(c.query("param save"))
            if re.search(r"\b(?:ERROR|FAIL|failed)\b", report["save_response"], re.I):
                raise ValueError("Parameter save reported an error")
            report["poles_after_save"] = read_parameter(c, "MOT_POLE_COUNT")
            if report["poles_after_save"] != 24:
                raise ValueError("Pole count changed after save")
        for i in range(rounds):
            sample = dict(index=i, pc_monotonic=time.monotonic(), raw={}, assessment={})
            selector = clean(c.query("listener estimator_selector_status -n 1"))
            sample["selector_before"] = selector
            multi = report["parameters"]["SENS_IMU_MODE"]
            if multi not in (0, 1):
                raise ValueError("Unknown IMU selection mode")
            primary = int(field(selector, "primary_instance")) if multi == 0 else 0
            if not 0 <= primary < 9:
                raise ValueError("Invalid primary estimator")
            if multi == 0:
                fresh(selector, 2)
            sample["primary_instance"] = primary
            instances = set((0, 1, 2, primary))
            for instance in sorted(instances):
                raw = clean(c.query(f"listener estimator_sensor_bias -i {instance} -n 1"))
                sample["raw"][str(instance)] = raw
                try:
                    sample["assessment"][str(instance)] = assess_bias(raw)
                except (ValueError, TypeError) as exc:
                    sample["assessment"][str(instance)] = dict(unknown=str(exc))
            for topic in ("sensor_accel", "vehicle_imu_status"):
                for instance in range(3):
                    sample["raw"][f"{topic}_{instance}"] = clean(c.query(f"listener {topic} -i {instance} -n 1"))
            sample["attitude"] = clean(c.query("listener vehicle_attitude -n 1"))
            sample["acceleration"] = clean(c.query("listener vehicle_acceleration -n 1"))
            sample["selector_after"] = clean(c.query("listener estimator_selector_status -n 1"))
            sample["primary_unchanged_during_reads"] = multi == 1 or int(field(sample["selector_after"], "primary_instance")) == primary
            report["samples"].append(sample)
            if i + 1 < rounds:
                time.sleep(interval)
        for topic in ("estimator_status", "estimator_status_flags"):
            for instance in range(3):
                cmd = f"listener {topic} -i {instance} -n 1"
                report["status_text"][cmd] = clean(c.query(cmd))
        report["after"] = guard(c, require_kill=apply_poles, allow_unpublished_battery=not apply_poles)
        if report["after"]["state"]["timestamp"] <= report["before"]["state"]["timestamp"]:
            raise ValueError("FC time did not advance")
        report["complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        write_report(out, c, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--esc-power-off", action="store_true")
    parser.add_argument("--apply-poles", action="store_true")
    parser.add_argument("--rounds", type=int, default=13)
    args = parser.parse_args()
    report = run(args.port, args.output, operator_power_off=args.esc_power_off,
                 apply_poles=args.apply_poles, rounds=args.rounds)
    print(json.dumps({k: report.get(k) for k in ("complete", "error", "poles_readback", "after", "port_closed")}, indent=2))
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
