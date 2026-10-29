"""Offline six-face quality audit from the archived reports; never opens a port.

Run after populating the evidence directory. Console hashes can be verified
separately against local raw recordings; this tool recomputes the published
measurements and compares parameter snapshots without claiming bit-exact floats.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

from px4_imu_pole_check import assess_bias, vector
from px4_rc_switch_setup import field, number


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def limits(values):
    return [min(values), max(values)]


def observation(path):
    report = load(path)
    if not report["complete"] or not report["port_closed"] or not report["samples"]:
        raise ValueError("Incomplete observation: " + str(path))
    samples = report["samples"]
    result = dict(report_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  console_sha256=report["console_sha256"], rounds=len(samples),
                  sample_start_span_seconds=samples[-1]["pc_monotonic"] - samples[0]["pc_monotonic"],
                  primary_instances=sorted({s["primary_instance"] for s in samples}),
                  instances=[], final_state=report["after"])
    for i in range(3):
        estimates = [assess_bias(s["raw"][str(i)]) for s in samples]
        if any(e != s["assessment"][str(i)] for e, s in zip(estimates, samples)):
            raise ValueError("Stored assessment differs from the raw response")
        if len({e["accel_device_id"] for e in estimates}) != 1:
            raise ValueError("Sensor identity changed")
        acc = [e["accel"] for e in estimates]
        gyro = [e["gyro"] for e in estimates]
        if not all(e["valid"] for e in acc + gyro):
            raise ValueError("Invalid bias estimate")
        imu = [s["raw"][f"vehicle_imu_status_{i}"] for s in samples]
        if any(int(field(raw, "accel_device_id")) != estimates[0]["accel_device_id"] for raw in imu):
            raise ValueError("IMU status and estimate sensor IDs differ")
        result["instances"].append(dict(
            instance=i, accel_device_id=estimates[0]["accel_device_id"],
            accel_bias_range=[limits([a["bias"][k] for a in acc]) for k in range(3)],
            accel_threshold_range=[limits([a["thresholds"][k] for a in acc]) for k in range(3)],
            accel_exceed_count=[sum(a["exceeds"][k] for a in acc) for k in range(3)],
            accel_stable_count=sum(a["stable"] for a in acc),
            gyro_exceed_count=[sum(a["exceeds"][k] for a in gyro) for k in range(3)],
            accel_error_max=max(number(raw, "accel_error_count") for raw in imu),
            gyro_error_max=max(number(raw, "gyro_error_count") for raw in imu),
            accel_clipping_max=[max(vector(raw, "accel_clipping")[k] for raw in imu) for k in range(3)],
            gyro_clipping_max=[max(vector(raw, "gyro_clipping")[k] for raw in imu) for k in range(3)],
            temperature_accel_range=limits([number(raw, "temperature_accel") for raw in imu])))
    norms = []
    for s in samples:
        e = s["assessment"][str(s["primary_instance"])]
        if not s["primary_unchanged_during_reads"] or any(
                int(field(s[selector], "accel_device_id")) != e["accel_device_id"]
                for selector in ("selector_before", "selector_after")):
            raise ValueError("Cannot approximate pre-bias acceleration across sensor switch")
        accel = vector(s["acceleration"], "xyz")
        norms.append(math.sqrt(sum((x + b)**2 for x, b in zip(accel, e["accel"]["bias"]))))
    result["approx_accel_norm_before_runtime_bias_range"] = limits(norms)
    result["approx_norm_method"] = "norm(vehicle_acceleration.xyz + selected accel_bias); sequential reads and LPF, not synchronized raw samples"
    result["bias_threshold_precision"] = "0.75 * limit + 3 * sqrt(variance), using rounded NSH values"
    return result


def summarize(root):
    session = load(root / "attempt3_status.json")["report"]
    restore = load(root / "restore_report.json")
    reboot = load(root / "restored_reboot_report.json")
    calibrated = load(root / "calibrated_reboot_report.json")
    if hashlib.sha256((root / "attempt3_status.json").read_bytes()).hexdigest() != restore["session_sha256"]:
        raise ValueError("Recovery source session hash mismatch")
    if any(r["firmware"] != session["firmware"] for r in (restore, reboot, calibrated)):
        raise ValueError("Firmware mismatch across recovery records")
    if not session["calibration_success"] or not session["post_readback_complete"] or not restore["complete"]:
        raise ValueError("Missing completed capture/restoration")
    names = {f"CAL_ACC{i}_{axis}{kind}" for i in range(3) for axis in "XYZ" for kind in ("OFF", "SCALE")}
    if set(restore["target"]) != names or any(
            restore["target"][name] != session["before_parameters"][name] for name in names):
        raise ValueError("Recovery differs from pre-calibration snapshot")
    if calibrated["parameters"] != session["after_parameters"]:
        raise ValueError("Calibration was not retained at readback precision")
    if reboot["parameters"] != restore["after_parameters"] or reboot["parameters"] != session["before_parameters"]:
        raise ValueError("Restored parameters differ after operator-reported reboot")
    stream = (root / "attempt3_calibration_stream.txt").read_bytes()
    if hashlib.sha256(stream).hexdigest() != session["stream_sha256"]:
        raise ValueError("Calibration stream hash mismatch")
    axes = {"down": (2, -1), "up": (2, 1), "left": (1, 1), "right": (1, -1),
            "front": (0, -1), "back": (0, 1)}
    faces = {}
    for name, numbers in re.findall(r"\[cal\] (\w+) side result: \[([^\]]+)\]", stream.decode("ascii")):
        if name not in axes or name in faces:
            raise ValueError("Ambiguous face measurement")
        v = [float(x) for x in numbers.split()]
        axis, sign = axes[name]
        norm = math.sqrt(sum(x*x for x in v))
        angle = math.degrees(math.acos(max(-1, min(1, sign*v[axis]/norm))))
        faces[name] = dict(first_sensor_average=v, norm=norm, inferred_axis_deviation_deg=angle)
    if set(faces) != set(axes):
        raise ValueError("Six distinct measurements required")
    observations = {name: observation(root / filename) for name, filename in (
        ("pre_calibration_fresh_boot", "attempt2_post_restart_report.json"),
        ("post_calibration_warm", "calibrated_warm_report.json"),
        ("post_calibration_reboot", "calibrated_reboot_report.json"),
        ("restored_reboot", "restored_reboot_report.json"))}
    if not any(any(i["accel_exceed_count"]) for i in observations["post_calibration_reboot"]["instances"]):
        raise ValueError("This rejection audit requires the recorded post-calibration bias failure")
    restored = observations["restored_reboot"]
    return dict(
        firmware=session["firmware"], observations=observations, face_geometry=faces,
        face_geometry_method="Direction inferred from rounded first-sensor averages under static-gravity assumption; not an external angular measurement",
        capture_completed=True, new_calibration_quality_accepted=False,
        new_calibration_disposition="Rejected; restored 18 offset/scale values at recorded NSH precision",
        restored_parameter_persistence_verified=True,
        reboot_evidence="Operator confirmed physical reboot; subsequent serial parameter readback matches restoration snapshot",
        restored_accel_bias_gate_passed_in_observed_window=all(
            not any(i["accel_exceed_count"]) for i in restored["instances"]),
        restored_max_abs_accel_bias=max(abs(x) for i in restored["instances"] for bounds in i["accel_bias_range"] for x in bounds),
        motor_poles_after_reboot=reboot["parameters"]["MOT_POLE_COUNT"],
        original_long_duration_bias_root_cause_resolved=False, flight_ready=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = summarize(args.evidence)
    with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, allow_nan=False, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({k: result[k] for k in (
        "capture_completed", "new_calibration_quality_accepted", "restored_parameter_persistence_verified",
        "restored_accel_bias_gate_passed_in_observed_window", "restored_max_abs_accel_bias", "motor_poles_after_reboot")}, indent=2))
