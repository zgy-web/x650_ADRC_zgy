"""Reproduce the stationary-bias summary from saved PC reports, without hardware."""
import argparse
import hashlib
import json
from pathlib import Path

from px4_imu_pole_check import READS, WRITES, assess_bias, vector
from px4_rc_switch_setup import number


def summarize(path):
    path = Path(path)
    report = json.loads(path.read_text(encoding="utf-8"))
    if not report["complete"] or len(report["samples"]) < 2:
        raise ValueError("Need a complete repeated observation")
    console = path.with_name("console.json")
    if hashlib.sha256(console.read_bytes()).hexdigest() != report["console_sha256"]:
        raise ValueError("Console/report hash mismatch")
    commands = [r["command"] for r in json.loads(console.read_text(encoding="utf-8"))]
    if any(c not in READS | WRITES | {None, "<enter>"} for c in commands):
        raise ValueError("Command outside reviewed scope in console history")
    mutations = [c for c in commands if c and (c.startswith("param set ") or c == "param save")]
    if mutations != ["param set MOT_POLE_COUNT 24", "param save"]:
        raise ValueError("Unexpected parameter-write history")
    samples = report["samples"]
    result = dict(report_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  console_sha256=report["console_sha256"], sample_rounds=len(samples),
                  sample_start_span_seconds=samples[-1]["pc_monotonic"] - samples[0]["pc_monotonic"],
                  primary_instances=sorted(set(s["primary_instance"] for s in samples)),
                  primary_stable_in_each_round=all(s["primary_unchanged_during_reads"] for s in samples),
                  parameters=report["parameters"], instances=[], writes=mutations,
                  pole_before=report["parameters"]["MOT_POLE_COUNT"], pole_after=report["poles_after_save"],
                  final_state=report["after"], power_cycle_persistence_verified=False,
                  rpm_tachometer_calibration_verified=False, imu_calibration_performed=False,
                  esc_crc_root_cause_resolved=False, esc_current_voltage_calibrated=False,
                  primary_accel_exceed_rounds=sum(any(s["assessment"][str(s["primary_instance"])]["accel"]["exceeds"])
                                                  for s in samples),
                  warning_text_reemitted=any("Preflight Fail: High Accelerometer Bias" in r["text"]
                                            for r in json.loads(console.read_text(encoding="utf-8"))),
                  threshold_precision="Approximate: calculated from rounded NSH bias/variance fields",
                  flight_ready=False)
    for i in range(3):
        values = [assess_bias(s["raw"][str(i)]) for s in samples]
        if any(v != s["assessment"][str(i)] for s, v in zip(samples, values)):
            raise ValueError("Stored assessment differs from recomputation")
        if len(set(v["accel_device_id"] for v in values)) != 1:
            raise ValueError("Sensor identity changed")
        accel = [v["accel"] for v in values]
        gyro = [v["gyro"] for v in values]
        if not all(v["valid"] for v in accel + gyro):
            raise ValueError("Invalid bias estimate")
        imu = [s["raw"][f"vehicle_imu_status_{i}"] for s in samples]
        result["instances"].append(dict(
            instance=i, accel_device_id=values[0]["accel_device_id"],
            first_last_bias_timestamp_us=[values[0]["timestamp"], values[-1]["timestamp"]],
            accel_bias_range=[[min(v["bias"][k] for v in accel), max(v["bias"][k] for v in accel)] for k in range(3)],
            accel_threshold_range=[[min(v["thresholds"][k] for v in accel), max(v["thresholds"][k] for v in accel)] for k in range(3)],
            accel_exceed_count=[sum(v["exceeds"][k] for v in accel) for k in range(3)],
            gyro_exceed_count=[sum(v["exceeds"][k] for v in gyro) for k in range(3)],
            max_accel_error_count=max(number(s, "accel_error_count") for s in imu),
            max_gyro_error_count=max(number(s, "gyro_error_count") for s in imu),
            max_accel_clipping=[max(vector(s, "accel_clipping")[k] for s in imu) for k in range(3)],
            max_gyro_clipping=[max(vector(s, "gyro_clipping")[k] for s in imu) for k in range(3)],
            accel_temperature_range=[min(number(s, "temperature_accel") for s in imu), max(number(s, "temperature_accel") for s in imu)],
        ))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.report)
    with args.output.open("x", encoding="utf-8", newline="\n") as f:
        json.dump(result, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write("\n")
    print(json.dumps({k: result[k] for k in ("sample_rounds", "sample_start_span_seconds", "primary_instances", "pole_before", "pole_after")}, indent=2))
