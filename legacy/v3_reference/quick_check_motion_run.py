#!/usr/bin/env python3

import json
import math
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


if len(sys.argv) != 4:
    raise SystemExit(
        "Usage: quick_check_motion_run.py "
        "RESULT_DIR EXPECTED_TARGET_DEG EXPECTED_SPEED_DEG_S"
    )

folder = Path(sys.argv[1]).resolve()
expected_target = float(sys.argv[2])
expected_speed = float(sys.argv[3])

result = json.loads(
    (folder / "result.json").read_text()
)

motion = json.loads(
    (folder / "motion_v1.json").read_text()
)

samples = motion["samples"]

if not samples:
    raise SystemExit("[FAIL] motion_v1.json has no samples")

q1 = np.asarray([r[2] for r in samples], dtype=float)
dq1 = np.asarray([r[3] for r in samples], dtype=float)
q2 = np.asarray([r[4] for r in samples], dtype=float)

q1_min_deg = math.degrees(float(np.min(q1)))
q1_final_deg = math.degrees(float(q1[-1]))
q2_max_deg = math.degrees(float(np.max(np.abs(q2))))
dq1_max = float(np.max(np.abs(dq1)))

failures = []
warnings = []

if not result.get("success"):
    failures.append("result.success != true")

if result.get("experiment") != "motion_underfold_v1":
    failures.append("wrong experiment type")

protocol = result.get("protocol", {})

if abs(float(protocol.get("motion_target_deg", 999)) - expected_target) > 1e-6:
    failures.append("protocol target mismatch")

if abs(float(protocol.get("motion_max_speed_deg_s", 999)) - expected_speed) > 1e-6:
    failures.append("protocol speed mismatch")

if abs(q1_min_deg - expected_target) > 0.08:
    failures.append(
        f"target tracking error too large: "
        f"q1_min={q1_min_deg:.4f} deg"
    )

if abs(q1_final_deg) > 0.05:
    failures.append(
        f"final zero error too large: "
        f"{q1_final_deg:.4f} deg"
    )

if q2_max_deg > 0.05:
    failures.append(
        f"q2 moved too far: {q2_max_deg:.4f} deg"
    )

expected_speed_rad = math.radians(expected_speed)

if dq1_max > 1.15 * expected_speed_rad + 1e-4:
    failures.append(
        f"q1 velocity too high: {dq1_max:.6f} rad/s"
    )


u = ULog(str(folder / "px4.ulg"))


def ds(name):
    m = [
        d for d in u.data_list
        if d.name == name and d.multi_id == 0
    ]

    if len(m) != 1:
        raise RuntimeError(f"{name}: count={len(m)}")

    return m[0]


def event_prefix(prefix):
    m = [
        e for e in result["events"]
        if e["event"].startswith(prefix)
    ]

    if len(m) != 1:
        raise RuntimeError(
            f"event prefix {prefix!r}: count={len(m)}"
        )

    return float(m[0]["sim_time_latest_truth"])


pre_lo = event_prefix(
    "Pre-motion stationary control hold"
)

pre_hi = event_prefix(
    "Pre-motion stationary control hold finished:"
)

hold_lo = event_prefix(
    "MOTION TARGET REACHED:"
)

hold_hi = event_prefix(
    "MOTION RETURN BEGIN:"
)


motors = ds("actuator_motors")

tm = (
    np.asarray(motors.data["timestamp"], dtype=float)
    / 1e6
)


def motor_means(lo, hi):
    mask = (tm >= lo) & (tm <= hi)

    values = []

    for i in range(4):
        x = np.asarray(
            motors.data[f"control[{i}]"],
            dtype=float,
        )[mask]

        x = x[np.isfinite(x)]

        if not len(x):
            raise RuntimeError(
                f"no actuator_motors data for motor {i}"
            )

        values.append(float(np.mean(x)))

    return values


pre = motor_means(pre_lo, pre_hi)
hold = motor_means(hold_lo, hold_hi)

D_pre = (
    (pre[0] + pre[2]) / 2
    -
    (pre[1] + pre[3]) / 2
)

D_hold = (
    (hold[0] + hold[2]) / 2
    -
    (hold[1] + hold[3]) / 2
)

pair_delta_hold = D_hold - D_pre

motor_max = max(
    float(
        np.nanmax(
            np.asarray(
                motors.data[f"control[{i}]"],
                dtype=float,
            )
        )
    )
    for i in range(4)
)

if motor_max >= 0.98:
    failures.append(
        f"normalized motor command too high: {motor_max:.4f}"
    )


est = ds("estimator_status")

fault_rows = int(
    np.count_nonzero(
        est.data["filter_fault_flags"]
    )
)

timeout_rows = int(
    np.count_nonzero(
        est.data["timeout_flags"]
    )
)

time_slip_max = float(
    np.max(
        np.asarray(
            est.data["time_slip"],
            dtype=float,
        )
    )
)

if fault_rows:
    failures.append(
        f"estimator filter fault rows={fault_rows}"
    )

if timeout_rows:
    failures.append(
        f"estimator timeout rows={timeout_rows}"
    )

if time_slip_max > 0.05:
    failures.append(
        f"time_slip too large: {time_slip_max:.6f}s"
    )


sensor = ds("sensor_combined")

acc_dt = np.asarray(
    sensor.data["accelerometer_integral_dt"],
    dtype=float,
)

gyro_dt = np.asarray(
    sensor.data["gyro_integral_dt"],
    dtype=float,
)

acc_bad = int(
    np.count_nonzero(
        (acc_dt < 3500)
        |
        (acc_dt > 4500)
    )
)

gyro_bad = int(
    np.count_nonzero(
        (gyro_dt < 3500)
        |
        (gyro_dt > 4500)
    )
)

max_imu_dt = max(
    float(np.max(acc_dt)),
    float(np.max(gyro_dt)),
)

if max_imu_dt > 20000:
    failures.append(
        f"IMU integral dt reached {max_imu_dt:.0f} us"
    )

if acc_bad or gyro_bad:
    warnings.append(
        f"IMU integral timing anomalies: "
        f"acc={acc_bad}, gyro={gyro_bad}"
    )


summary = {
    "status": "PASS" if not failures else "FAIL",
    "result_dir": str(folder),

    "expected_target_deg": expected_target,
    "expected_speed_deg_s": expected_speed,

    "q1_min_deg": q1_min_deg,
    "q1_final_deg": q1_final_deg,
    "q2_max_abs_deg": q2_max_deg,
    "q1_max_abs_velocity_rad_s": dq1_max,

    "motor_pre_means": pre,
    "motor_hold_means": hold,
    "pair_diff_delta_hold": pair_delta_hold,
    "max_normalized_motor_command": motor_max,

    "filter_fault_rows": fault_rows,
    "timeout_rows": timeout_rows,
    "time_slip_max_s": time_slip_max,

    "acc_integral_dt_anomalies": acc_bad,
    "gyro_integral_dt_anomalies": gyro_bad,

    "warnings": warnings,
    "failures": failures,
}

(folder / "quick_health.json").write_text(
    json.dumps(summary, indent=2)
)

print(json.dumps(summary, indent=2))

if failures:
    raise SystemExit(1)
