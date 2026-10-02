#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog

if len(sys.argv) != 2:
    raise SystemExit("Usage: analyze_motion_ulog.py <motion_dir>")

folder = Path(sys.argv[1]).resolve()
result = json.loads((folder / "result.json").read_text())
u = ULog(str(folder / "px4.ulg"))

def ds(name):
    m = [d for d in u.data_list if d.name == name and d.multi_id == 0]
    if len(m) != 1:
        raise RuntimeError(f"{name}: count={len(m)}")
    return m[0]

def exact(text):
    m = [e for e in result["events"] if e["event"] == text]
    if len(m) != 1:
        raise RuntimeError(text)
    return float(m[0]["sim_time_latest_truth"])

def prefix(text):
    m = [e for e in result["events"] if e["event"].startswith(text)]
    if len(m) != 1:
        raise RuntimeError(text)
    return float(m[0]["sim_time_latest_truth"])

windows = {
    "pre": (
        exact("Pre-motion stationary control hold"),
        prefix("Pre-motion stationary control hold finished:"),
    ),
    "outbound": (
        exact("MOTION BEGIN: q1 0deg -> -5deg -> 0deg"),
        exact("MOTION TARGET REACHED: q1=-5deg"),
    ),
    "target_hold": (
        exact("MOTION TARGET REACHED: q1=-5deg"),
        exact("MOTION RETURN BEGIN: q1 -5deg -> 0deg"),
    ),
    "return": (
        exact("MOTION RETURN BEGIN: q1 -5deg -> 0deg"),
        exact("MOTION END: returned to reviewed q1=0 pose"),
    ),
    "post": (
        exact("Post-motion stabilization hold"),
        prefix("Post-motion stabilization hold finished:"),
    ),
}

motors = ds("actuator_motors")
tm = np.asarray(motors.data["timestamp"], dtype=float) / 1e6

print("============================================================")
print(" MOTION V1 MOTOR RESPONSE")
print("============================================================")

pre_means = None

for name, (lo, hi) in windows.items():
    mask = (tm >= lo) & (tm <= hi)

    means = []
    maxima = []

    for i in range(4):
        x = np.asarray(motors.data[f"control[{i}]"], dtype=float)[mask]
        x = x[np.isfinite(x)]

        means.append(float(np.mean(x)))
        maxima.append(float(np.max(x)))

    if name == "pre":
        pre_means = means

    print()
    print(f"[{name.upper()}] {lo:.3f}->{hi:.3f}s")
    print(
        "  means = "
        + " ".join(f"{v:.6f}" for v in means)
    )
    print(
        "  max   = "
        + " ".join(f"{v:.6f}" for v in maxima)
    )

    if pre_means is not None:
        delta = [
            means[i] - pre_means[i]
            for i in range(4)
        ]

        print(
            "  delta from PRE mean = "
            + " ".join(f"{v:+.6f}" for v in delta)
        )

print()
print("============================================================")
print(" TIMING")
print("============================================================")

sensor = ds("sensor_combined")

dt_acc = np.asarray(
    sensor.data["accelerometer_integral_dt"],
    dtype=float,
)

dt_gyro = np.asarray(
    sensor.data["gyro_integral_dt"],
    dtype=float,
)

ts = np.asarray(sensor.data["timestamp"], dtype=np.int64)
stamp_dt = np.diff(ts).astype(float)

print(
    "acc integral-dt anomalies:",
    int(np.count_nonzero((dt_acc < 3500) | (dt_acc > 4500)))
)

print(
    "gyro integral-dt anomalies:",
    int(np.count_nonzero((dt_gyro < 3500) | (dt_gyro > 4500)))
)

print(
    "sensor timestamp gaps:",
    int(np.count_nonzero((stamp_dt < 3500) | (stamp_dt > 4500)))
)

est = ds("estimator_status")
slip = np.asarray(est.data["time_slip"], dtype=float)

print("time_slip max:", float(np.max(slip)))
print(
    "filter_fault rows:",
    int(np.count_nonzero(est.data["filter_fault_flags"]))
)
print(
    "timeout rows:",
    int(np.count_nonzero(est.data["timeout_flags"]))
)
