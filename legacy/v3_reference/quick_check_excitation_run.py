#!/usr/bin/env python3

import json
import math
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: quick_check_excitation_run.py RESULT_DIR"
    )

folder = Path(sys.argv[1]).resolve()

result = json.loads(
    (folder / "result.json").read_text()
)

motion = json.loads(
    (folder / "motion_v1.json").read_text()
)

failures = []
warnings = []

if not result.get("success"):
    failures.append("result.success != true")

if result.get("experiment") != "excitation_underfold_v2":
    failures.append(
        "experiment != excitation_underfold_v2"
    )


columns = motion["columns"]
samples = motion["samples"]

required = [
    "q1_rad",
    "dq1_rad_s",
    "q2_rad",
    "q1_cmd_rad_s",
    "q1_cmd_accel_rad_s2",
]

missing = [x for x in required if x not in columns]

if missing:
    failures.append(
        f"missing trajectory columns: {missing}"
    )
else:
    idx = {x: columns.index(x) for x in columns}

    q1 = np.asarray(
        [r[idx["q1_rad"]] for r in samples],
        dtype=float,
    )

    dq1 = np.asarray(
        [r[idx["dq1_rad_s"]] for r in samples],
        dtype=float,
    )

    q2 = np.asarray(
        [r[idx["q2_rad"]] for r in samples],
        dtype=float,
    )

    u = np.asarray(
        [r[idx["q1_cmd_rad_s"]] for r in samples],
        dtype=float,
    )

    a = np.asarray(
        [r[idx["q1_cmd_accel_rad_s2"]] for r in samples],
        dtype=float,
    )

    proto = result["protocol"]

    target = float(proto["motion_target_deg"])
    vmax = float(proto["motion_max_speed_deg_s"])
    amax = float(proto["motion_max_accel_deg_s2"])

    q1d = np.degrees(q1)
    q2d = np.degrees(q2)
    ud = np.degrees(u)
    ad = np.degrees(a)

    if abs(q1d.min() - target) > 0.08:
        failures.append(
            f"target error: {q1d.min():.5f} deg"
        )

    if abs(q1d[-1]) > 0.05:
        failures.append(
            f"return error: {q1d[-1]:.5f} deg"
        )

    if np.max(np.abs(q2d)) > 0.05:
        failures.append(
            f"q2 moved: {np.max(np.abs(q2d)):.5f} deg"
        )

    if np.max(np.abs(ud)) > vmax + 0.01:
        failures.append(
            "command velocity exceeded vmax"
        )

    if np.max(np.abs(ad)) > amax + 0.01:
        failures.append(
            "command acceleration exceeded amax"
        )


ulog = ULog(str(folder / "px4.ulg"))


def dataset(name):
    found = [
        d for d in ulog.data_list
        if d.name == name and d.multi_id == 0
    ]

    if len(found) != 1:
        raise RuntimeError(
            f"{name}: count={len(found)}"
        )

    return found[0]


motors = dataset("actuator_motors")

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
        f"motor command too high: {motor_max:.5f}"
    )


est = dataset("estimator_status")

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
        f"filter_fault_rows={fault_rows}"
    )

if timeout_rows:
    failures.append(
        f"timeout_rows={timeout_rows}"
    )

if time_slip_max > 0.05:
    failures.append(
        f"time_slip too large: {time_slip_max:.6f}s"
    )


sensor = dataset("sensor_combined")

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
        (acc_dt < 3500) | (acc_dt > 4500)
    )
)

gyro_bad = int(
    np.count_nonzero(
        (gyro_dt < 3500) | (gyro_dt > 4500)
    )
)

if acc_bad or gyro_bad:
    warnings.append(
        f"IMU timing anomalies: "
        f"acc={acc_bad}, gyro={gyro_bad}"
    )

if max(float(np.max(acc_dt)), float(np.max(gyro_dt))) > 20000:
    failures.append(
        "IMU integral dt > 20 ms"
    )


summary = {
    "status": "PASS" if not failures else "FAIL",
    "result_dir": str(folder),
    "motor_max": motor_max,
    "time_slip_max_s": time_slip_max,
    "filter_fault_rows": fault_rows,
    "timeout_rows": timeout_rows,
    "acc_integral_dt_anomalies": acc_bad,
    "gyro_integral_dt_anomalies": gyro_bad,
    "warnings": warnings,
    "failures": failures,
}

(folder / "quick_health_excitation_v2.json").write_text(
    json.dumps(summary, indent=2)
)

print(json.dumps(summary, indent=2))

if failures:
    raise SystemExit(1)
