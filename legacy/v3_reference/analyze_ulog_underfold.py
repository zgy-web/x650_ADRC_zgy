#!/usr/bin/env python3

import json
import math
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: analyze_ulog_underfold.py <baseline_dir>"
    )

folder = Path(sys.argv[1]).resolve()
ulog_path = folder / "px4.ulg"
result_path = folder / "result.json"

if not ulog_path.is_file():
    raise SystemExit(f"Missing: {ulog_path}")

if not result_path.is_file():
    raise SystemExit(f"Missing: {result_path}")

result = json.loads(result_path.read_text())
u = ULog(str(ulog_path))


def dataset(name, multi_id=0):
    matches = [
        d for d in u.data_list
        if d.name == name and d.multi_id == multi_id
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"{name}[{multi_id}] count={len(matches)}"
        )

    return matches[0]


def sim_event_exact(text):
    matches = [
        e for e in result["events"]
        if e["event"] == text
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"event {text!r}: count={len(matches)}"
        )

    value = matches[0]["sim_time_latest_truth"]

    if value is None:
        raise RuntimeError(
            f"event has no sim time: {text}"
        )

    return float(value)


def sim_event_prefix(prefix):
    matches = [
        e for e in result["events"]
        if e["event"].startswith(prefix)
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"event prefix {prefix!r}: count={len(matches)}"
        )

    value = matches[0]["sim_time_latest_truth"]

    if value is None:
        raise RuntimeError(
            f"event has no sim time: {prefix}"
        )

    return float(value)


windows = {
    "pre": (
        sim_event_exact(
            "Pre-baseline stabilization hold"
        ),
        sim_event_prefix(
            "Pre-baseline stabilization hold finished:"
        ),
    ),
    "baseline": (
        sim_event_exact(
            "STATIONARY UNDERFOLD BASELINE INTERVAL BEGIN"
        ),
        sim_event_exact(
            "STATIONARY UNDERFOLD BASELINE INTERVAL END"
        ),
    ),
    "post": (
        sim_event_exact(
            "Post-baseline stabilization hold"
        ),
        sim_event_prefix(
            "Post-baseline stabilization hold finished:"
        ),
    ),
}


def timestamp_s(d):
    return (
        np.asarray(d.data["timestamp"], dtype=np.float64)
        / 1e6
    )


def finite(a):
    a = np.asarray(a, dtype=np.float64)
    return a[np.isfinite(a)]


def basic_stats(a):
    a = finite(a)

    if len(a) == 0:
        return {"count": 0}

    return {
        "count": int(len(a)),
        "mean": float(np.mean(a)),
        "min": float(np.min(a)),
        "max": float(np.max(a)),
        "p99": float(np.percentile(a, 99)),
    }


def mask_window(t, bounds):
    lo, hi = bounds
    return (t >= lo) & (t <= hi)


sensor = dataset("sensor_combined")
est = dataset("estimator_status")
motors = dataset("actuator_motors")
outputs = dataset("actuator_outputs")
vimu_status = dataset("vehicle_imu_status")
vlp = dataset("vehicle_local_position")

ts_sensor = timestamp_s(sensor)
ts_est = timestamp_s(est)
ts_motors = timestamp_s(motors)
ts_outputs = timestamp_s(outputs)


print("============================================================")
print(" ULOG TIME RANGES")
print("============================================================")

for name, ts in (
    ("sensor_combined", ts_sensor),
    ("estimator_status", ts_est),
    ("actuator_motors", ts_motors),
    ("actuator_outputs", ts_outputs),
):
    print(
        f"{name:22s}: "
        f"{ts[0]:.3f} -> {ts[-1]:.3f} s "
        f"(n={len(ts)})"
    )

print()
print("Requested Gazebo simulation-time windows:")

for name, (lo, hi) in windows.items():
    print(
        f"{name:10s}: {lo:.3f} -> {hi:.3f} s "
        f"({hi-lo:.3f} s)"
    )


# Require all three requested windows to lie inside the high-rate
# sensor timestamp range. This prevents silently mixing clocks.
all_lo = min(v[0] for v in windows.values())
all_hi = max(v[1] for v in windows.values())

time_aligned = (
    ts_sensor[0] <= all_lo <= ts_sensor[-1]
    and
    ts_sensor[0] <= all_hi <= ts_sensor[-1]
)

print()
print(
    "SIM-TIME / ULOG TIMESTAMP ALIGNMENT:",
    "PASS" if time_aligned else "FAIL"
)

if not time_aligned:
    print(
        "Window statistics will NOT be treated as valid."
    )


print()
print("============================================================")
print(" IMU TIMING")
print("============================================================")

acc_dt = np.asarray(
    sensor.data["accelerometer_integral_dt"],
    dtype=np.float64,
)

gyro_dt = np.asarray(
    sensor.data["gyro_integral_dt"],
    dtype=np.float64,
)

stamp_dt = np.diff(
    np.asarray(sensor.data["timestamp"], dtype=np.int64)
).astype(np.float64)

for label, values in (
    ("accelerometer_integral_dt_us", acc_dt),
    ("gyro_integral_dt_us", gyro_dt),
    ("sensor_combined_timestamp_dt_us", stamp_dt),
):
    s = basic_stats(values)

    exact_4000 = int(
        np.count_nonzero(
            np.abs(values - 4000.0) < 0.5
        )
    )

    outside = int(
        np.count_nonzero(
            (values < 3500.0)
            |
            (values > 4500.0)
        )
    )

    print()
    print(label)
    print(
        f"  n={s['count']} "
        f"mean={s['mean']:.3f} us "
        f"min={s['min']:.3f} us "
        f"max={s['max']:.3f} us "
        f"p99={s['p99']:.3f} us"
    )
    print(
        f"  exactly_4000us={exact_4000}"
    )
    print(
        f"  outside_[3500,4500]us={outside}"
    )


print()
print("============================================================")
print(" ESTIMATOR TIMING / FAULTS")
print("============================================================")

time_slip = np.asarray(
    est.data["time_slip"]
)

print(
    "time_slip:",
    f"n={len(time_slip)}",
    f"nonzero={int(np.count_nonzero(time_slip))}",
    f"min={int(np.min(time_slip))}",
    f"max={int(np.max(time_slip))}",
)

for field in (
    "filter_fault_flags",
    "timeout_flags",
):
    a = np.asarray(est.data[field])

    print(
        f"{field}: "
        f"nonzero_rows={int(np.count_nonzero(a))}, "
        f"max={int(np.max(a))}"
    )

for field in (
    "reset_count_pos_ne",
    "reset_count_pod_d",
    "reset_count_quat",
    "reset_count_vel_d",
    "reset_count_vel_ne",
):
    a = np.asarray(est.data[field])

    print(
        f"{field}: first={int(a[0])}, "
        f"last={int(a[-1])}, "
        f"change={int(a[-1])-int(a[0])}"
    )


print()
print("============================================================")
print(" IMU STATUS")
print("============================================================")

for field in (
    "accel_error_count",
    "gyro_error_count",
):
    a = np.asarray(vimu_status.data[field])

    print(
        f"{field}: "
        f"first={int(a[0])}, "
        f"last={int(a[-1])}, "
        f"max={int(np.max(a))}"
    )

for field in (
    "accel_rate_hz",
    "gyro_rate_hz",
    "accel_raw_rate_hz",
    "gyro_raw_rate_hz",
):
    s = basic_stats(vimu_status.data[field])

    print(
        f"{field}: "
        f"mean={s['mean']:.3f}, "
        f"min={s['min']:.3f}, "
        f"max={s['max']:.3f}"
    )


def motor_window(d, ts, bounds, prefix):
    mask = mask_window(ts, bounds)

    result = {}

    for i in range(4):
        key = f"{prefix}[{i}]"

        if key not in d.data:
            continue

        values = np.asarray(
            d.data[key],
            dtype=np.float64,
        )[mask]

        values = values[np.isfinite(values)]

        if len(values) == 0:
            result[str(i)] = {
                "count": 0
            }
            continue

        result[str(i)] = {
            "count": int(len(values)),
            "mean": float(np.mean(values)),
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "p99": float(
                np.percentile(values, 99)
            ),
        }

    return result


window_results = {}

if time_aligned:
    print()
    print("============================================================")
    print(" MOTOR OUTPUTS BY SIMULATION-TIME WINDOW")
    print("============================================================")

    for name, bounds in windows.items():
        print()
        print(
            f"----- {name.upper()} "
            f"{bounds[0]:.3f}->{bounds[1]:.3f}s -----"
        )

        mot = motor_window(
            motors,
            ts_motors,
            bounds,
            "control",
        )

        out = motor_window(
            outputs,
            ts_outputs,
            bounds,
            "output",
        )

        print("actuator_motors:")

        for i in range(4):
            s = mot.get(str(i), {})

            if not s or s.get("count", 0) == 0:
                print(f"  motor{i}: no samples")
                continue

            print(
                f"  motor{i}: "
                f"mean={s['mean']:.6f} "
                f"min={s['min']:.6f} "
                f"max={s['max']:.6f} "
                f"p99={s['p99']:.6f} "
                f"n={s['count']}"
            )

        print("actuator_outputs:")

        for i in range(4):
            s = out.get(str(i), {})

            if not s or s.get("count", 0) == 0:
                print(f"  output{i}: no samples")
                continue

            print(
                f"  output{i}: "
                f"mean={s['mean']:.6f} "
                f"min={s['min']:.6f} "
                f"max={s['max']:.6f} "
                f"p99={s['p99']:.6f} "
                f"n={s['count']}"
            )

        window_results[name] = {
            "actuator_motors": mot,
            "actuator_outputs": out,
        }


print()
print("============================================================")
print(" LOCAL POSITION RESET COUNTERS")
print("============================================================")

for field in (
    "xy_reset_counter",
    "z_reset_counter",
    "vxy_reset_counter",
    "vz_reset_counter",
    "heading_reset_counter",
):
    a = np.asarray(vlp.data[field])

    print(
        f"{field}: "
        f"first={int(a[0])}, "
        f"last={int(a[-1])}, "
        f"change={int(a[-1])-int(a[0])}"
    )


summary = {
    "ulog": str(ulog_path),
    "ulog_sha256": result.get("ulog_sha256"),
    "time_aligned_to_gazebo_sim_time": bool(time_aligned),

    "sim_windows": {
        k: {
            "start_s": v[0],
            "end_s": v[1],
            "duration_s": v[1] - v[0],
        }
        for k, v in windows.items()
    },

    "imu": {
        "accelerometer_integral_dt_us":
            basic_stats(acc_dt),
        "gyro_integral_dt_us":
            basic_stats(gyro_dt),
        "sensor_timestamp_dt_us":
            basic_stats(stamp_dt),

        "accel_exact_4000_count":
            int(np.count_nonzero(
                np.abs(acc_dt-4000) < 0.5
            )),

        "gyro_exact_4000_count":
            int(np.count_nonzero(
                np.abs(gyro_dt-4000) < 0.5
            )),

        "timestamp_outside_3500_4500_count":
            int(np.count_nonzero(
                (stamp_dt < 3500)
                |
                (stamp_dt > 4500)
            )),
    },

    "estimator": {
        "time_slip_nonzero_count":
            int(np.count_nonzero(time_slip)),
        "time_slip_min":
            int(np.min(time_slip)),
        "time_slip_max":
            int(np.max(time_slip)),
        "filter_fault_nonzero_rows":
            int(np.count_nonzero(
                est.data["filter_fault_flags"]
            )),
        "timeout_nonzero_rows":
            int(np.count_nonzero(
                est.data["timeout_flags"]
            )),
    },

    "motor_windows": window_results,
}

out_path = folder / "analysis_ulog_v1.json"

out_path.write_text(
    json.dumps(
        summary,
        indent=2,
    )
)

print()
print("Saved:", out_path)
