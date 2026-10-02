#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: locate_ulog_anomalies.py <baseline_dir>"
    )

folder = Path(sys.argv[1]).resolve()

result = json.loads(
    (folder / "result.json").read_text()
)

u = ULog(str(folder / "px4.ulg"))


def ds(name):
    m = [
        d for d in u.data_list
        if d.name == name and d.multi_id == 0
    ]

    if len(m) != 1:
        raise RuntimeError(
            f"{name}: count={len(m)}"
        )

    return m[0]


def event_exact(text):
    m = [
        e for e in result["events"]
        if e["event"] == text
    ]

    if len(m) != 1:
        raise RuntimeError(text)

    return float(
        m[0]["sim_time_latest_truth"]
    )


def event_prefix(prefix):
    m = [
        e for e in result["events"]
        if e["event"].startswith(prefix)
    ]

    if len(m) != 1:
        raise RuntimeError(prefix)

    return float(
        m[0]["sim_time_latest_truth"]
    )


windows = {
    "pre": (
        event_exact(
            "Pre-baseline stabilization hold"
        ),
        event_prefix(
            "Pre-baseline stabilization hold finished:"
        ),
    ),
    "baseline": (
        event_exact(
            "STATIONARY UNDERFOLD BASELINE INTERVAL BEGIN"
        ),
        event_exact(
            "STATIONARY UNDERFOLD BASELINE INTERVAL END"
        ),
    ),
    "post": (
        event_exact(
            "Post-baseline stabilization hold"
        ),
        event_prefix(
            "Post-baseline stabilization hold finished:"
        ),
    ),
}


def classify(t):
    for name, (lo, hi) in windows.items():
        if lo <= t <= hi:
            return name

    return "outside"


sensor = ds("sensor_combined")

t = np.asarray(
    sensor.data["timestamp"],
    dtype=np.float64,
) / 1e6

acc_dt = np.asarray(
    sensor.data["accelerometer_integral_dt"],
    dtype=np.float64,
)

gyro_dt = np.asarray(
    sensor.data["gyro_integral_dt"],
    dtype=np.float64,
)

stamp_dt = np.diff(
    np.asarray(
        sensor.data["timestamp"],
        dtype=np.int64,
    )
).astype(np.float64)


print("==================================================")
print("IMU INTEGRAL-DT ANOMALIES")
print("==================================================")

for label, values in (
    ("accelerometer", acc_dt),
    ("gyro", gyro_dt),
):
    idx = np.where(
        (values < 3500)
        |
        (values > 4500)
    )[0]

    print()
    print(label, "count =", len(idx))

    for i in idx:
        print(
            f"  t={t[i]:.6f}s "
            f"dt={values[i]:.1f}us "
            f"window={classify(t[i])}"
        )


print()
print("==================================================")
print("SENSOR TIMESTAMP-GAP ANOMALIES")
print("==================================================")

idx = np.where(
    (stamp_dt < 3500)
    |
    (stamp_dt > 4500)
)[0]

print("count =", len(idx))

for i in idx:
    print(
        f"  t={t[i]:.6f}s -> {t[i+1]:.6f}s "
        f"dt={stamp_dt[i]:.1f}us "
        f"window={classify(t[i])}"
    )


print()
print("==================================================")
print("TIME_SLIP PRECISE VALUES")
print("==================================================")

est = ds("estimator_status")

te = np.asarray(
    est.data["timestamp"],
    dtype=np.float64,
) / 1e6

slip = np.asarray(
    est.data["time_slip"],
    dtype=np.float64,
)

print("dtype:", slip.dtype)
print("n:", len(slip))
print(
    "min:",
    f"{np.min(slip):.12g}"
)
print(
    "max:",
    f"{np.max(slip):.12g}"
)
print(
    "mean:",
    f"{np.mean(slip):.12g}"
)

for tol in (
    0.0,
    1e-12,
    1e-9,
    1e-6,
    1e-3,
):
    print(
        f"|time_slip| > {tol:g}:",
        int(
            np.count_nonzero(
                np.abs(slip) > tol
            )
        )
    )

print()
print("by window:")

for name, (lo, hi) in windows.items():
    m = (
        (te >= lo)
        &
        (te <= hi)
    )

    x = slip[m]

    print(
        f"  {name}: n={len(x)} "
        f"min={np.min(x):.12g} "
        f"max={np.max(x):.12g} "
        f"mean={np.mean(x):.12g}"
    )


print()
print("==================================================")
print("RESET COUNTER TRANSITIONS")
print("==================================================")

vlp = ds("vehicle_local_position")

tv = np.asarray(
    vlp.data["timestamp"],
    dtype=np.float64,
) / 1e6

fields = (
    "xy_reset_counter",
    "z_reset_counter",
    "vxy_reset_counter",
    "vz_reset_counter",
    "heading_reset_counter",
)

for field in fields:

    a = np.asarray(
        vlp.data[field]
    )

    changes = np.where(
        a[1:] != a[:-1]
    )[0]

    print()
    print(
        field,
        "changes =",
        len(changes),
    )

    for i in changes:
        tc = tv[i+1]

        print(
            f"  t={tc:.6f}s "
            f"{int(a[i])}->{int(a[i+1])} "
            f"window={classify(tc)}"
        )


print()
print("==================================================")
print("WINDOW SUMMARY")
print("==================================================")

for name, bounds in windows.items():
    lo, hi = bounds

    sm = (
        (t >= lo)
        &
        (t <= hi)
    )

    acc_bad = int(
        np.count_nonzero(
            (
                acc_dt[sm] < 3500
            )
            |
            (
                acc_dt[sm] > 4500
            )
        )
    )

    gyro_bad = int(
        np.count_nonzero(
            (
                gyro_dt[sm] < 3500
            )
            |
            (
                gyro_dt[sm] > 4500
            )
        )
    )

    print(
        f"{name}: "
        f"acc_bad={acc_bad}, "
        f"gyro_bad={gyro_bad}"
    )
