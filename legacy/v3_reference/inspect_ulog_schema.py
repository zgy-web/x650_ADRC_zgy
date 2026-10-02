#!/usr/bin/env python3

from pathlib import Path
from pyulog import ULog

ulog = Path(
    "/root/autodl-tmp/px4_baseline_bags/"
    "baseline_underfold_20261001_173458_6909/px4.ulg"
)

print("ULog:", ulog)
print("Size:", ulog.stat().st_size)

u = ULog(str(ulog))

keywords = (
    "imu",
    "sensor_combined",
    "actuator",
    "motor",
    "vehicle_attitude",
    "vehicle_local_position",
    "estimator",
)

matches = []

for d in u.data_list:
    if any(k in d.name.lower() for k in keywords):
        matches.append(d)

print()
print("========== RELEVANT DATASETS ==========")

for d in sorted(
    matches,
    key=lambda x: (x.name, x.multi_id)
):
    timestamp = d.data.get("timestamp")

    rows = (
        len(timestamp)
        if timestamp is not None
        else 0
    )

    print(
        f"{d.name}  "
        f"multi_id={d.multi_id}  "
        f"rows={rows}"
    )

print()
print("========== RELEVANT FIELDS ==========")

for d in sorted(
    matches,
    key=lambda x: (x.name, x.multi_id)
):
    print()
    print(
        f"[{d.name}] "
        f"multi_id={d.multi_id}"
    )

    print(
        ", ".join(
            sorted(d.data.keys())
        )
    )

print()
print("========== DONE ==========")
