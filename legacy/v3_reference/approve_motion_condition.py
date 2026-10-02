#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit(
        "Usage: approve_motion_condition.py TARGET_DEG SPEED_DEG_S"
    )

target = float(sys.argv[1])
speed = float(sys.argv[2])

path = Path("approved_motion_conditions.csv")

rows = []

if path.exists():
    with path.open() as f:
        rows = list(csv.DictReader(f))

key = (round(target, 6), round(speed, 6))

existing = {
    (
        round(float(r["target_deg"]), 6),
        round(float(r["speed_deg_s"]), 6),
    )
    for r in rows
}

if key not in existing:
    rows.append({
        "target_deg": target,
        "speed_deg_s": speed,
    })

with path.open("w", newline="") as f:
    w = csv.DictWriter(
        f,
        fieldnames=["target_deg", "speed_deg_s"],
    )
    w.writeheader()
    w.writerows(rows)

print(
    f"[ OK ] approved target={target:g} deg, "
    f"speed={speed:g} deg/s"
)
