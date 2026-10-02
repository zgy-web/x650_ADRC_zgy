#!/usr/bin/env python3

import json
import math
import sys
from pathlib import Path

import numpy as np


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: analyze_excitation_trajectory.py RESULT_DIR"
    )

folder = Path(sys.argv[1]).resolve()

motion = json.loads(
    (folder / "motion_v1.json").read_text()
)

result = json.loads(
    (folder / "result.json").read_text()
)

columns = motion["columns"]
samples = motion["samples"]

expected_columns = [
    "sim_s",
    "phase",
    "q1_rad",
    "dq1_rad_s",
    "q2_rad",
    "dq2_rad_s",
    "q1_cmd_rad_s",
    "q1_cmd_accel_rad_s2",
]

missing = [
    x for x in expected_columns
    if x not in columns
]

if missing:
    raise SystemExit(
        f"[FAIL] Missing columns: {missing}\n"
        f"Actual schema: {columns}"
    )

if not samples:
    raise SystemExit("[FAIL] No trajectory samples")

idx = {
    name: columns.index(name)
    for name in expected_columns
}

t = np.asarray(
    [r[idx["sim_s"]] for r in samples],
    dtype=float,
)

phase = np.asarray(
    [r[idx["phase"]] for r in samples],
    dtype=object,
)

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

dq2 = np.asarray(
    [r[idx["dq2_rad_s"]] for r in samples],
    dtype=float,
)

u = np.asarray(
    [r[idx["q1_cmd_rad_s"]] for r in samples],
    dtype=float,
)

a_cmd = np.asarray(
    [r[idx["q1_cmd_accel_rad_s2"]] for r in samples],
    dtype=float,
)


q1_deg = np.degrees(q1)
dq1_deg_s = np.degrees(dq1)

q2_deg = np.degrees(q2)
dq2_deg_s = np.degrees(dq2)

u_deg_s = np.degrees(u)
a_deg_s2 = np.degrees(a_cmd)


protocol = result["protocol"]

target = float(
    protocol["motion_target_deg"]
)

vmax = float(
    protocol["motion_max_speed_deg_s"]
)

amax = float(
    protocol["motion_max_accel_deg_s2"]
)


print("=" * 72)
print(" EXCITATION V2 TRAJECTORY ACCEPTANCE")
print("=" * 72)

print("schema =", columns)
print("samples =", len(samples))
print()

print(f"configured target = {target:.6f} deg")
print(f"configured vmax   = {vmax:.6f} deg/s")
print(f"configured amax   = {amax:.6f} deg/s^2")

print()
print("POSITION")
print(f"q1 min       = {q1_deg.min():.9f} deg")
print(f"q1 max       = {q1_deg.max():.9f} deg")
print(f"q1 final     = {q1_deg[-1]:.9f} deg")
print(f"q2 max |q|   = {np.max(np.abs(q2_deg)):.9f} deg")

print()
print("VELOCITY")
print(
    f"actual max |dq1| = "
    f"{np.max(np.abs(dq1_deg_s)):.9f} deg/s"
)
print(
    f"command max |u|  = "
    f"{np.max(np.abs(u_deg_s)):.9f} deg/s"
)

print()
print("ACCELERATION")
print(
    f"command max |a|  = "
    f"{np.max(np.abs(a_deg_s2)):.9f} deg/s^2"
)

near_v = (
    np.abs(u_deg_s)
    >= 0.95 * vmax
)

near_a = (
    np.abs(a_deg_s2)
    >= 0.95 * amax
)

print(
    "samples >=95% vmax =",
    int(np.count_nonzero(near_v)),
)

print(
    "samples >=95% amax =",
    int(np.count_nonzero(near_a)),
)


print()
print("=" * 72)
print(" PHASE SUMMARY")
print("=" * 72)

phase_summary = {}

for ph in dict.fromkeys(phase.tolist()):

    mask = phase == ph

    tt = t[mask]
    qq = q1_deg[mask]
    vv = dq1_deg_s[mask]
    uu = u_deg_s[mask]
    aa = a_deg_s2[mask]

    if len(tt) == 0:
        continue

    duration = (
        float(tt[-1] - tt[0])
        if len(tt) >= 2
        else 0.0
    )

    info = {
        "duration_s": duration,
        "q1_min_deg": float(np.min(qq)),
        "q1_max_deg": float(np.max(qq)),
        "actual_max_abs_dq1_deg_s":
            float(np.max(np.abs(vv))),
        "command_max_abs_velocity_deg_s":
            float(np.max(np.abs(uu))),
        "command_max_abs_accel_deg_s2":
            float(np.max(np.abs(aa))),
    }

    phase_summary[str(ph)] = info

    print(
        f"{str(ph):14s} "
        f"T={duration:7.3f}s   "
        f"q1=[{np.min(qq):8.4f},"
        f"{np.max(qq):8.4f}] deg   "
        f"|dq|={np.max(np.abs(vv)):7.4f}   "
        f"|u|={np.max(np.abs(uu)):7.4f}   "
        f"|a|={np.max(np.abs(aa)):7.4f}"
    )


# ------------------------------------------------------------
# Numerically verify that command-velocity increments agree
# with the recorded acceleration command.
# Do this only within continuous phases.
# ------------------------------------------------------------

accel_reconstruction_errors = []

for ph in dict.fromkeys(phase.tolist()):

    mask = phase == ph

    tt = t[mask]
    uu = u_deg_s[mask]
    aa = a_deg_s2[mask]

    if len(tt) < 3:
        continue

    dt = np.diff(tt)
    du = np.diff(uu)

    good = (
        (dt > 1e-6)
        &
        (dt < 0.2)
    )

    if not np.any(good):
        continue

    reconstructed = (
        du[good]
        /
        dt[good]
    )

    recorded = aa[1:][good]

    accel_reconstruction_errors.extend(
        np.abs(
            reconstructed - recorded
        ).tolist()
    )


print()
print("=" * 72)
print(" COMMAND CONSISTENCY")
print("=" * 72)

if accel_reconstruction_errors:

    err = np.asarray(
        accel_reconstruction_errors,
        dtype=float,
    )

    print(
        f"|du/dt - a_cmd| median = "
        f"{np.median(err):.9f} deg/s^2"
    )

    print(
        f"|du/dt - a_cmd| p95    = "
        f"{np.percentile(err,95):.9f} deg/s^2"
    )

else:
    print(
        "No valid samples for acceleration reconstruction."
    )


fails = []

if abs(q1_deg.min() - target) > 0.08:
    fails.append(
        "target tracking error too large: "
        f"{q1_deg.min():.6f} deg"
    )

if abs(q1_deg[-1]) > 0.05:
    fails.append(
        "return-to-zero error too large: "
        f"{q1_deg[-1]:.6f} deg"
    )

if np.max(np.abs(q2_deg)) > 0.05:
    fails.append(
        "unexpected q2 motion"
    )

if np.max(np.abs(u_deg_s)) > vmax + 0.01:
    fails.append(
        "command velocity exceeded configured vmax"
    )

if np.max(np.abs(a_deg_s2)) > amax + 0.01:
    fails.append(
        "command acceleration exceeded configured amax"
    )

if not np.any(near_v):
    fails.append(
        "trajectory never reached configured velocity"
    )

if not np.any(near_a):
    fails.append(
        "trajectory never exercised configured acceleration"
    )


summary = {
    "status":
        "PASS" if not fails else "FAIL",

    "result_dir":
        str(folder),

    "target_deg":
        target,

    "vmax_deg_s":
        vmax,

    "amax_deg_s2":
        amax,

    "samples":
        len(samples),

    "q1_min_deg":
        float(q1_deg.min()),

    "q1_final_deg":
        float(q1_deg[-1]),

    "q2_max_abs_deg":
        float(np.max(np.abs(q2_deg))),

    "actual_max_abs_dq1_deg_s":
        float(np.max(np.abs(dq1_deg_s))),

    "command_max_abs_velocity_deg_s":
        float(np.max(np.abs(u_deg_s))),

    "command_max_abs_accel_deg_s2":
        float(np.max(np.abs(a_deg_s2))),

    "samples_near_vmax":
        int(np.count_nonzero(near_v)),

    "samples_near_amax":
        int(np.count_nonzero(near_a)),

    "phases":
        phase_summary,

    "failures":
        fails,
}


out = (
    folder
    /
    "analysis_excitation_trajectory_v2.json"
)

out.write_text(
    json.dumps(
        summary,
        indent=2,
    )
)


print()
print("=" * 72)

if fails:
    print("[FAIL] Excitation V2 trajectory")
    for f in fails:
        print(" -", f)
    raise SystemExit(1)

print("[PASS] Excitation V2 trajectory accepted")
print("Saved:", out)
