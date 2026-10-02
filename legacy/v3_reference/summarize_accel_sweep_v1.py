#!/usr/bin/env python3

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


SWEEP = Path(
    "/root/autodl-tmp/px4_baseline_bags/"
    "sweeps/excitation_accel_explore_v1"
)

MANIFEST = SWEEP / "manifest.csv"
OUT_CSV = SWEEP / "accel_segment_summary_v1.csv"
OUT_JSON = SWEEP / "accel_segment_summary_v1.json"


def event_time(result, kind):
    events = result["events"]

    if kind == "pre_start":
        found = [
            e for e in events
            if e.get("event", "") ==
            "Pre-motion stationary control hold"
        ]

    elif kind == "pre_end":
        found = [
            e for e in events
            if e.get("event", "").startswith(
                "Pre-motion stationary control hold finished:"
            )
        ]

    else:
        raise ValueError(kind)

    if len(found) != 1:
        raise RuntimeError(
            f"{kind}: found {len(found)} matching events"
        )

    return float(
        found[0]["sim_time_latest_truth"]
    )


def get_ulog_dataset(ulog, name):
    found = [
        d for d in ulog.data_list
        if d.name == name and d.multi_id == 0
    ]

    if len(found) != 1:
        raise RuntimeError(
            f"{name}: found {len(found)} datasets"
        )

    return found[0]


def interval_from_mask(t, mask, name):
    ii = np.flatnonzero(mask)

    if len(ii) < 2:
        raise RuntimeError(
            f"Cannot identify segment {name}"
        )

    return (
        float(t[ii[0]]),
        float(t[ii[-1]]),
    )


def trajectory_stats(
    t, q_deg, dq_deg_s, u_deg_s, a_deg_s2,
    mask
):
    ii = np.flatnonzero(mask)

    if len(ii) == 0:
        raise RuntimeError(
            "empty trajectory segment"
        )

    return {
        "q_start_deg": float(q_deg[ii[0]]),
        "q_end_deg": float(q_deg[ii[-1]]),
        "mean_abs_dq_deg_s":
            float(np.mean(np.abs(dq_deg_s[ii]))),
        "mean_cmd_accel_deg_s2":
            float(np.mean(a_deg_s2[ii])),
        "max_abs_cmd_accel_deg_s2":
            float(np.max(np.abs(a_deg_s2[ii]))),
    }


with MANIFEST.open() as f:
    manifest = list(csv.DictReader(f))

runs = [
    r for r in manifest
    if r["status"] == "PASS"
]

if len(runs) != 4:
    raise SystemExit(
        f"[FAIL] Expected 4 PASS runs, found {len(runs)}"
    )


all_rows = []
run_summaries = []


for run in runs:

    run_id = run["run_id"]
    folder = Path(run["result_dir"])

    result = json.loads(
        (folder / "result.json").read_text()
    )

    motion = json.loads(
        (folder / "motion_v1.json").read_text()
    )

    columns = motion["columns"]
    samples = motion["samples"]

    idx = {
        name: columns.index(name)
        for name in columns
    }

    t = np.asarray(
        [r[idx["sim_s"]] for r in samples],
        dtype=float,
    )

    phase = np.asarray(
        [r[idx["phase"]] for r in samples],
        dtype=object,
    )

    q = np.degrees(
        np.asarray(
            [r[idx["q1_rad"]] for r in samples],
            dtype=float,
        )
    )

    dq = np.degrees(
        np.asarray(
            [r[idx["dq1_rad_s"]] for r in samples],
            dtype=float,
        )
    )

    u = np.degrees(
        np.asarray(
            [r[idx["q1_cmd_rad_s"]] for r in samples],
            dtype=float,
        )
    )

    a = np.degrees(
        np.asarray(
            [
                r[idx["q1_cmd_accel_rad_s2"]]
                for r in samples
            ],
            dtype=float,
        )
    )

    proto = result["protocol"]

    target = float(
        proto["motion_target_deg"]
    )

    vmax = float(
        proto["motion_max_speed_deg_s"]
    )

    amax = float(
        proto["motion_max_accel_deg_s2"]
    )

    direction = (
        1.0 if target > 0 else -1.0
    )

    # 10% threshold rejects tiny command corrections.
    athr = max(
        0.02,
        0.10 * amax,
    )

    outbound = phase == "outbound"
    ret = phase == "return"
    hold = phase == "target_hold"

    segments = {
        "out_accel":
            outbound
            &
            ((a * direction) > athr),

        "out_cruise":
            outbound
            &
            (np.abs(a) <= athr)
            &
            (np.abs(u) >= 0.95 * vmax),

        "out_decel":
            outbound
            &
            ((a * direction) < -athr),

        "target_hold":
            hold,

        "ret_accel":
            ret
            &
            ((a * direction) < -athr),

        "ret_cruise":
            ret
            &
            (np.abs(a) <= athr)
            &
            (np.abs(u) >= 0.95 * vmax),

        "ret_decel":
            ret
            &
            ((a * direction) > athr),
    }

    ulog = ULog(
        str(folder / "px4.ulg")
    )

    motors = get_ulog_dataset(
        ulog,
        "actuator_motors",
    )

    tm = (
        np.asarray(
            motors.data["timestamp"],
            dtype=float,
        )
        / 1e6
    )

    motor = np.column_stack([
        np.asarray(
            motors.data[f"control[{i}]"],
            dtype=float,
        )
        for i in range(4)
    ])

    D = (
        (motor[:, 0] + motor[:, 2]) / 2.0
        -
        (motor[:, 1] + motor[:, 3]) / 2.0
    )

    common = np.mean(
        motor,
        axis=1,
    )

    pre_start = event_time(
        result,
        "pre_start",
    )

    pre_end = event_time(
        result,
        "pre_end",
    )

    # Use last 5 seconds immediately before motion,
    # so the reference is temporally close to excitation.
    pre_tail_start = max(
        pre_start,
        pre_end - 5.0,
    )

    pre_mask = (
        (tm >= pre_tail_start)
        &
        (tm <= pre_end)
    )

    if np.count_nonzero(pre_mask) < 20:
        raise RuntimeError(
            f"{run_id}: insufficient PRE-tail motor data"
        )

    D_pre = float(
        np.mean(D[pre_mask])
    )

    common_pre = float(
        np.mean(common[pre_mask])
    )

    wide = {
        "run_id": run_id,
        "target_deg": target,
        "vmax_deg_s": vmax,
        "amax_deg_s2": amax,
        "D_pre_tail": D_pre,
        "common_pre_tail": common_pre,
    }


    for seg_name, seg_mask in segments.items():

        lo, hi = interval_from_mask(
            t,
            seg_mask,
            seg_name,
        )

        mm = (
            (tm >= lo)
            &
            (tm <= hi)
        )

        if np.count_nonzero(mm) < 5:
            raise RuntimeError(
                f"{run_id}/{seg_name}: "
                "insufficient motor samples"
            )

        D_mean = float(
            np.mean(D[mm])
        )

        common_mean = float(
            np.mean(common[mm])
        )

        stats = trajectory_stats(
            t,
            q,
            dq,
            u,
            a,
            seg_mask,
        )

        row = {
            "run_id": run_id,
            "target_deg": target,
            "vmax_deg_s": vmax,
            "amax_deg_s2": amax,
            "segment": seg_name,
            "t0_s": lo,
            "t1_s": hi,
            "duration_s": hi - lo,
            **stats,
            "pair_diff_mean": D_mean,
            "pair_diff_delta_from_pre":
                D_mean - D_pre,
            "common_mean": common_mean,
            "common_delta_from_pre":
                common_mean - common_pre,
        }

        all_rows.append(row)

        wide[
            f"{seg_name}_dD"
        ] = (
            D_mean - D_pre
        )

        wide[
            f"{seg_name}_dCommon"
        ] = (
            common_mean - common_pre
        )

    run_summaries.append(wide)


fieldnames = list(
    all_rows[0].keys()
)

with OUT_CSV.open(
    "w",
    newline="",
) as f:

    w = csv.DictWriter(
        f,
        fieldnames=fieldnames,
    )

    w.writeheader()
    w.writerows(all_rows)


OUT_JSON.write_text(
    json.dumps(
        {
            "runs": run_summaries,
            "segments": all_rows,
        },
        indent=2,
    )
)


run_summaries.sort(
    key=lambda x: x["amax_deg_s2"]
)


print()
print("=" * 112)
print(" ACCELERATION EXPLORATION V1 — MOTOR PAIR RESPONSE")
print("=" * 112)

print(
    f"{'amax':>7s} | "
    f"{'OA accel':>10s} "
    f"{'OA cruise':>10s} "
    f"{'OA decel':>10s} | "
    f"{'HOLD':>10s} | "
    f"{'RT accel':>10s} "
    f"{'RT cruise':>10s} "
    f"{'RT decel':>10s}"
)

print("-" * 112)

for r in run_summaries:

    print(
        f"{r['amax_deg_s2']:7.3f} | "
        f"{r['out_accel_dD']:+10.6f} "
        f"{r['out_cruise_dD']:+10.6f} "
        f"{r['out_decel_dD']:+10.6f} | "
        f"{r['target_hold_dD']:+10.6f} | "
        f"{r['ret_accel_dD']:+10.6f} "
        f"{r['ret_cruise_dD']:+10.6f} "
        f"{r['ret_decel_dD']:+10.6f}"
    )


print()
print("Common-thrust change at target hold:")

for r in run_summaries:
    print(
        f"  a={r['amax_deg_s2']:5.3f}: "
        f"{r['target_hold_dCommon']:+.7f}"
    )


hold = np.asarray([
    r["target_hold_dD"]
    for r in run_summaries
])

print()
print(
    "Target-hold pair response: "
    f"mean={hold.mean():+.6f}, "
    f"std={hold.std(ddof=1):.6f}, "
    f"CV={hold.std(ddof=1)/abs(hold.mean())*100:.2f}%"
)

print()
print("Saved:")
print(" ", OUT_CSV)
print(" ", OUT_JSON)
