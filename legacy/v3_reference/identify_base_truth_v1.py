#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


SWEEP = Path(
    "/root/autodl-tmp/px4_baseline_bags/"
    "sweeps/excitation_accel_explore_v1"
)

MANIFEST = SWEEP / "manifest.csv"

OUT_JSON = SWEEP / "identification_base_truth_v1.json"
OUT_CSV = SWEEP / "identification_base_truth_cv_v1.csv"

HISTORY_S = 0.100

HORIZONS_S = [
    0.100,
    0.200,
    0.300,
]

MODELS = {
    "B": [
        "pitch",
        "pitch_rate",
        "z",
        "vz",
    ],

    "BQ": [
        "pitch",
        "pitch_rate",
        "z",
        "vz",
        "q",
    ],

    "BQV": [
        "pitch",
        "pitch_rate",
        "z",
        "vz",
        "q",
        "dq",
    ],

    "BQVA": [
        "pitch",
        "pitch_rate",
        "z",
        "vz",
        "q",
        "dq",
        "a",
    ],
}


def event_time(result, kind):
    events = result["events"]

    if kind == "pre_end":
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
            f"{kind}: expected one event, got {len(found)}"
        )

    return float(
        found[0]["sim_time_latest_truth"]
    )


def quaternion_pitch_deg(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    s = 2.0 * (w * y - z * x)
    s = max(-1.0, min(1.0, s))

    return math.degrees(
        math.asin(s)
    )


def read_ground_truth(folder):
    bag = folder / "bag"

    reader = rosbag2_py.SequentialReader()

    storage = rosbag2_py.StorageOptions(
        uri=str(bag),
        storage_id="sqlite3",
    )

    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="",
        output_serialization_format="",
    )

    reader.open(
        storage,
        converter,
    )

    topic_types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }

    required = [
        "/clock",
        "/ground_truth/odometry",
    ]

    for topic in required:
        if topic not in topic_types:
            raise RuntimeError(
                f"Missing rosbag topic {topic}. "
                f"Available: {sorted(topic_types)}"
            )

    clock_type = get_message(
        topic_types["/clock"]
    )

    odom_type = get_message(
        topic_types["/ground_truth/odometry"]
    )

    clock_record_ns = []
    clock_sim_s = []

    gt_record_ns = []
    gt_z = []
    gt_pitch = []

    while reader.has_next():
        topic, data, record_ns = reader.read_next()

        if topic == "/clock":
            msg = deserialize_message(
                data,
                clock_type,
            )

            sim_s = (
                float(msg.clock.sec)
                +
                float(msg.clock.nanosec) * 1e-9
            )

            clock_record_ns.append(
                int(record_ns)
            )

            clock_sim_s.append(
                sim_s
            )

        elif topic == "/ground_truth/odometry":
            msg = deserialize_message(
                data,
                odom_type,
            )

            gt_record_ns.append(
                int(record_ns)
            )

            gt_z.append(
                float(
                    msg.pose.pose.position.z
                )
            )

            gt_pitch.append(
                quaternion_pitch_deg(
                    msg.pose.pose.orientation
                )
            )

    if len(clock_record_ns) < 10:
        raise RuntimeError(
            "Insufficient /clock samples"
        )

    if len(gt_record_ns) < 10:
        raise RuntimeError(
            "Insufficient ground-truth samples"
        )

    origin_ns = min(
        clock_record_ns[0],
        gt_record_ns[0],
    )

    clock_record_s = (
        np.asarray(
            clock_record_ns,
            dtype=np.float64,
        )
        -
        float(origin_ns)
    ) * 1e-9

    clock_sim_s = np.asarray(
        clock_sim_s,
        dtype=float,
    )

    gt_record_s = (
        np.asarray(
            gt_record_ns,
            dtype=np.float64,
        )
        -
        float(origin_ns)
    ) * 1e-9

    gt_z = np.asarray(
        gt_z,
        dtype=float,
    )

    gt_pitch = np.asarray(
        gt_pitch,
        dtype=float,
    )

    # Ensure clock interpolation axis is strictly monotonic.
    order = np.argsort(
        clock_record_s
    )

    clock_record_s = clock_record_s[order]
    clock_sim_s = clock_sim_s[order]

    keep = np.ones(
        len(clock_record_s),
        dtype=bool,
    )

    keep[1:] = (
        np.diff(clock_record_s) > 1e-12
    )

    clock_record_s = clock_record_s[keep]
    clock_sim_s = clock_sim_s[keep]

    gt_sim_s = np.interp(
        gt_record_s,
        clock_record_s,
        clock_sim_s,
    )

    order = np.argsort(
        gt_sim_s
    )

    gt_sim_s = gt_sim_s[order]
    gt_z = gt_z[order]
    gt_pitch = gt_pitch[order]

    keep = np.ones(
        len(gt_sim_s),
        dtype=bool,
    )

    keep[1:] = (
        np.diff(gt_sim_s) > 1e-9
    )

    return (
        gt_sim_s[keep],
        gt_pitch[keep],
        gt_z[keep],
    )


def load_motion(folder):
    result = json.loads(
        (folder / "result.json").read_text()
    )

    motion = json.loads(
        (folder / "motion_v1.json").read_text()
    )

    cols = motion["columns"]
    samples = motion["samples"]

    required = [
        "sim_s",
        "q1_rad",
        "dq1_rad_s",
        "q1_cmd_accel_rad_s2",
    ]

    missing = [
        x for x in required
        if x not in cols
    ]

    if missing:
        raise RuntimeError(
            f"{folder}: missing columns {missing}"
        )

    idx = {
        x: cols.index(x)
        for x in required
    }

    t = np.asarray(
        [r[idx["sim_s"]] for r in samples],
        dtype=float,
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

    a = np.degrees(
        np.asarray(
            [
                r[idx["q1_cmd_accel_rad_s2"]]
                for r in samples
            ],
            dtype=float,
        )
    )

    return result, t, q, dq, a


def make_run(row, horizon_s):
    run_id = row["run_id"]
    folder = Path(
        row["result_dir"]
    )

    result, t, q, dq, a = load_motion(
        folder
    )

    gt_t, gt_pitch, gt_z = read_ground_truth(
        folder
    )

    pre_end = event_time(
        result,
        "pre_end",
    )

    pre_mask = (
        (gt_t >= pre_end - 5.0)
        &
        (gt_t <= pre_end)
    )

    if np.count_nonzero(pre_mask) < 10:
        raise RuntimeError(
            f"{run_id}: insufficient PRE ground truth"
        )

    pitch_ref = float(
        np.mean(
            gt_pitch[pre_mask]
        )
    )

    z_ref = float(
        np.mean(
            gt_z[pre_mask]
        )
    )

    valid = (
        (t - HISTORY_S >= gt_t[0])
        &
        (t + horizon_s <= gt_t[-1])
    )

    t = t[valid]
    q = q[valid]
    dq = dq[valid]
    a = a[valid]

    pitch_now_abs = np.interp(
        t,
        gt_t,
        gt_pitch,
    )

    pitch_hist_abs = np.interp(
        t - HISTORY_S,
        gt_t,
        gt_pitch,
    )

    pitch_future_abs = np.interp(
        t + horizon_s,
        gt_t,
        gt_pitch,
    )

    pitch_future_hist_abs = np.interp(
        t + horizon_s - HISTORY_S,
        gt_t,
        gt_pitch,
    )

    z_now_abs = np.interp(
        t,
        gt_t,
        gt_z,
    )

    z_hist_abs = np.interp(
        t - HISTORY_S,
        gt_t,
        gt_z,
    )

    z_future_abs = np.interp(
        t + horizon_s,
        gt_t,
        gt_z,
    )

    z_future_hist_abs = np.interp(
        t + horizon_s - HISTORY_S,
        gt_t,
        gt_z,
    )

    pitch = (
        pitch_now_abs
        -
        pitch_ref
    )

    z = (
        z_now_abs
        -
        z_ref
    )

    pitch_rate = (
        pitch_now_abs
        -
        pitch_hist_abs
    ) / HISTORY_S

    vz = (
        z_now_abs
        -
        z_hist_abs
    ) / HISTORY_S

    pitch_rate_future = (
        pitch_future_abs
        -
        pitch_future_hist_abs
    ) / HISTORY_S

    vz_future = (
        z_future_abs
        -
        z_future_hist_abs
    ) / HISTORY_S

    pitch_delta = (
        pitch_future_abs
        -
        pitch_now_abs
    )

    z_delta = (
        z_future_abs
        -
        z_now_abs
    )

    return {
        "run_id": run_id,
        "n": len(t),

        "pitch": pitch,
        "pitch_rate": pitch_rate,
        "z": z,
        "vz": vz,

        "q": q,
        "dq": dq,
        "a": a,

        "pitch_rate_future": pitch_rate_future,
        "vz_future": vz_future,
        "pitch_delta": pitch_delta,
        "z_delta": z_delta,
    }


def concat(runs, key):
    return np.concatenate([
        r[key]
        for r in runs
    ])


def design(runs, features):
    return np.column_stack([
        concat(
            runs,
            feature,
        )
        for feature in features
    ])


def fit_model(
    train_runs,
    features,
    target,
):
    X = design(
        train_runs,
        features,
    )

    y = concat(
        train_runs,
        target,
    )

    mu = np.mean(
        X,
        axis=0,
    )

    sigma = np.std(
        X,
        axis=0,
    )

    sigma[
        sigma < 1e-12
    ] = 1.0

    Xs = (
        X - mu
    ) / sigma

    Xa = np.column_stack([
        np.ones(len(Xs)),
        Xs,
    ])

    # Each run receives equal total fitting weight.
    weights = np.concatenate([
        np.full(
            r["n"],
            1.0 / r["n"],
            dtype=float,
        )
        for r in train_runs
    ])

    sw = np.sqrt(
        weights
    )

    beta = np.linalg.lstsq(
        Xa * sw[:, None],
        y * sw,
        rcond=None,
    )[0]

    return {
        "features": features,
        "mu": mu,
        "sigma": sigma,
        "beta": beta,
    }


def predict(
    model,
    run,
):
    X = np.column_stack([
        run[f]
        for f in model["features"]
    ])

    Xs = (
        X - model["mu"]
    ) / model["sigma"]

    Xa = np.column_stack([
        np.ones(len(Xs)),
        Xs,
    ])

    return (
        Xa
        @ model["beta"]
    )


def metrics(
    y,
    pred,
):
    err = (
        pred - y
    )

    rmse = float(
        np.sqrt(
            np.mean(
                err ** 2
            )
        )
    )

    mae = float(
        np.mean(
            np.abs(err)
        )
    )

    denom = float(
        np.sum(
            (y - np.mean(y)) ** 2
        )
    )

    if denom > 1e-15:
        r2 = float(
            1.0
            -
            np.sum(
                err ** 2
            )
            / denom
        )
    else:
        r2 = float("nan")

    return {
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
    }


with MANIFEST.open() as f:
    rows = [
        r
        for r in csv.DictReader(f)
        if r["status"] == "PASS"
    ]

if len(rows) != 4:
    raise SystemExit(
        f"[FAIL] Expected 4 PASS runs, got {len(rows)}"
    )


TARGETS = [
    "pitch_rate_future",
    "vz_future",
    "pitch_delta",
    "z_delta",
]

all_records = []

print()
print("=" * 96)
print(" BASE-TRUTH IDENTIFICATION V1")
print("=" * 96)
print(
    "B = current base state; "
    "BQ/BQV/BQVA add arm q/dq/a."
)


for horizon_s in HORIZONS_S:

    runs = [
        make_run(
            row,
            horizon_s,
        )
        for row in rows
    ]

    print()
    print("#" * 96)
    print(
        f" FUTURE HORIZON = "
        f"{horizon_s*1000:.0f} ms"
    )
    print("#" * 96)

    for target in TARGETS:

        predictions = {
            name: []
            for name in MODELS
        }

        for held in runs:

            train = [
                r for r in runs
                if r["run_id"]
                != held["run_id"]
            ]

            for model_name, features in MODELS.items():

                model = fit_model(
                    train,
                    features,
                    target,
                )

                pred = predict(
                    model,
                    held,
                )

                y = held[target]

                met = metrics(
                    y,
                    pred,
                )

                all_records.append({
                    "horizon_ms":
                        horizon_s * 1000.0,

                    "target":
                        target,

                    "held_out_run":
                        held["run_id"],

                    "model":
                        model_name,

                    **met,
                })

                predictions[
                    model_name
                ].append({
                    "y": y,
                    "pred": pred,
                })

        pooled = {}

        for model_name in MODELS:

            y = np.concatenate([
                x["y"]
                for x in predictions[
                    model_name
                ]
            ])

            pred = np.concatenate([
                x["pred"]
                for x in predictions[
                    model_name
                ]
            ])

            pooled[
                model_name
            ] = metrics(
                y,
                pred,
            )

        def gain(
            new_name,
            ref_name,
        ):
            ref = pooled[
                ref_name
            ]["rmse"]

            new = pooled[
                new_name
            ]["rmse"]

            return (
                (ref - new)
                /
                ref
                * 100.0
            )

        q_gain = gain(
            "BQ",
            "B",
        )

        dq_gain = gain(
            "BQV",
            "BQ",
        )

        qva_gain = gain(
            "BQVA",
            "BQ",
        )

        a_increment = gain(
            "BQVA",
            "BQV",
        )

        print()
        print(target)

        print(
            f"  B    : "
            f"RMSE={pooled['B']['rmse']:.8f} "
            f"R2={pooled['B']['r2']:+.5f}"
        )

        print(
            f"  BQ   : "
            f"RMSE={pooled['BQ']['rmse']:.8f} "
            f"R2={pooled['BQ']['r2']:+.5f} "
            f"(q gain {q_gain:+.3f}%)"
        )

        print(
            f"  BQV  : "
            f"RMSE={pooled['BQV']['rmse']:.8f} "
            f"R2={pooled['BQV']['r2']:+.5f} "
            f"(dq incremental {dq_gain:+.3f}%)"
        )

        print(
            f"  BQVA : "
            f"RMSE={pooled['BQVA']['rmse']:.8f} "
            f"R2={pooled['BQVA']['r2']:+.5f} "
            f"(dynamic vs BQ {qva_gain:+.3f}%, "
            f"a incremental {a_increment:+.3f}%)"
        )


with OUT_CSV.open(
    "w",
    newline="",
) as f:

    fieldnames = list(
        all_records[0].keys()
    )

    w = csv.DictWriter(
        f,
        fieldnames=fieldnames,
    )

    w.writeheader()
    w.writerows(
        all_records
    )


OUT_JSON.write_text(
    json.dumps(
        {
            "history_s":
                HISTORY_S,

            "horizons_s":
                HORIZONS_S,

            "models":
                MODELS,

            "targets":
                TARGETS,

            "cross_validation":
                all_records,
        },
        indent=2,
    )
)


print()
print("=" * 96)
print("Saved:")
print(" ", OUT_CSV)
print(" ", OUT_JSON)
