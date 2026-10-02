#!/usr/bin/env python3

import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


SWEEP = Path(
    "/root/autodl-tmp/px4_baseline_bags/"
    "sweeps/excitation_accel_explore_v1"
)

MANIFEST = SWEEP / "manifest.csv"

OUT_JSON = SWEEP / "identification_motor_pair_v1.json"
OUT_CSV = SWEEP / "identification_motor_pair_cv_v1.csv"


MODELS = {
    "Q": ["q"],
    "QV": ["q", "dq"],
    "QVA": ["q", "dq", "a"],
}


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
            f"{kind}: expected 1 event, got {len(found)}"
        )

    return float(
        found[0]["sim_time_latest_truth"]
    )


def dataset(ulog, name):
    found = [
        d for d in ulog.data_list
        if d.name == name and d.multi_id == 0
    ]

    if len(found) != 1:
        raise RuntimeError(
            f"{name}: expected one dataset, got {len(found)}"
        )

    return found[0]


def load_run(row):

    run_id = row["run_id"]
    folder = Path(row["result_dir"])

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
        "phase",
        "q1_rad",
        "dq1_rad_s",
        "q1_cmd_rad_s",
        "q1_cmd_accel_rad_s2",
    ]

    missing = [
        x for x in required
        if x not in cols
    ]

    if missing:
        raise RuntimeError(
            f"{run_id}: missing {missing}"
        )

    idx = {
        x: cols.index(x)
        for x in required
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

    vmax = float(
        proto["motion_max_speed_deg_s"]
    )

    amax = float(
        proto["motion_max_accel_deg_s2"]
    )

    ulog = ULog(
        str(folder / "px4.ulg")
    )

    motors = dataset(
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

    m = np.column_stack([
        np.asarray(
            motors.data[f"control[{i}]"],
            dtype=float,
        )
        for i in range(4)
    ])

    D = (
        (m[:, 0] + m[:, 2]) / 2.0
        -
        (m[:, 1] + m[:, 3]) / 2.0
    )

    pre_start = event_time(
        result,
        "pre_start",
    )

    pre_end = event_time(
        result,
        "pre_end",
    )

    # Reference only against the final 5 s before motion.
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
            f"{run_id}: insufficient PRE reference"
        )

    D_pre = float(
        np.mean(D[pre_mask])
    )

    # Make sure interpolation is only inside available ULog range.
    valid = (
        (t >= tm[0])
        &
        (t <= tm[-1])
    )

    if np.count_nonzero(valid) != len(t):
        print(
            f"[WARN] {run_id}: dropping "
            f"{len(t)-np.count_nonzero(valid)} "
            "trajectory samples outside ULog time range"
        )

    t = t[valid]
    phase = phase[valid]
    q = q[valid]
    dq = dq[valid]
    u = u[valid]
    a = a[valid]

    # Motor response evaluated at the exact trajectory sample times.
    D_interp = np.interp(
        t,
        tm,
        D,
    )

    y = D_interp - D_pre

    # Diagnostic subset labels.
    subset = np.full(
        len(t),
        "OTHER",
        dtype=object,
    )

    hold = (
        phase == "target_hold"
    )

    accel_active = (
        np.abs(a)
        >= 0.95 * amax
    )

    cruise = (
        (np.abs(a) <= max(0.01, 0.05 * amax))
        &
        (np.abs(u) >= 0.95 * vmax)
    )

    subset[accel_active] = "ACCEL_ACTIVE"
    subset[cruise] = "CRUISE"
    subset[hold] = "HOLD"

    return {
        "run_id": run_id,
        "folder": str(folder),
        "amax": amax,
        "vmax": vmax,
        "t": t,
        "phase": phase,
        "subset": subset,
        "q": q,
        "dq": dq,
        "u": u,
        "a": a,
        "y": y,
        "D_pre": D_pre,
    }


def stack_runs(runs):

    return {
        key: np.concatenate([
            r[key]
            for r in runs
        ])
        for key in [
            "q",
            "dq",
            "a",
            "y",
            "subset",
        ]
    }


def design(data, names):

    cols = []

    for name in names:
        cols.append(
            np.asarray(
                data[name],
                dtype=float,
            )
        )

    return np.column_stack(cols)


def fit_standardized(
    X,
    y,
    run_lengths,
):
    """
    Standardized weighted OLS.

    Each run gets equal total weight, so the slower
    acceleration run cannot dominate simply because
    it contains more samples.
    """

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
        (X - mu)
        / sigma
    )

    Xa = np.column_stack([
        np.ones(len(Xs)),
        Xs,
    ])

    weights = []

    for n in run_lengths:
        weights.extend(
            [1.0 / n] * n
        )

    weights = np.asarray(
        weights,
        dtype=float,
    )

    sw = np.sqrt(weights)

    beta = np.linalg.lstsq(
        Xa * sw[:, None],
        y * sw,
        rcond=None,
    )[0]

    return {
        "beta": beta,
        "mu": mu,
        "sigma": sigma,
    }


def predict(model, X):

    Xs = (
        (X - model["mu"])
        /
        model["sigma"]
    )

    Xa = np.column_stack([
        np.ones(len(Xs)),
        Xs,
    ])

    return (
        Xa
        @ model["beta"]
    )


def metrics(y, pred):

    err = pred - y

    rmse = float(
        np.sqrt(
            np.mean(err ** 2)
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
            np.sum(err ** 2)
            / denom
        )
    else:
        r2 = float("nan")

    return {
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
    }


def fit_raw_equal_run(
    runs,
    names,
):
    data = stack_runs(runs)

    X = design(
        data,
        names,
    )

    y = data["y"]

    Xa = np.column_stack([
        np.ones(len(X)),
        X,
    ])

    weights = []

    for r in runs:
        n = len(r["y"])

        weights.extend(
            [1.0 / n] * n
        )

    weights = np.asarray(weights)

    sw = np.sqrt(weights)

    beta = np.linalg.lstsq(
        Xa * sw[:, None],
        y * sw,
        rcond=None,
    )[0]

    # Standardized condition number for identifiability diagnostic.
    Xstd = (
        X - np.mean(X, axis=0)
    )

    scale = np.std(
        Xstd,
        axis=0,
    )

    scale[scale < 1e-12] = 1.0

    Xstd = (
        Xstd / scale
    )

    condition = float(
        np.linalg.cond(
            np.column_stack([
                np.ones(len(Xstd)),
                Xstd,
            ])
        )
    )

    return beta, condition


# ------------------------------------------------------------
# Load formal sweep runs
# ------------------------------------------------------------

with MANIFEST.open() as f:
    rows = list(
        csv.DictReader(f)
    )

rows = [
    r for r in rows
    if r["status"] == "PASS"
]

if len(rows) != 4:
    raise SystemExit(
        f"[FAIL] expected 4 PASS runs, got {len(rows)}"
    )

runs = [
    load_run(r)
    for r in rows
]

runs.sort(
    key=lambda r: r["amax"]
)


print()
print("=" * 86)
print(" IDENTIFICATION V1 — LEAVE-ONE-RUN-OUT")
print("=" * 86)

cv_rows = []
all_predictions = {
    name: []
    for name in MODELS
}


for held in runs:

    train = [
        r for r in runs
        if r["run_id"] != held["run_id"]
    ]

    train_data = stack_runs(
        train
    )

    test_data = stack_runs(
        [held]
    )

    print()
    print(
        f"HELD OUT: {held['run_id']} "
        f"(amax={held['amax']:.3f} deg/s^2)"
    )

    for model_name, features in MODELS.items():

        Xtr = design(
            train_data,
            features,
        )

        ytr = train_data["y"]

        Xte = design(
            test_data,
            features,
        )

        yte = test_data["y"]

        fit = fit_standardized(
            Xtr,
            ytr,
            [
                len(r["y"])
                for r in train
            ],
        )

        pred = predict(
            fit,
            Xte,
        )

        met = metrics(
            yte,
            pred,
        )

        print(
            f"  {model_name:3s}: "
            f"RMSE={met['rmse']:.7f}  "
            f"MAE={met['mae']:.7f}  "
            f"R2={met['r2']:+.5f}"
        )

        row = {
            "held_out_run": held["run_id"],
            "held_out_amax_deg_s2": held["amax"],
            "model": model_name,
            **met,
        }

        cv_rows.append(row)

        all_predictions[
            model_name
        ].append({
            "run_id": held["run_id"],
            "y": yte,
            "pred": pred,
            "subset": test_data["subset"],
        })


# ------------------------------------------------------------
# Pooled CV results
# ------------------------------------------------------------

print()
print("=" * 86)
print(" POOLED HELD-OUT PERFORMANCE")
print("=" * 86)

summary = {}


for model_name in MODELS:

    records = all_predictions[
        model_name
    ]

    y = np.concatenate([
        x["y"]
        for x in records
    ])

    pred = np.concatenate([
        x["pred"]
        for x in records
    ])

    subset = np.concatenate([
        x["subset"]
        for x in records
    ])

    overall = metrics(
        y,
        pred,
    )

    summary[
        model_name
    ] = {
        "overall": overall,
        "subsets": {},
    }

    print(
        f"{model_name:3s}: "
        f"RMSE={overall['rmse']:.7f}  "
        f"MAE={overall['mae']:.7f}  "
        f"R2={overall['r2']:+.5f}"
    )

    for subset_name in [
        "ACCEL_ACTIVE",
        "CRUISE",
        "HOLD",
    ]:

        mask = (
            subset == subset_name
        )

        if np.count_nonzero(mask) < 5:
            continue

        met = metrics(
            y[mask],
            pred[mask],
        )

        summary[
            model_name
        ]["subsets"][
            subset_name
        ] = met


print()
print("=" * 86)
print(" HELD-OUT RMSE BY DYNAMIC REGION")
print("=" * 86)

print(
    f"{'MODEL':>6s} | "
    f"{'ACCEL_ACTIVE':>14s} "
    f"{'CRUISE':>14s} "
    f"{'HOLD':>14s}"
)

print("-" * 58)

for model_name in MODELS:

    ss = summary[
        model_name
    ]["subsets"]

    def get(name):
        if name not in ss:
            return float("nan")
        return ss[name]["rmse"]

    print(
        f"{model_name:>6s} | "
        f"{get('ACCEL_ACTIVE'):14.7f} "
        f"{get('CRUISE'):14.7f} "
        f"{get('HOLD'):14.7f}"
    )


# ------------------------------------------------------------
# Improvement relative to Q
# ------------------------------------------------------------

rmse_q = summary["Q"]["overall"]["rmse"]

print()
print("=" * 86)
print(" IMPROVEMENT RELATIVE TO Q-ONLY")
print("=" * 86)

for model_name in [
    "QV",
    "QVA",
]:

    rmse = summary[
        model_name
    ]["overall"]["rmse"]

    improvement = (
        (rmse_q - rmse)
        /
        rmse_q
        * 100.0
    )

    summary[
        model_name
    ][
        "rmse_improvement_vs_Q_percent"
    ] = improvement

    print(
        f"{model_name}: "
        f"{improvement:+.3f}%"
    )


# ------------------------------------------------------------
# Full-data interpretable raw-unit fit
# ------------------------------------------------------------

print()
print("=" * 86)
print(" FULL-DATA LINEAR COEFFICIENTS")
print("=" * 86)

coefficients = {}


for model_name, features in MODELS.items():

    beta, condition = fit_raw_equal_run(
        runs,
        features,
    )

    rec = {
        "intercept": float(beta[0]),
        "condition_number_standardized":
            condition,
    }

    for name, value in zip(
        features,
        beta[1:],
    ):
        rec[name] = float(value)

    coefficients[
        model_name
    ] = rec

    parts = [
        f"intercept={beta[0]:+.8f}"
    ]

    for name, value in zip(
        features,
        beta[1:],
    ):
        parts.append(
            f"beta_{name}={value:+.8f}"
        )

    parts.append(
        f"cond={condition:.2f}"
    )

    print(
        f"{model_name:3s}: "
        + "  ".join(parts)
    )


# ------------------------------------------------------------
# Save results
# ------------------------------------------------------------

with OUT_CSV.open(
    "w",
    newline="",
) as f:

    fieldnames = [
        "held_out_run",
        "held_out_amax_deg_s2",
        "model",
        "rmse",
        "mae",
        "r2",
    ]

    w = csv.DictWriter(
        f,
        fieldnames=fieldnames,
    )

    w.writeheader()
    w.writerows(cv_rows)


serializable = {
    "runs": [
        {
            "run_id": r["run_id"],
            "amax_deg_s2": r["amax"],
            "vmax_deg_s": r["vmax"],
            "samples": len(r["y"]),
            "D_pre": r["D_pre"],
        }
        for r in runs
    ],
    "models": MODELS,
    "cross_validation": cv_rows,
    "pooled": summary,
    "full_data_coefficients": coefficients,
}

OUT_JSON.write_text(
    json.dumps(
        serializable,
        indent=2,
    )
)


print()
print("Saved:")
print(" ", OUT_CSV)
print(" ", OUT_JSON)
