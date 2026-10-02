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

OUT_CSV = SWEEP / "identification_motor_pair_lag_v1.csv"
OUT_JSON = SWEEP / "identification_motor_pair_lag_v1.json"

LAGS_S = np.arange(
    0.0,
    0.3001,
    0.020,
)


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


def get_dataset(ulog, name):
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

    needed = [
        "sim_s",
        "phase",
        "q1_rad",
        "dq1_rad_s",
        "q1_cmd_accel_rad_s2",
    ]

    missing = [
        x for x in needed
        if x not in cols
    ]

    if missing:
        raise RuntimeError(
            f"{run_id}: missing columns {missing}"
        )

    idx = {
        x: cols.index(x)
        for x in needed
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

    a = np.degrees(
        np.asarray(
            [
                r[idx["q1_cmd_accel_rad_s2"]]
                for r in samples
            ],
            dtype=float,
        )
    )

    amax = float(
        result["protocol"]["motion_max_accel_deg_s2"]
    )

    ulog = ULog(
        str(folder / "px4.ulg")
    )

    motors = get_dataset(
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
            f"{run_id}: insufficient PRE motor samples"
        )

    D_pre = float(
        np.mean(D[pre_mask])
    )

    valid = (
        (t >= tm[0])
        &
        (t <= tm[-1])
    )

    t = t[valid]
    phase = phase[valid]
    q = q[valid]
    dq = dq[valid]
    a = a[valid]

    D_t = np.interp(
        t,
        tm,
        D,
    )

    y = D_t - D_pre

    return {
        "run_id": run_id,
        "amax": amax,
        "t": t,
        "phase": phase,
        "q": q,
        "dq": dq,
        "a": a,
        "y": y,
    }


def lagged_run(run, lag_s):
    t = run["t"]

    query = t - lag_s

    valid = (
        query >= t[0]
    )

    tv = t[valid]
    q_current = run["q"][valid]
    y = run["y"][valid]

    dq_lag = np.interp(
        query[valid],
        t,
        run["dq"],
    )

    a_lag = np.interp(
        query[valid],
        t,
        run["a"],
    )

    accel_active = (
        np.abs(a_lag)
        >= 0.95 * run["amax"]
    )

    return {
        "run_id": run["run_id"],
        "q": q_current,
        "dq": dq_lag,
        "a": a_lag,
        "y": y,
        "accel_active": accel_active,
        "n": len(y),
    }


def concatenate(runs, key):
    return np.concatenate([
        r[key]
        for r in runs
    ])


def design(runs, features):
    cols = []

    for feature in features:
        cols.append(
            concatenate(
                runs,
                feature,
            )
        )

    return np.column_stack(cols)


def fit_model(train_runs, features):
    X = design(
        train_runs,
        features,
    )

    y = concatenate(
        train_runs,
        "y",
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

    weights = np.concatenate([
        np.full(
            r["n"],
            1.0 / r["n"],
            dtype=float,
        )
        for r in train_runs
    ])

    sw = np.sqrt(weights)

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


def predict(model, run):
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
        f"[FAIL] Expected 4 PASS runs, got {len(rows)}"
    )

base_runs = [
    load_run(r)
    for r in rows
]

base_runs.sort(
    key=lambda x: x["amax"]
)


MODELS = {
    "Q": ["q"],
    "QV": ["q", "dq"],
    "QVA": ["q", "dq", "a"],
}


records = []


for lag_s in LAGS_S:

    lag_runs = [
        lagged_run(
            r,
            lag_s,
        )
        for r in base_runs
    ]

    predictions = {
        name: []
        for name in MODELS
    }

    for held in lag_runs:

        train = [
            r for r in lag_runs
            if r["run_id"] != held["run_id"]
        ]

        for model_name, features in MODELS.items():

            model = fit_model(
                train,
                features,
            )

            pred = predict(
                model,
                held,
            )

            predictions[
                model_name
            ].append({
                "y": held["y"],
                "pred": pred,
                "accel_active":
                    held["accel_active"],
            })

    pooled = {}

    for model_name in MODELS:

        y = np.concatenate([
            x["y"]
            for x in predictions[model_name]
        ])

        pred = np.concatenate([
            x["pred"]
            for x in predictions[model_name]
        ])

        accel_active = np.concatenate([
            x["accel_active"]
            for x in predictions[model_name]
        ])

        overall = metrics(
            y,
            pred,
        )

        if np.count_nonzero(accel_active) >= 5:
            accel_met = metrics(
                y[accel_active],
                pred[accel_active],
            )
        else:
            accel_met = {
                "rmse": float("nan"),
                "mae": float("nan"),
                "r2": float("nan"),
            }

        pooled[
            model_name
        ] = {
            "overall": overall,
            "accel_active": accel_met,
        }

    q_rmse = pooled["Q"]["overall"]["rmse"]

    qv_rmse = pooled["QV"]["overall"]["rmse"]

    qva_rmse = pooled["QVA"]["overall"]["rmse"]

    q_acc_rmse = pooled["Q"]["accel_active"]["rmse"]

    qva_acc_rmse = pooled["QVA"]["accel_active"]["rmse"]

    qv_improvement = (
        (q_rmse - qv_rmse)
        / q_rmse
        * 100.0
    )

    qva_improvement = (
        (q_rmse - qva_rmse)
        / q_rmse
        * 100.0
    )

    if np.isfinite(q_acc_rmse) and q_acc_rmse > 0:
        qva_acc_improvement = (
            (q_acc_rmse - qva_acc_rmse)
            / q_acc_rmse
            * 100.0
        )
    else:
        qva_acc_improvement = float("nan")

    records.append({
        "lag_ms": float(lag_s * 1000.0),

        "Q_rmse":
            q_rmse,

        "QV_rmse":
            qv_rmse,

        "QVA_rmse":
            qva_rmse,

        "QV_improvement_vs_Q_percent":
            qv_improvement,

        "QVA_improvement_vs_Q_percent":
            qva_improvement,

        "Q_accel_active_rmse":
            q_acc_rmse,

        "QVA_accel_active_rmse":
            qva_acc_rmse,

        "QVA_accel_active_improvement_percent":
            qva_acc_improvement,

        "Q_r2":
            pooled["Q"]["overall"]["r2"],

        "QV_r2":
            pooled["QV"]["overall"]["r2"],

        "QVA_r2":
            pooled["QVA"]["overall"]["r2"],
    })


best_qva = max(
    records,
    key=lambda r:
        r["QVA_improvement_vs_Q_percent"],
)

best_accel = max(
    records,
    key=lambda r:
        (
            r[
                "QVA_accel_active_improvement_percent"
            ]
            if np.isfinite(
                r[
                    "QVA_accel_active_improvement_percent"
                ]
            )
            else -1e99
        ),
)


print()
print("=" * 108)
print(" MOTOR-PAIR LAG IDENTIFICATION V1")
print("=" * 108)

print(
    f"{'lag':>7s} | "
    f"{'Q RMSE':>10s} "
    f"{'QV RMSE':>10s} "
    f"{'QVA RMSE':>10s} | "
    f"{'QV gain':>9s} "
    f"{'QVA gain':>9s} | "
    f"{'QVA accel gain':>15s}"
)

print("-" * 108)


for r in records:

    print(
        f"{r['lag_ms']:6.0f}ms | "
        f"{r['Q_rmse']:10.7f} "
        f"{r['QV_rmse']:10.7f} "
        f"{r['QVA_rmse']:10.7f} | "
        f"{r['QV_improvement_vs_Q_percent']:+8.3f}% "
        f"{r['QVA_improvement_vs_Q_percent']:+8.3f}% | "
        f"{r['QVA_accel_active_improvement_percent']:+14.3f}%"
    )


print()
print("=" * 108)
print(" BEST OVERALL QVA LAG")
print("=" * 108)

print(
    f"lag = {best_qva['lag_ms']:.0f} ms"
)

print(
    f"Q RMSE   = {best_qva['Q_rmse']:.7f}"
)

print(
    f"QVA RMSE = {best_qva['QVA_rmse']:.7f}"
)

print(
    "QVA improvement vs Q = "
    f"{best_qva['QVA_improvement_vs_Q_percent']:+.3f}%"
)


print()
print("=" * 108)
print(" BEST ACCEL-ACTIVE QVA LAG")
print("=" * 108)

print(
    f"lag = {best_accel['lag_ms']:.0f} ms"
)

print(
    "QVA accel-region improvement = "
    f"{best_accel['QVA_accel_active_improvement_percent']:+.3f}%"
)


with OUT_CSV.open(
    "w",
    newline="",
) as f:

    fieldnames = list(
        records[0].keys()
    )

    w = csv.DictWriter(
        f,
        fieldnames=fieldnames,
    )

    w.writeheader()
    w.writerows(records)


OUT_JSON.write_text(
    json.dumps(
        {
            "lag_definition":
                "positive lag means dq/a at t-lag predict motor response at t",

            "lags_ms": [
                float(x * 1000.0)
                for x in LAGS_S
            ],

            "records":
                records,

            "best_overall_qva":
                best_qva,

            "best_accel_active_qva":
                best_accel,
        },
        indent=2,
    )
)


print()
print("Saved:")
print(" ", OUT_CSV)
print(" ", OUT_JSON)
