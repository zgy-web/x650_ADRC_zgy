#!/usr/bin/env python3
"""Plot certified isolated hold/normal physical records without alignment.

Example: python3 scripts/plot_x650_isolated_validation.py --report AUDIT/report.json --output NEW_DIRECTORY
The complete certificate and original CSV/LiveStart/task remain authoritative.
This renderer does not run a controller, change a certificate, or grant a gate.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from simulations.x650_wholebody_mpc_v1 import run_case as shared


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def checked_input(report, path):
    path = Path(path).resolve(strict=True)
    bindings = {str(Path(p).resolve()): h for p, h in report.get("artifacts", {}).items()}
    expected = bindings.get(str(path))
    actual = sha(path)
    require(expected is not None and expected == actual,
            "Original chart input is missing from certificate or has changed: " + str(path))
    return {"path": str(path), "sha256": actual}


def verified_data(report_path, *, certificate_kinds=None):
    """Return original physical rows only after matching the certified metric."""
    report_path = Path(report_path).resolve(strict=True)
    report = read(report_path)
    kinds = ({"tip_hold": "x650_isolated_live_stage_certificate_v1",
              "normal": "x650_isolated_normal_stage_certificate_v1"}
             if certificate_kinds is None else certificate_kinds)
    stage = report.get("stage")
    require(stage in kinds and report.get("kind") == kinds[stage],
            "Only certified isolated hold or normal figure eight can be plotted")
    require(all(report.get(key) is True for key in ("passed", "complete", "full_window_ulog_verified",
            "parameter_restore_verified", "native_ground_verified", "actuator_log_verified")),
            "A complete passed physical certificate is required")
    run = Path(report["source_run_directory"]).resolve(strict=True)
    paths = {"native_trace": run / "native_trace.csv", "live_start": run / "isolated_live/live_start.json",
             "task": run / "controller_task.json"}
    inputs = {key: checked_input(report, path) for key, path in paths.items()}
    inputs["report"] = {"path": str(report_path), "sha256": sha(report_path)}
    start, task = read(paths["live_start"]), read(paths["task"])
    origin, duration = float(start["origin_s"]), float(task["duration_s"])
    require(task["stage"] == stage and duration == (30. if stage == "tip_hold" else 32.),
            "Unexpected stage or incomplete declared duration")
    window = np.asarray(report["window_sim_s"], float)
    require(window.shape == (2,) and np.isfinite(window).all()
            and np.max(abs(window - [origin, origin + duration])) <= 1e-9,
            "Report window differs from actual handoff and task duration")
    center = np.asarray(start["target_position_enu"], float)
    require(center.shape == (3,) and np.isfinite(center).all(), "Invalid actual reference center")
    reference = (shared.ConstantTarget(center) if stage == "tip_hold" else
                 shared.FigureEight(center, **task["trajectory_parameters"]))
    metrics = shared.native_tip_metrics(run, reference.sample, origin, (0., duration))
    certified = report["endpoint"]
    require(certified.get("same_time") is True and certified.get("no_phase_alignment") is True,
            "Certificate does not declare same-time unaligned native metrics")
    require(np.array_equal(np.asarray(certified["interval_task_s"], float), [0., duration]),
            "Certificate metrics do not cover the entire task")
    for key in ("position_rmse_m", "position_max_m", "axis_rmse_m", "duration_s"):
        actual, expected = np.asarray(metrics[key], float), np.asarray(certified[key], float)
        require(actual.shape == expected.shape and np.isfinite(actual).all() and np.isfinite(expected).all()
                and np.allclose(actual, expected, rtol=1e-10, atol=1e-12),
                "Recomputed native metric differs from certificate: " + key)
    require(metrics["sample_count"] == certified["sample_count"], "Native sample count differs from certificate")
    # Exactly the native_tip_metrics window predicate, fields and target times.
    samples = []
    with paths["native_trace"].open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sim = float(row["sim_s"])
            t = sim - origin
            if -1e-9 <= t <= duration + 1e-9:
                require(row["valid_tip"] == "1", "Invalid independent native tip row")
                samples.append([sim, t, *[float(row["tip_" + axis]) for axis in "xyz"]])
    values = np.asarray(samples, float)
    require(values.shape == (metrics["sample_count"], 5) and np.isfinite(values).all(),
            "Nonfinite or incomplete original chart samples")
    t, actual = values[:, 1], values[:, 2:5]
    require(np.all(np.diff(t) > 0.) and np.max(np.diff(t)) <= .008 + 1e-9
            and t[0] <= .008 + 1e-9 and t[-1] >= duration - .008 - 1e-9,
            "Physical chart samples do not cover the full native window")
    desired = np.asarray([reference.sample(at)[0] for at in t], float)
    error = actual - desired
    norm = np.linalg.norm(error, axis=1)
    integrate = getattr(np, "trapezoid", None) or np.trapz
    require(np.isfinite(desired).all()
            and np.isclose(np.sqrt(integrate(norm ** 2, t) / (t[-1] - t[0])), metrics["position_rmse_m"], rtol=1e-10, atol=1e-12)
            and np.isclose(norm.max(), metrics["position_max_m"], rtol=1e-10, atol=1e-12),
            "Exported chart rows do not reproduce the native audit metrics")
    # Ensure a concurrent rewrite did not change input bytes during the reads.
    require(all(sha(item["path"]) == item["sha256"] for item in inputs.values()), "Chart inputs changed while being read")
    return {"stage": stage, "duration": duration, "reference": reference, "metrics": metrics,
            "inputs": inputs, "values": values, "desired": desired, "error": error, "norm": norm}


def render(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import ScalarFormatter

    output = Path(output)
    require(not output.exists(), "Output must be a new directory; existing results are never overwritten")
    t, actual = data["values"][:, 1], data["values"][:, 2:5]
    desired, errors, norm = data["desired"], data["error"], data["norm"]
    metrics = data["metrics"]
    title = "Normal figure eight" if data["stage"] == "normal" else "End-effector hold"
    with plt.rc_context({"font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10,
                         "pdf.fonttype": 42, "ps.fonttype": 42, "path.simplify": False}):
        figure = plt.figure(figsize=(13, 7.4))
        grid = figure.add_gridspec(2, 2, width_ratios=(1., 1.25), hspace=.42, wspace=.28,
                                  left=.075, right=.97, bottom=.15, top=.83)
        path_axis = figure.add_subplot(grid[:, 0])
        magnitude_axis = figure.add_subplot(grid[0, 1])
        component_axis = figure.add_subplot(grid[1, 1], sharex=magnitude_axis)
        path_axis.plot(actual[:, 0], actual[:, 2], color="#0072B2", lw=1.2, label="Native physical tip", zorder=2)
        if data["stage"] == "normal":
            path_axis.plot(desired[:, 0], desired[:, 2], color="#D55E00", lw=1.25, ls="--", label="Reference", zorder=3)
        else:
            path_axis.scatter(desired[0, 0], desired[0, 2], marker="+", s=140, linewidths=1.8,
                              color="#D55E00", label="Fixed reference", zorder=3)
        path_axis.scatter(actual[0, 0], actual[0, 2], marker="o", s=28, color="#009E73", label="Actual start", zorder=4)
        path_axis.scatter(actual[-1, 0], actual[-1, 2], marker="x", s=35, color="#333333", label="Actual finish", zorder=4)
        path_axis.set(xlabel="World X [m]", ylabel="World Z [m]", title="A  End-effector path (world XZ projection)")
        path_axis.set_aspect("equal", adjustable="box")
        path_axis.legend(loc="best", fontsize=9)
        for axis in (path_axis.xaxis, path_axis.yaxis):
            axis.set_major_formatter(ScalarFormatter(useOffset=False))
        magnitude_axis.plot(t, norm * 1000., color="#0072B2", lw=1.1, label="3D position error")
        magnitude_axis.axhline(metrics["position_rmse_m"] * 1000., color="#333333", lw=.9, ls="--", label="Time-weighted RMS")
        magnitude_axis.set(title="B  Same-time three-dimensional position error", ylabel="Error norm [mm]", ylim=(0., None))
        magnitude_axis.legend(loc="best", fontsize=9)
        for index, color, label in zip(range(3), ("#0072B2", "#D55E00", "#009E73"), ("X", "Y", "Z")):
            component_axis.plot(t, errors[:, index] * 1000., color=color, lw=1., label=label)
        component_axis.axhline(0., color="#888888", lw=.6)
        component_axis.set(title="C  Signed position error (actual minus reference)",
                           xlabel="Task time [s]", ylabel="Axis error [mm]", xlim=(0., data["duration"]))
        component_axis.legend(loc="best", ncol=3, fontsize=9)
        if data["stage"] == "normal":
            for axis in (magnitude_axis, component_axis):
                axis.axvspan(0., data["reference"].start_s, color="#777777", alpha=.08)
                axis.axvspan(data["reference"].end_s, data["duration"], color="#777777", alpha=.08)
        for axis in (path_axis, magnitude_axis, component_axis):
            axis.grid(True, color="#dddddd", linewidth=.6)
            axis.set_axisbelow(True)
        figure.suptitle("X650 + two-link arm | " + title + " | Isolated MPC runtime", y=.965, fontsize=15)
        figure.text(.5, .91, "Complete %.0f s window | Native samples: %d | 3D RMS %.3f mm | Peak %.3f mm" %
                    (data["duration"], len(t), metrics["position_rmse_m"] * 1000., metrics["position_max_m"] * 1000.),
                    ha="center", fontsize=11)
        footer = "Original Gazebo physics timestamps and measured tip; no time/position alignment or resampling. Simulation result; not hardware qualification."
        if data["stage"] == "normal":
            footer += "\nLight shading: initial/final hold; the full 32 s remains in every metric."
        footer += "\nNative CSV SHA256: " + data["inputs"]["native_trace"]["sha256"]
        figure.text(.075, .065, footer, fontsize=8.3, va="center", linespacing=1.55)
        # Only now create the destination, after all numeric/evidence checks.
        output.mkdir(parents=True, exist_ok=False)
        figure.savefig(output / "endpoint_validation.png", dpi=180, facecolor="white")
        figure.savefig(output / "endpoint_validation.pdf", facecolor="white",
                       metadata={"Title": "X650 isolated MPC: " + title,
                                 "Subject": "Certified original-time independent native endpoint record"})
        plt.close(figure)
    fields = (["sim_s", "task_s"] + ["actual_" + axis + "_m" for axis in "xyz"]
              + ["reference_" + axis + "_m" for axis in "xyz"]
              + ["error_" + axis + "_m" for axis in "xyz"] + ["error_norm_m"])
    with (output / "endpoint_chart_data.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(np.column_stack((data["values"], desired, errors, norm)).tolist())
    manifest = {"kind": "x650_isolated_validation_figure_v1", "complete": True, "stage": data["stage"],
                "full_window_task_s": [0., data["duration"]], "metrics": metrics,
                "metric_match_verified_before_write": True, "inputs": data["inputs"],
                "coordinate_frame": "world ENU", "no_offset_or_phase_fit": True, "resampled": False,
                "hardware_qualified": False, "renderer": {"path": str(Path(__file__).resolve()), "sha256": sha(__file__)},
                "outputs": {p.name: {"bytes": p.stat().st_size, "sha256": sha(p)} for p in sorted(output.iterdir())}}
    with (output / "figure_manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists(), "Output directory already exists; choose a new destination")
    result = render(verified_data(args.report), args.output)
    print(json.dumps({"output": str(args.output.resolve()), "stage": result["stage"],
                      "metrics": result["metrics"], "complete": result["complete"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
