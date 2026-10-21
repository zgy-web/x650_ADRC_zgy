#!/usr/bin/env python3
"""Plot two accepted native precision-EV commissioning stages, without fitting.

Every curve uses original physics samples inside the audited 20/30 s window.
Displayed RMS/peak statistics are copied from the full-resolution audit, never
recomputed from rendered or reduced samples. These are hover/joint checks, not
MPC or endpoint-compensation results.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for directory in (Path(__file__).resolve().parent, ROOT/"simulations/x650_wholebody_mpc_v1"):
    sys.path.insert(0, str(directory))
import analyze_x650_joint_probe as joint_helpers
from diagnostic_parameters import load_parameter_evidence
from research_condition import canonical_hash, validate_operating_condition

COLORS = ("#2865a8", "#db7827", "#29875d")
PHASE_COLORS = ("#dce9f5", "#fbe8d5", "#e1eee4")
PHASE_NAMES = ("J1 only", "J2 only", "Opposite J1/J2")
CONTROL_NAMES = frozenset(load_parameter_evidence().CORE_CONTROL_PARAMETERS)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def actual_pid_digest(values):
    """Match the frozen/audited 42-value contract, including INT32 readbacks."""
    require(isinstance(values, dict) and set(values) == CONTROL_NAMES and len(values) == 42,
            "Actual PID must contain exactly the 42 canonical control parameter names")
    require(all(not isinstance(value, bool) and isinstance(value, (int, float)) and np.isfinite(value)
                for value in values.values()), "Actual PID must contain finite numeric readbacks")
    # The parameter evidence retains INT32 values as ints; freezing and the
    # independent full-window audit canonicalize every actual MC/MPC value to
    # float before hashing. Preserve that contract here, without value rounding.
    return canonical_hash({name: float(value) for name, value in values.items()})


def verified_local_input(path, report):
    """Resolve delivered run files by original run/name, without server access."""
    path = Path(path)
    entries = report.get("inputs_sha256", {})
    require(isinstance(entries, dict), "Audit has no hashed inputs")
    expected = [digest for source, digest in entries.items()
                if PurePosixPath(source.replace("\\", "/")).name == path.name
                and PurePosixPath(source.replace("\\", "/")).parent.name == report["source_run"]]
    require(len(expected) == 1 and path.is_file() and sha(path) == expected[0],
            "Local file differs from the accepted original run: "+str(path))
    return expected[0]


def load_stage(run_path, audit_path, expected_stage):
    folder, supplied = Path(run_path).resolve(), Path(audit_path).resolve()
    report_path = supplied/"precision_stage_audit.json" if supplied.is_dir() else supplied
    report = read_json(report_path)
    require(report.get("kind") == "x650_precision_research_stage_audit"
            and report.get("stage") == expected_stage and report.get("simulation_backend") == "gazebo_px4",
            "Expected accepted precision-EV "+expected_stage+" audit")
    for key in ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
                "parameter_restore_verified", "ground_truth_aided"):
        require(report.get(key) is True, "Full accepted research audit is required: "+key)
    condition = validate_operating_condition(report.get("operating_condition"))
    require(report.get("operating_condition_sha256") == canonical_hash(condition)
            and report.get("pid_parameters_sha256") == condition["pid_parameters_sha256"],
            "Audit operating-condition identity differs")
    certificate_path = report_path.parent/"research_stage.json"
    certificate = read_json(certificate_path)
    require(certificate.get("kind") == "x650_native_stage_evidence"
            and certificate.get("stage") == expected_stage and certificate.get("source_run") == report["source_run"]
            and certificate.get("eligible_as_stage_gate") is True and certificate.get("passed") is True
            and certificate.get("research_acceptance_verified") is True
            and certificate.get("operating_condition_sha256") == canonical_hash(condition),
            "Stage certificate differs from the accepted audit")
    bound_report = [digest for source, digest in certificate.get("artifacts", {}).items()
                    if PurePosixPath(source.replace("\\", "/")).name == report_path.name]
    require(bound_report == [sha(report_path)], "Stage certificate does not bind this audit report")
    names = ("controller_metadata.json", "controller_task.json", "hover_reference.json",
             "native_trace.csv", "native_trace_manifest.json")
    inputs = {str(folder/name): verified_local_input(folder/name, report) for name in names}
    metadata, task, reference, manifest = (read_json(folder/name) for name in
        ("controller_metadata.json", "controller_task.json", "hover_reference.json", "native_trace_manifest.json"))
    require(metadata.get("complete") is True and metadata.get("landed_and_disarmed") is True
            and metadata.get("task") == task and task.get("stage") == expected_stage
            and metadata.get("operating_condition") == condition
            and metadata.get("operating_condition_sha256") == canonical_hash(condition)
            and actual_pid_digest(metadata.get("actual_pid_parameters")) == condition["pid_parameters_sha256"],
            "Delivered run metadata differs from the accepted condition")
    require(manifest.get("complete") is True and manifest.get("sha256") == sha(folder/"native_trace.csv"),
            "Complete native trace manifest does not match")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(metadata.get(key) == manifest.get(key) == report.get(key), "Model identity mismatch: "+key)
    target = np.asarray(report["target_world_enu_m"], float)
    require(target.shape == (3,) and np.isfinite(target).all()
            and np.allclose(target, reference["target_world_enu_m"], rtol=0, atol=1e-9)
            and np.allclose(target, metadata["reference_base_enu_m"], rtol=0, atol=1e-9)
            and reference.get("frame_mode") == "raw_ev_world", "Fixed world target/frame mismatch")
    window = np.asarray(report["window_sim_s"], float)
    duration = 20. if expected_stage == "hover_folded" else 30.
    require(window.shape == (2,) and np.isfinite(window).all()
            and abs(window[1]-window[0]-duration) <= .05
            and task.get("duration_s") == duration
            and np.allclose(window, [metadata["task_start_sim_s"], metadata["task_end_sim_s"]], atol=1e-9, rtol=0),
            "Plot window differs from the full declared 20/30 second audit")
    with (folder/"native_trace.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    data = joint_helpers.native_arrays(rows, float(window[0]), float(window[1]-window[0]), target)
    position_metrics = report["position_native_world_enu"]
    require(position_metrics.get("unit") == "m" and position_metrics.get("sample_count") == len(data["t"])
            and abs(position_metrics["duration_s"]-(data["t"][-1]-data["t"][0])) <= 1e-8,
            "Native plotting rows differ from the audited metric window")
    require(all(np.isfinite(position_metrics[key]) and position_metrics[key] >= 0
                for key in ("rmse_3d", "peak_3d")), "Nonfinite full-resolution position metrics")
    phases, joint_metrics, qref = [], None, None
    if expected_stage == "joint_probe":
        sections = report["stage_acceptance"]["segments"]
        joint_metrics = sections["whole"]
        metric_window = np.asarray(joint_metrics.get("actual_interval_task_s"), float)
        require(metric_window.shape == (2,) and np.isfinite(metric_window).all()
                and 0 <= metric_window[0] <= .008+1e-9 and abs(metric_window[1]-duration) <= .008+1e-9,
                "Joint metrics do not identify the complete nominal probe window")
        metric_rows = (data["t"] >= metric_window[0]-1e-9)&(data["t"] <= metric_window[1]+1e-9)
        require(joint_metrics.get("sample_count") == int(metric_rows.sum()), "Joint metric/source sample window differs")
        for key in ("q_tracking_rmse_deg", "q_tracking_peak_deg"):
            values = np.asarray(joint_metrics[key], float)
            require(values.shape == (2,) and np.isfinite(values).all() and np.all(values >= 0), "Invalid audited joint metric: "+key)
        # Reuse the existing reference helper. No new probe equation or alignment.
        qref, _ = joint_helpers.reference(data["t"], duration)
        for name, label, color in zip(("J1_only", "J2_only", "both"), PHASE_NAMES, PHASE_COLORS):
            bounds = np.asarray(sections[name]["declared_interval_task_s"], float)
            require(bounds.shape == (2,) and 0 <= bounds[0] < bounds[1] <= duration, "Invalid audited probe phase")
            phases.append({"name": label, "start_s": float(bounds[0]), "end_s": float(bounds[1]), "color": color})
    inputs.update({str(report_path): sha(report_path), str(certificate_path): sha(certificate_path)})
    return {"folder": folder, "report_path": report_path, "report": report, "condition": condition,
            "data": data, "duration": duration, "window": window.tolist(), "target": target.tolist(),
            "position_metrics": position_metrics, "joint_metrics": joint_metrics, "qref": qref,
            "phases": phases, "inputs_sha256": inputs}


def verify_pair(hover, joint):
    require(hover["report"]["source_run"] != joint["report"]["source_run"], "Independent hover and joint runs are required")
    require(canonical_hash(hover["condition"]) == canonical_hash(joint["condition"]),
            "Hover/joint comparison changes operating conditions or PID")
    require(hover["report"]["source_urdf_sha256"] == joint["report"]["source_urdf_sha256"], "Hover/joint URDF differs")


def shade_phases(ax, phases, *, labels=False):
    for phase in phases:
        ax.axvspan(phase["start_s"], phase["end_s"], color=phase["color"], alpha=.7, zorder=0)
        ax.axvline(phase["start_s"], color="#777777", lw=.65, ls=":", alpha=.7)
        if labels:
            ax.text((phase["start_s"]+phase["end_s"])/2, .96, phase["name"],
                    transform=ax.get_xaxis_transform(), ha="center", va="top", fontsize=9,
                    bbox={"facecolor": "white", "edgecolor": "none", "alpha": .8, "pad": 1.5})
    ax.axvline(phases[-1]["end_s"], color="#777777", lw=.65, ls=":", alpha=.7)


def render(hover, joint, output):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10.5,
        "axes.titlesize": 12, "axes.spines.top": False, "axes.spines.right": False,
        "axes.unicode_minus": False, "pdf.fonttype": 42, "figure.facecolor": "white"})
    fig, axes = plt.subplots(3, 2, figsize=(14.4, 11.0))
    fig.subplots_adjust(left=.075, right=.975, top=.86, bottom=.135, hspace=.58, wspace=.21)
    fig.suptitle("X650 + 2-link arm: precision-EV commissioning", y=.977, fontsize=18, weight="bold")
    fig.text(.5, .942, "PX4 fixed-position control | Ideal EV: 50 Hz | Same frozen 42-parameter PID profile",
             ha="center", fontsize=11.5)
    fig.text(.5, .916, "Original native samples; absolute fixed-target errors. No mean removal, phase fitting or resampling.",
             ha="center", fontsize=10, color="#444444")
    body_limit = max(.05, 1.16*max(float(np.max(abs(run["data"]["body_error"]*100))) for run in (hover, joint)))
    for ax, run, title in zip(axes[0], (hover, joint), ("A  Folded-arm hover (20 s)", "B  Joint probe (30 s)")):
        if run is joint:
            shade_phases(ax, joint["phases"], labels=True)
        for index, (label, color) in enumerate(zip(("X", "Y", "Z"), COLORS)):
            ax.plot(run["data"]["t"], run["data"]["body_error"][:, index]*100, color=color, lw=1.15, label=label)
        metrics = run["position_metrics"]
        ax.set(title=f"{title}\n3D RMS {metrics['rmse_3d']*100:.3f} cm | peak {metrics['peak_3d']*100:.3f} cm",
               xlabel="Time from measurement start (s)", ylabel="Body position - fixed target (cm)",
               xlim=(0, run["window"][1]-run["window"][0]), ylim=(-body_limit, body_limit))
        ax.axhline(0, color="#555555", lw=.65, zorder=0)
        ax.grid(alpha=.22)
    axes[0, 0].legend(loc="upper right", ncol=3, fontsize=9, framealpha=.9)
    actual = np.degrees(joint["data"]["q"])
    desired = np.degrees(joint["qref"])
    error = actual-desired
    angle_limit = max(1., 1.12*float(np.max(abs(np.vstack((actual, desired))))))
    error_limit = max(.02, 1.16*float(np.max(abs(error))))
    for index in range(2):
        ax, err_ax = axes[1, index], axes[2, index]
        for current in (ax, err_ax):
            shade_phases(current, joint["phases"])
            current.set(xlim=(0, joint["window"][1]-joint["window"][0]), xlabel="Joint-probe time (s)")
            current.grid(alpha=.22)
            current.axhline(0, color="#555555", lw=.65, zorder=0)
        ax.plot(joint["data"]["t"], actual[:, index], color=COLORS[index], lw=1.5, label="Actual native joint angle")
        ax.plot(joint["data"]["t"], desired[:, index], color="#252525", lw=1.1, ls="--", label="Declared reference")
        ax.set(title=f"{'C' if index == 0 else 'D'}  Joint {index+1}: actual and reference",
               ylabel="Joint angle (deg)", ylim=(-angle_limit, angle_limit))
        ax.legend(loc="upper right", fontsize=8.5, framealpha=.9)
        err_ax.plot(joint["data"]["t"], error[:, index], color=COLORS[index], lw=1.2)
        qm = joint["joint_metrics"]
        err_ax.set(title=f"{'E' if index == 0 else 'F'}  Joint {index+1}: actual - reference\n"
            f"RMS {qm['q_tracking_rmse_deg'][index]:.3f} deg | peak {qm['q_tracking_peak_deg'][index]:.3f} deg",
            ylabel="Tracking error (deg)", ylim=(-error_limit, error_limit))
    handles = [Patch(facecolor=phase["color"], label=f"{phase['name']}: {phase['start_s']:.2f}-{phase['end_s']:.2f} s")
               for phase in joint["phases"]]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .070), ncol=3, frameon=False, fontsize=10)
    fig.text(.5, .051, "RMS/peak values are copied from accepted full-resolution audits. Each run retains its own declared world target.",
             ha="center", fontsize=9.5, color="#444444")
    fig.text(.5, .025, "Commissioning evidence only: no MPC, endpoint active-compensation, external-force rejection or hardware-precision claim.",
             ha="center", fontsize=9.5, color="#444444")
    paths = []
    for suffix in ("png", "pdf"):
        path = output/("precision_hover_joint_commissioning."+suffix)
        fig.savefig(path, dpi=180, facecolor="white")
        paths.append(path)
    plt.close(fig)
    return paths


def generate(hover_run, hover_audit, joint_run, joint_audit, output):
    output = Path(output).resolve()
    require(not output.exists(), "Use a new plot output directory; previous results are preserved")
    hover = load_stage(hover_run, hover_audit, "hover_folded")
    joint = load_stage(joint_run, joint_audit, "joint_probe")
    verify_pair(hover, joint)
    output.mkdir(parents=True)
    paths = render(hover, joint, output)
    manifest = {"kind": "x650_precision_commissioning_plot_manifest", "schema_version": 1,
        "complete": True, "scope": "Accepted precision-EV native hover and joint commissioning only",
        "operating_condition": hover["condition"], "operating_condition_sha256": canonical_hash(hover["condition"]),
        "no_mean_removal": True, "no_phase_fit": True, "native_curves_resampled": False,
        "metrics_source": "Copied from accepted full-resolution audits; no metric computed from plotting samples",
        "window_semantics": "All original native rows in the audit's actual stage window are plotted. Joint statistics retain their separately audited nominal probe interval; a scheduler tail is never removed or included in recomputed statistics.",
        "whole_body_compensation_claim": False, "mpc_claim": False, "hardware_performance_claim": False,
        "runs": {label: {"source_run": run["report"]["source_run"], "window_sim_s": run["window"],
            "target_world_enu_m": run["target"], "native_samples_plotted": len(run["data"]["t"]),
            "native_median_interval_s": float(np.median(np.diff(run["data"]["t"]))),
            "position_native_world_enu": run["position_metrics"], "joint_metrics": run["joint_metrics"]}
            for label, run in (("hover", hover), ("joint", joint))},
        "probe_phases": joint["phases"], "inputs_sha256": {**hover["inputs_sha256"], **joint["inputs_sha256"]},
        "generator_sources_sha256": {str(Path(__file__).resolve()): sha(__file__),
                                     str(Path(joint_helpers.__file__).resolve()): sha(joint_helpers.__file__)},
        "outputs": [{"path": path.name, "bytes": path.stat().st_size, "sha256": sha(path)} for path in paths]}
    (output/"plot_manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("hover-run", "hover-audit", "joint-run", "joint-audit", "output"):
        parser.add_argument("--"+flag, type=Path, required=True)
    args = parser.parse_args()
    manifest = generate(args.hover_run, args.hover_audit, args.joint_run, args.joint_audit, args.output)
    print(json.dumps({"output": str(args.output), "complete": manifest["complete"], "files": manifest["outputs"]}, indent=2))


if __name__ == "__main__":
    main()
