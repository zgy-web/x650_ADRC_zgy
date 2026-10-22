#!/usr/bin/env python3
"""Display two accepted mixed-stage runs; never fit, tune or issue a stage gate."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import audit_x650_mixed_stage as audit
import analyze_x650_joint_probe as native_reader
import analyze_x650_hover_precision as precision

COLORS = {"pid_ff": "#1768a6", "mpc": "#d06420"}
LABELS = {"pid_ff": "PID allocation baseline", "mpc": "Upper MPC"}
FLAGS = ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
         "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def csv_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def rpy_degrees(quaternion):
    """Native world ENU orientation; no reference subtraction or phase unwrapping."""
    w, x, y, z = np.asarray(quaternion).T
    return np.degrees(np.column_stack((np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x), -1, 1)), np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))))


def load_run(audit_path, run_dir, controller):
    audit_path, folder = Path(audit_path).resolve(), Path(run_dir).resolve()
    report = read(audit_path)
    require(report.get("kind") == "x650_precision_mixed_stage_audit" and report.get("schema_version") == 3
            and all(report.get(key) is True for key in FLAGS), "Only complete, independently accepted mixed-stage audits can be plotted")
    require(report.get("controller") == controller and report.get("source_run") == folder.name
            and Path(report["source_run_directory"]).resolve() == folder, "Audit/controller/run directory mismatch")
    require(report.get("simulation_backend") == "gazebo_px4" and report.get("acceleration_feedforward") == "zero"
            and report.get("no_fitted_origin_phase_or_mean") is True, "Unexpected simulation or reference contract")
    stage = report["stage_acceptance"]
    require(stage.get("passed") is True and stage.get("criteria")
            and all(value is True for value in stage["criteria"].values()), "Nested stage criteria did not all pass")
    protocol = report["mixed_mpc_protocol"]
    require(canonical(protocol) == report["mixed_mpc_protocol_sha256"], "Protocol digest mismatch")
    require(canonical(report["operating_condition"]) == report["operating_condition_sha256"], "Operating condition digest mismatch")
    bound = {str(Path(path).resolve()): digest for path, digest in report["inputs_sha256"].items()}
    inputs = {str(audit_path): sha(audit_path)}
    def original(name):
        path = folder / name
        require(path.is_file() and bound.get(str(path)) == sha(path), "Original accepted plot input missing or changed: "+str(path))
        inputs[str(path)] = sha(path)
        return path
    metadata = read(original("controller_metadata.json"))
    task = read(original("controller_task.json"))
    require(task == metadata["task"] and task["controller"] == controller and task["stage"] == report["stage"]
            and task["mixed_mpc_protocol"] == protocol, "Task differs from accepted stage/protocol")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(metadata[key] == report[key], "Physical model identity differs from original metadata")
    window = np.asarray(report["window_sim_s"], float)
    require(window.shape == (2,) and np.isfinite(window).all() and window[1] > window[0]
            and np.allclose(window, [metadata["task_start_sim_s"], metadata["task_end_sim_s"]], atol=1e-9, rtol=0),
            "Original measurement window differs")
    data = native_reader.native_arrays(csv_rows(original("native_trace.csv")), window[0], window[1]-window[0],
                                       metadata["reference_base_enu_m"])
    center = np.asarray(stage["reference_center_world_enu_m"], float)
    target = audit.declared_tip_positions(task, center, data["t"])
    error = data["tip"]-target
    replay = precision.metrics(window[0]+data["t"], error, "m")
    accepted = stage["native_tip_error"]
    for key in ("rmse_3d", "peak_3d", "axis_rmse", "mean_vector"):
        require(np.allclose(replay[key], accepted[key], atol=1e-9, rtol=1e-9), "Native tip statistics do not reproduce the accepted audit: "+key)
    control = [row for row in csv_rows(original("control_trace.csv")) if row["phase"] == "measure"
               and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    ct = native_reader.array(control, ["sim_s"]).ravel()-window[0]
    require(len(ct) >= 2 and np.all(np.diff(ct) > 0), "Missing/nonmonotone original command times")
    command = native_reader.array(control, ["qd1_cmd", "qd2_cmd"])
    compute = native_reader.array(control, ["compute_ms"]).ravel()
    entries = [json.loads(line) for line in original("controller_diagnostics.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    entries = [row for row in entries if row.get("phase") == "measure" and window[0]-1e-9 <= row["sim_s"] <= window[1]+1e-9]
    require(len(entries) == len(ct) and np.allclose([row["sim_s"] for row in entries], ct+window[0], atol=1e-8, rtol=0),
            "Original diagnostics do not match command receipts")
    solve = None
    if controller == "mpc":
        solve = np.asarray([row["diagnostics"]["solve_ms"] for row in entries], float)
        require(np.isfinite(solve).all() and np.all(solve >= 0), "Invalid original optimizer times")
    require(np.all(compute >= 0), "Invalid recorded control compute times")
    return dict(controller=controller, report=report, task=task, metadata=metadata, physical=data,
                target=target, error=error, control_t=ct, joint_command=command, compute_ms=compute,
                solve_ms=solve, inputs=inputs, replay=replay)


def verify_pair(pid, mpc):
    require(pid["controller"] == "pid_ff" and mpc["controller"] == "mpc", "Comparison must contain PID then MPC")
    for key in ("stage", "source_stage", "model_sha256", "source_urdf_sha256", "pid_parameters_sha256",
                "operating_condition", "operating_condition_sha256", "mixed_mpc_protocol", "mixed_mpc_protocol_sha256",
                "feedback_contract", "control_interface", "acceleration_feedforward"):
        require(pid["report"].get(key) is not None and pid["report"][key] == mpc["report"].get(key), "Runs are not a matched comparison: "+key)
    for key in ("stage", "duration_s", "trajectory_parameters", "control_period_s"):
        require(pid["task"].get(key) == mpc["task"].get(key), "Declared tasks differ: "+key)
    require(pid["metadata"]["physical_profile"] == mpc["metadata"]["physical_profile"], "Arm physical profiles differ")
    for key in ("model", "evidence", "preregistration", "source_archive", "candidate", "training_report"):
        require(pid["report"]["accepted_response_binding"][key]["sha256"] ==
                mpc["report"]["accepted_response_binding"][key]["sha256"], "Accepted response evidence differs: "+key)
    # The two reference rotations/centers are bound separately before each run.
    # Their small physical differences are retained, never fitted away.


def decorate(axes):
    for ax in np.asarray(axes).ravel():
        ax.grid(True, alpha=.25)
        ax.set_xlabel("Time since this run's declared measurement start (s)")
        ax.legend(fontsize=8, loc="best")


def figures(runs, output):
    files = []
    def save(fig, name):
        for suffix in (".png", ".pdf"):
            path = output/(name+suffix)
            fig.savefig(path, dpi=160, bbox_inches="tight")
            files.append(path)
        plt.close(fig)
    fig, axes = plt.subplots(4, 2, figsize=(15, 13), constrained_layout=True)
    for run in runs:
        data, error = run["physical"], run["error"]
        c, label = COLORS[run["controller"]], LABELS[run["controller"]]
        for axis in range(3):
            axes[axis, 0].plot(data["t"], 1000*error[:, axis], color=c, label=label)
            axes[axis, 0].set_ylabel("Tip "+"XYZ"[axis]+" error (mm)")
            axes[axis, 1].plot(data["t"], data["position"][:, axis], color=c, label=label)
            axes[axis, 1].set_ylabel("Body world ENU "+"XYZ"[axis]+" (m)")
        axes[3, 0].plot(data["t"], 1000*np.linalg.norm(error, axis=1), color=c, label=label)
        axes[3, 0].set_ylabel("Tip 3D position error norm (mm)")
        for j, style in enumerate(("-", "--", ":")):
            axes[3, 1].plot(data["t"], rpy_degrees(data["quaternion"])[:, j], color=c, ls=style,
                            label=label+" "+("roll", "pitch", "yaw")[j])
        axes[3, 1].set_ylabel("Native world orientation (deg)")
    decorate(axes)
    fig.suptitle(runs[0]["report"]["stage"]+": accepted native position tracking | zero acceleration feedforward\n"
                 "Each run keeps its own pre-bound reference center; no mean subtraction or fitted time/phase alignment", fontsize=12)
    save(fig, "mixed_stage_tracking")
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), constrained_layout=True)
    for run in runs:
        data, c, label = run["physical"], COLORS[run["controller"]], LABELS[run["controller"]]
        for j in range(2):
            axes[0, j].plot(data["t"], np.degrees(data["q"][:, j]), color=c, label=label)
            axes[0, j].set_ylabel(f"Joint {j+1} actual angle (deg)")
            axes[1, j].plot(data["t"], np.degrees(data["dq"][:, j]), color=c, label=label+" actual")
            axes[1, j].step(run["control_t"], np.degrees(run["joint_command"][:, j]), where="post", color=c, ls="--", label=label+" command")
            axes[1, j].set_ylabel(f"Joint {j+1} velocity (deg/s)")
        axes[2, 1].plot(run["control_t"], run["compute_ms"], color=c, label=label)
        if run["solve_ms"] is not None:
            axes[2, 0].plot(run["control_t"], run["solve_ms"], color=c, label="MPC optimizer solve_ms")
    budget = runs[1]["report"]["mixed_mpc_protocol"]["acceptance"]["solver"]["solve_budget_ms"]
    axes[2, 0].axhline(budget, color="#555555", ls="--", label=f"Declared optimizer budget {budget:g} ms")
    axes[2, 0].set_ylabel("MPC optimizer solve_ms (ms)")
    axes[2, 1].set_ylabel("Recorded control compute_ms (ms)")
    decorate(axes)
    fig.suptitle("Native joints and original commands | joint commands are velocities, not angle references\n"
                 "Optimizer solve_ms excludes part of setup/KKT; compute_ms is the separately logged control span", fontsize=12)
    save(fig, "mixed_stage_joints_solver")
    if runs[0]["task"].get("trajectory_parameters") is not None:
        fig, axes = plt.subplots(2, len(runs), figsize=(15, 7), squeeze=False,
                                 gridspec_kw={"height_ratios": [1.5, 1]}, constrained_layout=True)
        y_extent_mm = max(1e-3, 1100*max(float(np.max(abs(run["physical"]["tip"][:, 1]-run["target"][:, 1])))
                                      for run in runs))
        for column, run in enumerate(runs):
            data, reference = run["physical"], run["target"]
            color, label = COLORS[run["controller"]], LABELS[run["controller"]]
            center = np.asarray(run["report"]["stage_acceptance"]["reference_center_world_enu_m"], float)
            path_ax, y_ax = axes[:, column]
            path_ax.plot(reference[:, 0], reference[:, 2], color="#444444", ls="--", lw=1.7,
                         label="Declared reference", zorder=3)
            path_ax.plot(data["tip"][:, 0], data["tip"][:, 2], color=color, lw=1.25,
                         label="Actual native tip", zorder=2)
            path_ax.scatter([center[0]], [center[2]], marker="+", s=75, color="#222222",
                            label="Pre-bound center", zorder=4)
            path_ax.set_aspect("equal", adjustable="box")
            path_ax.ticklabel_format(axis="both", style="plain", useOffset=False)
            path_ax.set_xlabel("Absolute world ENU X (m)")
            path_ax.set_ylabel("Absolute world ENU Z (m)")
            path_ax.set_title(label+"\nPre-bound center ENU (m): "
                              +", ".join(f"{value:.6f}" for value in center), fontsize=10, pad=32)
            path_ax.grid(True, alpha=.25)
            path_ax.legend(fontsize=8, loc="lower center", bbox_to_anchor=(.5, 1.01), ncol=3)
            y_ax.plot(data["t"], 1000*(data["tip"][:, 1]-reference[:, 1]), color=color,
                      label="Native Y minus this run's declared Y")
            y_ax.axhline(0., color="#444444", ls="--", lw=1., label="Declared XZ plane")
            y_ax.set_xlabel("Time since this run's declared measurement start (s)")
            y_ax.set_ylabel("Y departure (mm)")
            y_ax.set_ylim(-y_extent_mm, y_extent_mm)
            y_ax.grid(True, alpha=.25)
            y_ax.legend(fontsize=8, loc="best")
        fig.suptitle(runs[0]["report"]["stage"]+": native XZ paths | FULL MEASUREMENT WINDOW\n"
                     "Absolute ENU coordinates; equal X/Z scale in each panel; each original center retained\n"
                     "Includes initial hold, smooth start/stop, both cycles and final hold; no translation or phase fit",
                     fontsize=12)
        save(fig, "mixed_stage_paths")
    return files


def generate(pid_audit, pid_run, mpc_audit, mpc_run, output):
    runs = [load_run(pid_audit, pid_run, "pid_ff"), load_run(mpc_audit, mpc_run, "mpc")]
    verify_pair(*runs)
    output = Path(output).resolve()
    require(not output.exists(), "Use a new output directory; original evidence/output is never overwritten")
    output.mkdir(parents=True)
    files = figures(runs, output)
    inputs = {}
    for run in runs:
        for path, digest in run["inputs"].items():
            require(path not in inputs or inputs[path] == digest, "Conflicting input hashes")
            inputs[path] = digest
    # Fail if an input changed while the viewer was rendering.
    require(all(sha(path) == digest for path, digest in inputs.items()), "Original input changed during plotting")
    result = {"kind": "x650_mixed_stage_comparison_display", "schema_version": 1,
        "complete": True, "eligible_as_stage_gate": False, "new_physical_audit_performed": False,
        "scope": "Display of two previously accepted runs; selected display input hashes verified, not a new full ULog audit",
        "stage": runs[0]["report"]["stage"], "mixed_mpc_protocol_sha256": runs[0]["report"]["mixed_mpc_protocol_sha256"],
        "zero_acceleration_feedforward": True, "mean_subtraction": False, "fitted_time_or_phase_alignment": False,
        "refitting": False, "position_error_source": "native tip position minus declared target at original native time",
        "time_axis": "Original sim_s minus each predeclared task_start_sim_s",
        "runs": [{"controller": run["controller"], "source_run": run["report"]["source_run"],
                  "window_sim_s": run["report"]["window_sim_s"],
                  "reference_center_world_enu_m": run["report"]["stage_acceptance"]["reference_center_world_enu_m"],
                  "native_tip_error": {key: run["replay"][key] for key in ("rmse_3d", "peak_3d", "axis_rmse", "mean_vector")},
                  "maximum_recorded_compute_ms": float(run["compute_ms"].max()),
                  "maximum_optimizer_solve_ms": None if run["solve_ms"] is None else float(run["solve_ms"].max())} for run in runs],
        "inputs_sha256": inputs,
        "plot_sources_sha256": {str(Path(module.__file__).resolve()): sha(module.__file__) for module in (audit, native_reader, precision)},
        "generator_sha256": sha(__file__),
        "outputs_sha256": {str(path): sha(path) for path in files}}
    (output/"comparison_manifest.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("pid-audit", "pid-run", "mpc-audit", "mpc-run", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    args = parser.parse_args()
    result = generate(args.pid_audit, args.pid_run, args.mpc_audit, args.mpc_run, args.output)
    print(json.dumps({"complete": result["complete"], "stage": result["stage"], "output": str(args.output.resolve())}))


if __name__ == "__main__":
    main()
