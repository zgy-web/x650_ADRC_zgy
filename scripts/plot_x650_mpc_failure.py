#!/usr/bin/env python3
"""Plot the original partial window of a failed MPC run; never issue acceptance."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
import plot_x650_mixed_stage_comparison as display
import audit_x650_mixed_stage as audit

require, read, sha = display.require, display.read, display.sha


def load_failure(folder):
    folder = Path(folder).resolve()
    names = ("controller_metadata.json", "controller_task.json", "native_trace.csv",
             "control_trace.csv", "controller_diagnostics.jsonl")
    paths = {name: folder/name for name in names}
    require(all(path.is_file() for path in paths.values()), "Failed run lacks original plot inputs")
    inputs = {str(path): sha(path) for path in paths.values()}
    meta, task = read(paths["controller_metadata.json"]), read(paths["controller_task.json"])
    require(meta.get("complete") is False and meta.get("phase") == "failed", "Only explicitly incomplete failed runs are accepted")
    require(task == meta.get("task") and task.get("controller") == "mpc", "Original failed MPC task is required")
    start = float(meta["task_start_sim_s"])
    require(np.isfinite(start), "Declared actual measurement start is missing")
    control = [row for row in display.csv_rows(paths["control_trace.csv"]) if row.get("phase") == "measure"]
    ct = display.native_reader.array(control, ["sim_s"]).ravel()
    require(len(ct) >= 2 and np.all(np.diff(ct) > 0) and ct[0] >= start-1e-9 and ct[0] <= start+.05,
            "Original measurement commands are missing or have invalid time coverage")
    require(all(row.get("pva_sent") in ("True", "true", "1") for row in control), "Every plotted command must have actually been sent")
    end = float(ct[-1])
    require(start < end and end-start <= float(task["duration_s"])+.05, "Invalid partial measurement window")
    require(np.allclose(ct-start, display.native_reader.array(control, ["t_s"]).ravel(), atol=1e-8, rtol=0),
            "Original measurement time origin differs")
    physical = display.native_reader.native_arrays(display.csv_rows(paths["native_trace.csv"]), start, end-start,
                                                  meta["reference_base_enu_m"])
    model = audit.ArmModel(**meta["physical_profile"])
    center = audit.tip_world(meta["reference_base_enu_m"], meta["reference_rotation"], [0., 0.], model)
    target = audit.declared_tip_positions(task, center, physical["t"])
    error = physical["tip"]-target
    entries = [json.loads(line) for line in paths["controller_diagnostics.jsonl"].read_text(encoding="utf-8").splitlines() if line.strip()]
    entries = [row for row in entries if row.get("phase") == "measure" and start-1e-9 <= row["sim_s"] <= end+1e-9]
    require(len(entries) == len(ct) and np.allclose([row["sim_s"] for row in entries], ct, atol=1e-8, rtol=0),
            "Original successful-command diagnostics do not pair with controls")
    solve = np.asarray([row["diagnostics"]["solve_ms"] for row in entries], float)
    compute = display.native_reader.array(control, ["compute_ms"]).ravel()
    require(np.isfinite(solve).all() and np.all(solve >= 0) and np.all(compute >= 0), "Invalid recorded timing")
    stage_path = folder/"stage_result.json"
    if stage_path.is_file():
        stage = read(stage_path)
        require(stage.get("complete") is False and stage.get("passed") is False, "Failed metadata conflicts with stage result")
        inputs[str(stage_path)] = sha(stage_path)
        for name in ("native_trace.csv", "control_trace.csv"):
            require(stage.get("artifacts", {}).get(name) == sha(paths[name]), "Original failed stage artifact changed: "+name)
    return dict(folder=folder, metadata=meta, task=task, window=[start, end], inputs=inputs,
                physical=physical, center=center, error=error, control=control, control_t=ct-start,
                solve_ms=solve, compute_ms=compute)


def generate(folder, output):
    data = load_failure(folder)
    output = Path(output).resolve()
    require(not output.exists(), "A new output directory is required")
    output.mkdir(parents=True)
    p, error, ct = data["physical"], data["error"], data["control_t"]
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), constrained_layout=True)
    for j, label in enumerate("XYZ"):
        axes[0, 0].plot(p["t"], 1000*error[:, j], label="Tip "+label)
    axes[0, 0].set_ylabel("Native tip position error (mm)")
    axes[0, 1].plot(p["t"], 1000*np.linalg.norm(error, axis=1), label="Tip 3D error norm")
    axes[0, 1].set_ylabel("Native tip position error norm (mm)")
    axes[1, 0].plot(p["t"], 1000*p["body_error"][:, 1], label="Body Y minus fixed base anchor Y")
    axes[1, 0].plot(p["t"], 1000*error[:, 1], label="Tip Y minus declared target Y")
    axes[1, 0].plot(p["t"], 1000*(error[:, 1]-p["body_error"][:, 1]), ls=":",
                    label="Geometry Y contribution = tip error - body deviation")
    axes[1, 0].set_ylabel("World ENU Y deviations (mm)")
    axes[1, 1].plot(p["t"], p["q"][:, 1], label="Native q2")
    axes[1, 1].plot(ct, display.native_reader.array(data["control"], ["q2"]).ravel(), ls=":", label="Controller measured q2")
    lower = data["task"]["mixed_mpc_protocol"]["common_controller_config"]["q_lower"][1]
    axes[1, 1].axhline(lower, color="red", ls="--", label="Declared q2 soft lower bound")
    axes[1, 1].set_ylabel("Joint 2 angle (rad)")
    axes[2, 0].plot(p["t"], p["dq"][:, 1], label="Native dq2")
    axes[2, 0].step(ct, display.native_reader.array(data["control"], ["qd2_cmd"]).ravel(), where="post", ls="--", label="Sent dq2 command")
    axes[2, 0].set_ylabel("Joint 2 velocity (rad/s)")
    axes[2, 1].plot(ct, data["solve_ms"], label="Optimizer solve_ms")
    axes[2, 1].plot(ct, data["compute_ms"], label="Recorded control compute_ms")
    axes[2, 1].set_ylabel("Recorded wall computation (ms)")
    display.decorate(axes)
    duration = data["window"][1]-data["window"][0]
    fig.suptitle(f"FAILED MPC {data['task']['stage']}: partial {duration:.3f} s through last successfully sent command\n"
                 "Diagnostic only: original time/target, no fitted alignment, no full-window acceptance or PID ranking", fontsize=12)
    files = []
    for suffix in (".png", ".pdf"):
        path = output/("mpc_failure_partial"+suffix)
        fig.savefig(path, dpi=160, bbox_inches="tight")
        files.append(path)
    plt.close(fig)
    require(all(sha(path) == digest for path, digest in data["inputs"].items()), "Original input changed while rendering")
    manifest = {"kind": "x650_failed_mpc_partial_display", "schema_version": 1,
        "render_complete": True, "experiment_complete": False, "diagnostic_only": True, "eligible_as_stage_gate": False,
        "formal_performance_claim": False, "full_window_pid_comparison_permitted": False,
        "source_run": data["folder"].name, "source_run_directory": str(data["folder"]),
        "stage": data["task"]["stage"], "recorded_error": data["metadata"].get("error"),
        "landed_and_disarmed_recorded": data["metadata"].get("landed_and_disarmed"),
        "parameters_restored_recorded": data["metadata"].get("diagnostic_parameters_restored"),
        "partial_window_sim_s": data["window"], "partial_duration_s": duration,
        "declared_full_duration_s": data["task"]["duration_s"], "successful_sent_commands": len(ct),
        "window_end_semantics": "Last successful measure control row; excludes unissued failing solve and all recovery/landing samples",
        "reference_center_world_enu_m": data["center"].tolist(),
        "partial_native_tip_error": {key: display.precision.metrics(data["window"][0]+p["t"], error, "m")[key]
                                     for key in ("rmse_3d", "peak_3d", "axis_rmse", "mean_vector")},
        "fitted_time_or_phase_alignment": False, "mean_subtraction": False, "refitting": False,
        "y_decomposition": "tip_y-target_tip_y = (base_y-fixed_base_anchor_y) + geometric remainder; exact position decomposition, not causal attribution",
        "solver_timing_semantics": "solve_ms is optimizer only; compute_ms is separately recorded control span, not end-to-end delay",
        "inputs_sha256": data["inputs"], "generator_sha256": sha(__file__),
        "display_helpers_sha256": {str(Path(module.__file__).resolve()): sha(module.__file__)
                                   for module in (display, audit, display.native_reader, display.precision)},
        "outputs_sha256": {str(path): sha(path) for path in files}}
    (output/"failure_display_manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = generate(args.run, args.output)
    print(json.dumps({"diagnostic_only": True, "partial_duration_s": result["partial_duration_s"], "output": str(args.output.resolve())}))


if __name__ == "__main__":
    main()
