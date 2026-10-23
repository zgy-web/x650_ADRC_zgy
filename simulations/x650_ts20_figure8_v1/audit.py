"""Complete native figure-eight audit with the unchanged hold actuator/ULog checks."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1 import audit as hold_audit
from simulations.x650_ts20_hold_v1.audit import (fixed, upper, mixed_feedback, precision, shared,
    condition_for_task, frozen_research_pid, rows, fields, actuator_check, causal_feedback_check)
from simulations.x650_ts20_figure8_v1 import task


def audit(folder, launch, output):
    folder, launch, output = Path(folder).resolve(), Path(launch).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError("Use a new audit directory")
    output.mkdir(parents=True)
    meta, stage, cfg = (task.read(folder / name) for name in ("controller_metadata.json", "stage_result.json", "controller_task.json"))
    cfg = task.validate_task_config(cfg)
    fixed.require(meta.get("task") == cfg and stage.get("kind") == "x650_ts20_conditional_run_result_v1",
                  "A new native TS20 run is required")
    fixed.require(all(meta.get(key) is True for key in ("complete", "landed_and_disarmed", "diagnostic_parameters_restored"))
                  and meta.get("diagnostic_restore_pending") is False
                  and stage.get("complete") is True and stage.get("passed") is True
                  and stage.get("eligible_as_stage_gate") is False, "Failed or un-restored run cannot be certified")
    identity = task.condition(cfg, model_sha256=meta["model_sha256"], source_urdf_sha256=meta["source_urdf_sha256"])
    fixed.require(meta.get("ts20_operating_condition") == identity == stage.get("ts20_operating_condition"),
                  "TS20 condition/source identity mismatch")
    artifacts = fixed.source_and_launch_evidence(launch, folder, meta, cfg)
    artifacts.update(fixed.verified_artifacts(folder, stage))
    condition = condition_for_task(cfg)
    _, pid, _ = frozen_research_pid(cfg)
    documents = {key: task.read(folder / name) for key, name in {
        "session": "diagnostic_parameter_session.json", "journal": "diagnostic_restore_journal_completed.json",
        "before": "diagnostic_parameters_before.json", "applied": "diagnostic_parameters_applied.json",
        "restored": "diagnostic_parameters_restored.json", "px4_before": "px4_parameters_before.json",
        "px4_after": "px4_parameters_after.json"}.items()}
    parameters = fixed.verify_parameter_session(documents, meta, condition, pid)
    reference = task.read(folder / "hover_reference.json")
    target = precision.verify_reference(reference, precision.DEFAULT_REFERENCE_SOURCE)
    window = [float(meta[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    fixed.require(abs(window[1] - window[0] - cfg["duration_s"]) <= .05,
                  "Complete declared measurement interval is required")
    manifest = task.read(folder / "native_trace_manifest.json")
    fixed.require(manifest["sha256"] == task.sha(folder / "native_trace.csv") and manifest["complete"] is True,
                  "Native trace completion/hash mismatch")
    precision.capture_window_covers(manifest, window)
    native = precision.native_states(folder / "native_trace.csv", window)
    from pyulog import ULog
    ulog = ULog(str(folder / "px4.ulg"))
    configuration = fixed.effective_configuration(ulog, native, reference, window, fixed.records(documents["applied"]))
    controls = rows(folder / "control_trace.csv")
    selected = [row for row in controls if row["phase"] == "measure" and window[0] - 1e-9 <= float(row["sim_s"]) <= window[1] + 1e-9]
    ct = fields(selected, ["sim_s"]).ravel()
    fixed.require(len(ct) >= 2 and ct[0] <= window[0] + .05 and ct[-1] >= window[1] - .05
                  and np.all(np.diff(ct) > 0) and np.diff(ct).max() <= .05 + 1e-9,
                  "Actual send intervals exceed frozen 50 ms or omit part of window")
    compute = fields(selected, ["compute_ms"]).ravel()
    fixed.require(float(compute.max()) <= 50., "Control computation exceeds frozen 50 ms")
    ground = fixed.native_ground_check(folder, task.read(folder / "native_ground.json"))
    actuator = actuator_check(folder, task.baseline_config(cfg["frozen_hold"]), window)
    commands = upper.upper_commands(ulog, native, reference, controls, window, pid, response=False)
    fixed.require(commands["zero_acceleration_feedforward"] is True, "Frozen zero acceleration FF required")
    flight = upper.flight_states(folder / "flight_samples.csv", window)
    center = shared.tip_world(np.asarray(meta["reference_base_enu_m"]), np.asarray(meta["reference_rotation"]),
                              np.zeros(2), shared.ArmModel.from_report(cfg["preparation_report"]))
    trajectory = shared.FigureEight(center, **cfg["trajectory_parameters"])
    endpoint = shared.native_tip_metrics(folder, trajectory.sample, window[0], (0., cfg["duration_s"]))
    tracking = shared.native_tip_metrics(folder, trajectory.sample, window[0], (trajectory.start_s, trajectory.end_s))
    p = cfg["figure8_protocol"]
    fixed.require(all(m["position_rmse_m"] <= p["tip_rms_max_m"] and m["position_max_m"] <= p["tip_peak_max_m"]
                      for m in (endpoint, tracking)) and float(native["tilt"].max()) <= p["tilt_max_deg"],
                  "Complete trajectory endpoint/tilt envelope failed")
    diag = [json.loads(line) for line in (folder / "controller_diagnostics.jsonl").read_text().splitlines() if line.strip()]
    causal_feedback = causal_feedback_check(folder, meta, controls, reference, window, ulog, diag)
    diag = [row["diagnostics"] for row in diag if row.get("phase") == "measure"]
    fixed.require(len(diag) == len(selected) and all(row.get("accepted_optimum") is True
                  and row.get("previous_plan_fallback", False) is False and row.get("ts20_nonlinear_check_passed") is True
                  and row.get("solve_budget_exceeded", False) is False for row in diag), "MPC acceptance missing")
    fixed.require(meta.get("figure8_protocol") == p and meta.get("frozen_hold") == cfg["frozen_hold"],
                  "Executed trajectory protocol differs")
    task.validate_stage_gates(cfg, meta["provenance"])
    result = {"endpoint": endpoint, "tracking_endpoint": tracking, "solver_solves": len(diag),
              "causal_feedback": causal_feedback, "maximum_tilt_deg": float(native["tilt"].max()),
              "trajectory_center_world_m": center.tolist(), "trajectory_parameters": cfg["trajectory_parameters"]}
    for path in folder.iterdir():
        if path.is_file() and path.suffix in (".json", ".jsonl", ".csv", ".ulg"):
            artifacts[str(path)] = task.sha(path)
    artifacts[str(Path(__file__).resolve())] = task.sha(__file__)
    for module in (fixed, fixed.joint_audit, precision, upper, mixed_feedback, shared, hold_audit, task):
        path = Path(module.__file__).resolve()
        artifacts[str(path)] = task.sha(path)
    report = {"kind": "x650_ts20_figure8_stage_certificate_v1", "figure8_protocol": cfg["figure8_protocol"], "frozen_hold": cfg["frozen_hold"], "complete": True, "passed": True,
        "stage": cfg["stage"], "source_run_directory": str(folder), "window_sim_s": window,
        "ts20_operating_condition": identity, "full_window_ulog_verified": True,
        "parameter_restore_verified": True, "native_ground_verified": True, "actuator_log_verified": True,
        "new_joint_prediction_verified": actuator["new_joint_prediction_verified"],
        "actuator": actuator, "configuration": configuration, "parameters": parameters,
        "commands": commands, "flight": flight, "ground": ground, "result": result,
        "timing": {"max_actual_send_interval_s": float(np.diff(ct).max()), "max_compute_ms": float(compute.max())},
        "qualification": cfg["ts20_protocol"]["qualification"], "artifacts": artifacts}
    (output / "figure8_stage_certificate.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("run-dir", "launch-dir", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir, args.launch_dir, args.output)
    print(json.dumps({key: report[key] for key in ("kind", "stage", "passed", "timing", "qualification")}, indent=2))


if __name__ == "__main__":
    main()
