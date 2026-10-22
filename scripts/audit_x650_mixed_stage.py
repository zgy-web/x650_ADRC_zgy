#!/usr/bin/env python3
"""Independent physical/ULog/command/solver audit of mixed-response v3 stages.

Old response audits and their evidence remain unchanged. Each new stage is
accepted only from complete native flight, matched protocol and zero-FF inputs.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1",
                  ROOT/"control_packages/x650_wholebody_v1"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
import audit_x650_upper_stage as upper
from audit_x650_upper_stage import *
import audit_x650_mixed_feedback as mixed_feedback
from audit_x650_mixed_feedback import (feedback_check_mixed, audit_mixed_acceleration_feedback,
                                        load_native_acceleration)
from mixed_mpc_protocol import validate_mixed_mpc_protocol, protocol_hash, baseline_config, controller_config
from accepted_response_model import load_accepted_response_model

AUDIT_FILENAME = "mixed_mpc_stage_audit.json"
AUDIT_KIND = "x650_precision_mixed_stage_audit"


def verify_model_archive(launch_dir, metadata, binding):
    launch_dir = Path(launch_dir).resolve()
    launch = read_json(launch_dir/"launch.json")
    require(metadata["provenance"].get("accepted_response_binding") == binding
            and launch["provenance"].get("accepted_response_binding") == binding,
            "Runtime/launch accepted model binding changed")
    archived = launch.get("accepted_response_archive", {})
    require(set(archived) == {"model", "evidence", "preregistration", "source_archive", "candidate", "training_report"},
            "Mixed launch lacks original model/certification snapshots")
    artifacts = {}
    for key, saved in archived.items():
        path = Path(saved["path"]).resolve()
        require(path.parent == launch_dir and path.is_file()
                and saved["sha256"] == binding[key]["sha256"] == precision.sha(path)
                and Path(saved["source_path"]).resolve() == Path(binding[key]["path"]).resolve(),
                "Accepted response snapshot changed: "+key)
        artifacts[str(path)] = precision.sha(path)
    return artifacts


def verify_controller_details(metadata, task, protocol, accepted_model, pid):
    """Reconstruct frozen settings only; never run an optimizer or fit data."""
    from x650_wholebody.adapter import ControllerAdapter
    from x650_wholebody.baseline import BaselineController
    from x650_wholebody.mpc import MpcController
    require(task.get("controller") in ("pid_ff", "mpc"), "Unknown mixed controller")
    model = ArmModel(**metadata["physical_profile"])
    allocator = (MpcController(model, pid, config=controller_config(protocol, accepted_model))
                 if task["controller"] == "mpc" else BaselineController(model, pid, config=baseline_config(protocol)))
    expected = ControllerAdapter(allocator, model, pid, acceleration_feedforward_policy="zero").metadata()
    require(canonical_hash(metadata.get("controller_details")) == canonical_hash(expected),
            "Executed controller/model/weights differ from the frozen comparison settings")
    return {"verified": True, "metadata_sha256": canonical_hash(expected), "optimizer_executed": False}


def declared_tip_positions(task, center, elapsed):
    t = np.asarray(elapsed, dtype=float)
    require(t.ndim == 1 and np.isfinite(t).all(), "Invalid original trajectory times")
    require(task.get("stage") in ("tip_hold", "small", "normal", "fast"), "Unknown mixed trajectory stage")
    result = np.tile(vector(center, 3, "declared tip center"), (len(t), 1))
    if task["stage"] == "tip_hold":
        return result
    c = task["trajectory_parameters"]
    s = t-c["start_s"]
    ramp, total = c["ramp_s"], c["cycles"]*c["period_s"]+c["ramp_s"]
    omega = 2*np.pi/c["period_s"]
    def primitive(u):
        return 2.5*u**4-3*u**5+u**6
    phi = np.empty_like(t)
    for i, x in enumerate(s):
        if x <= 0: phi[i] = 0.
        elif x >= total: phi[i] = 2*np.pi*c["cycles"]
        elif x < ramp: phi[i] = omega*ramp*primitive(x/ramp)
        elif x > total-ramp: phi[i] = 2*np.pi*c["cycles"]-omega*ramp*primitive((total-x)/ramp)
        else: phi[i] = omega*(x-.5*ramp)
    result[:, 0] += c["amplitude_x"]*np.sin(phi)
    result[:, 2] += c["amplitude_z"]*np.sin(2*phi)
    return result


def logged_tip_target_audit(flight, control, task, center, window):
    selected, times = selected_rows(control, window)
    ft = np.asarray([float(row["sim_time_s"])-window[0] for row in flight])
    require(len(ft) >= 2 and np.isfinite(ft).all() and np.all(np.diff(ft) > 0),
            "Missing or nonmonotone recorded target samples")
    actual = fields(flight, "tip_target_world_", ("x_m", "y_m", "z_m"))
    # Recorder callbacks can precede command publication in the same pump.
    # Match exact declared target values within the established control-gap
    # and clock-boundary allowances; never fit a phase or accept a free offset.
    source_t = np.r_[0., times-window[0]]
    source_p = declared_tip_positions(task, center, source_t)
    for now, target in zip(ft, actual):
        indices = np.flatnonzero((source_t >= now-.05-CLOCK_BOUNDARY_S-1e-9)
                                &(source_t <= now+CLOCK_BOUNDARY_S+1e-9))
        require(len(indices) and np.any(np.max(abs(source_p[indices]-target), axis=1) <= 1e-9),
                "Logged target is not an original-time held declared trajectory target")
    return {"verified": True, "samples": len(ft), "maximum_hold_error_m": float(np.max(
        np.linalg.norm(actual-declared_tip_positions(task, center, ft), axis=1))),
        "control_gap_allowance_s": .05, "clock_boundary_allowance_s": CLOCK_BOUNDARY_S}


def mixed_solver_audit(folder, control, metadata, task, protocol, window):
    selected, times = selected_rows(control, window)
    with (folder/"controller_diagnostics.jsonl").open(encoding="utf-8") as f:
        entries = [json.loads(line) for line in f if line.strip()]
    entries = [x for x in entries if x.get("phase") == "measure"
               and window[0]-1e-9 <= x["sim_s"] <= window[1]+1e-9]
    require(len(entries) == len(selected), "Every sent command requires its original allocator diagnostics")
    model = ArmModel(**metadata["physical_profile"])
    center = tip_world(metadata["reference_base_enu_m"], metadata["reference_rotation"], [0., 0.], model)
    targets = declared_tip_positions(task, center, times-window[0])
    results, limit = [], protocol["acceptance"]["solver"]
    for row, entry, now, target in zip(selected, entries, times, targets):
        require(abs(entry["sim_s"]-now) <= 1e-8 and abs(entry["t_s"]-float(row["t_s"])) <= 1e-8,
                "Allocator diagnostics do not pair with original command")
        d = entry["diagnostics"]
        require(np.max(abs(vector(d["allocation"]["tip_ref"], 3, "allocator tip target")-target)) <= 1e-9,
                "Allocator target differs from the declared trajectory at its original time")
        require(np.max(abs(vector(d["allocation"]["base_velocity_cmd"], 3, "allocated base velocity")
                           -fields([row], "base_velocity_target_", "xyz")[0])) <= 1e-9
                and np.max(abs(vector(d["allocation"]["qd_cmd"], 2, "allocated joint velocity")
                               -fields([row], "qd", ("1_cmd", "2_cmd"))[0])) <= 1e-9,
                "Sent command differs from the paired allocator output")
        require(d.get("acceleration_feedforward_policy") == "zero"
                and d.get("acceleration_feedforward_applied") is False
                and np.max(abs(vector(d["command_acceleration_world"], 3, "executed acceleration"))) <= 1e-12,
                "Both comparison groups must execute zero acceleration FF")
        if task["controller"] == "mpc":
            if "prediction_chain" in protocol["mpc_controller_config"]:
                prediction_chain_use_audit(d, row, protocol)
            require(d.get("solver_success") is True and d.get("accepted_optimum") is True and d.get("previous_plan_fallback") is False,
                    "MPC returned an unaccepted optimum or fallback")
            for key in ("primal_violation", "kkt_stationarity_inf", "solve_ms"):
                require(isinstance(d[key], (float, int)) and not isinstance(d[key], bool)
                        and np.isfinite(d[key]) and d[key] >= 0., "Invalid solver metric")
            require(d.get("prediction_response_kind") == protocol["response_model_kind"]
                    and d.get("prediction_response_identified") is True
                    and d.get("prediction_state_order") == ["px", "py", "pz", "vx", "vy", "vz", "q1", "q2", "dq1", "dq2", "ax", "ay"],
                    "MPC did not execute the reviewed mixed prediction state")
            require(d["primal_violation"] <= limit["primal_max"]
                    and d["kkt_stationarity_inf"] <= limit["kkt_max"]
                    and d["solve_budget_ms"] == limit["solve_budget_ms"],
                    "MPC solution exceeds predeclared numerical quality limits")
            require(d.get("solve_budget_exceeded") is (d["solve_ms"] > limit["solve_budget_ms"]),
                    "Solver timing flag disagrees with actual duration")
            require(np.max(abs(vector(d["measured_acceleration_enu"], 3, "MPC acceleration")
                               -fields([row], "feedback_a", "xyz")[0])) <= 1e-9
                    and np.max(abs(vector(d["initial_prediction_acceleration_xy"], 2, "MPC initial acceleration")
                                   -fields([row], "feedback_a", "xy")[0])) <= 1e-9,
                    "MPC initialization did not use the paired actual PX4 acceleration")
            results.append(d)
    if task["controller"] != "mpc":
        return {"verified": True, "controller": "pid_ff", "samples": len(entries),
                "zero_acceleration_ff_verified": True, "solver_applicable": False, "passed": True}
    return {"verified": True, "controller": "mpc", "samples": len(results), "solver_applicable": True,
            "zero_acceleration_ff_verified": True, "all_accepted_optimum": True, "fallback_count": 0,
            "maximum_primal_violation": max(x["primal_violation"] for x in results),
            "maximum_kkt_stationarity_inf": max(x["kkt_stationarity_inf"] for x in results),
            "maximum_solve_ms": max(x["solve_ms"] for x in results),
            "solve_budget_exceeded_count": sum(x["solve_budget_exceeded"] for x in results),
            "passed": not any(x["solve_budget_exceeded"] for x in results)}


def prediction_chain_use_audit(d, row, protocol):
    """Check the paired input and output semantics; do not rerun the optimizer."""
    require(d.get("prediction_chain") == protocol["mpc_controller_config"]["prediction_chain"],
            "MPC did not use the frozen prediction chain")
    now, stamp = float(row["sim_s"]), float(row["joint_stamp_s"])
    for key, expected in (("prediction_control_time_s", now), ("prediction_joint_stamp_s", stamp),
                          ("prediction_joint_measurement_age_s", now-stamp)):
        require(isinstance(d.get(key), (int, float)) and np.isfinite(d[key]) and abs(d[key]-expected) <= 1e-9,
                "Prediction chain time differs from actual consumed state: "+key)
    for key, expected in (("prediction_measured_joint_position", [float(row["q1"]), float(row["q2"])]),
                          ("prediction_measured_joint_velocity", [float(row["qd1"]), float(row["qd2"])])):
        require(np.max(abs(vector(d.get(key), 2, key)-expected)) <= 1e-9,
                "Prediction chain joint initialization differs from actual measured feedback")
    for key in ("prediction_initial_joint_position", "prediction_initial_joint_velocity"):
        vector(d.get(key), 2, key)
    rotations = np.asarray(d.get("predicted_rotations_flu_to_enu"), float)
    states = np.asarray(d.get("predicted_states"), float)
    count = protocol["mpc_controller_config"]["nodes"]
    require(rotations.shape == (count, 3, 3) and np.isfinite(rotations).all()
            and states.shape == (count, 12) and np.isfinite(states).all(), "Missing chain forecast evidence")
    for rotation in rotations:
        require(np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6, rtol=0)
                and abs(np.linalg.det(rotation)-1) <= 1e-6, "Invalid predicted attitude rotation")
    return True


def mixed_stage_metrics(folder, metadata, task, protocol, window, physical, common_checks):
    require(metadata.get("folded_before_landing") is True
            and metadata["physical_profile"].get("source_report_sha256") ==
                metadata["provenance"].get("preparation_report_sha256"),
            "Mixed stage lacks folded landing or bound physical profile")
    model = ArmModel(**metadata["physical_profile"])
    center = tip_world(metadata["reference_base_enu_m"], metadata["reference_rotation"], [0., 0.], model)
    target = declared_tip_positions(task, center, physical["t"])
    native_geometry = joint_audit.native_tip_audit(physical, metadata["physical_profile"])
    tip_metrics = precision.metrics(window[0]+physical["t"], physical["tip"]-target, "m")
    flight = [row for row in rows(folder/"flight_samples.csv") if row["phase"] == "measure"
              and window[0]-1e-9 <= float(row["sim_time_s"]) <= window[1]+1e-9]
    control = rows(folder/"control_trace.csv")
    logged_target = logged_tip_target_audit(flight, control, task, center, window)
    solver = mixed_solver_audit(folder, control, metadata, task, protocol, window)
    acceptance = protocol["acceptance"]
    checks = {**common_checks,
        "tip_rms_within_predeclared_limit": tip_metrics["rmse_3d"] <= acceptance["tip_rms_max_m"],
        "tip_peak_within_predeclared_limit": tip_metrics["peak_3d"] <= acceptance["tip_peak_max_m"],
        "physical_joint_limits": bool(np.all(physical["q"] >= model.joint_lower)
                                      and np.all(physical["q"] <= model.joint_upper)),
        "solver_quality_and_time_budget": solver["passed"]}
    chain_limits = None
    if "prediction_chain" in protocol["mpc_controller_config"]:
        chain_limits = prediction_chain_limit_audit(control, window, physical, protocol)
        checks.update(chain_limits["criteria"])
    return {"passed": all(checks.values()), "criteria": checks, "native_tip_error": tip_metrics,
            "native_tip_geometry": native_geometry, "solver": solver,
            "reference_center_world_enu_m": center.tolist(),
            "logged_target_audit": logged_target,
            "logged_target_maximum_hold_error_m": logged_target["maximum_hold_error_m"],
            **({"prediction_chain_limits": chain_limits} if chain_limits is not None else {}),
            "scoring": "Full native measurement window; independently evaluated declared target; no fitted offset/phase",
            "scope": "Zero acceleration FF upper PID/MPC commissioning; no injected external disturbance or torque NMPC claim"}


def prediction_chain_limit_audit(control, window, physical, protocol):
    """Check actual physical limits and command slew, independent of QP flags."""
    settings = protocol["common_controller_config"]
    selected, times = selected_rows(control, window)
    commands = np.column_stack((fields(selected, "base_velocity_target_", "xyz"),
                                fields(selected, "qd", ("1_cmd", "2_cmd"))))
    recorded_dt = np.asarray([float(row["control_dt_s"]) for row in selected])
    require(np.isfinite(recorded_dt).all() and np.all(recorded_dt > 0)
            and np.all(recorded_dt <= settings["max_dt_s"]+1e-10),
            "Prediction chain recorded invalid control intervals")
    # The preceding measured row is required; never invent a zero initial input.
    previous = [row for row in control if float(row["sim_s"]) < times[0]-1e-9]
    require(previous, "Prediction chain command audit requires the actual preceding command")
    row = previous[-1]
    initial = np.r_[fields([row], "base_velocity_target_", "xyz")[0],
                    fields([row], "qd", ("1_cmd", "2_cmd"))[0]]
    # The prediction-chain runner preserves the actual previous command time
    # at activation. Every dt, including the first, must match that clock;
    # slew is calculated independently from the original command timestamps.
    dt = np.diff(np.r_[float(row["sim_s"]), times])
    require(np.isfinite(dt).all() and np.all(dt > 0)
            and np.all(dt <= settings["max_dt_s"]+1e-10),
            "Prediction chain actual command intervals exceed the common limit")
    require(np.all(abs(recorded_dt-dt) <= 1e-9),
            "Prediction chain recorded control intervals differ from command timestamps")
    delta = commands-np.vstack((initial, commands[:-1]))
    rates = abs(delta)/dt[:, None]
    q, dq = np.asarray(physical["q"]), np.asarray(physical["dq"])
    checks = {
        "native_joints_inside_common_soft_limits": bool(np.all(q >= np.asarray(settings["q_lower"])-1e-8)
                                                       and np.all(q <= np.asarray(settings["q_upper"])+1e-8)),
        "native_joint_velocity_inside_common_caps": bool(np.all(abs(dq) <= np.asarray(settings["velocity_limit"])[3:]+1e-8)),
        "sent_command_slew_inside_common_caps": bool(np.all(abs(delta) <= dt[:, None]*np.asarray(settings["acceleration_limit"])+1e-8)),
    }
    return {"criteria": checks, "maximum_sent_command_slew": rates.max(axis=0).tolist(),
            "maximum_actual_command_interval_s": float(dt.max()),
            "maximum_recorded_control_interval_s": float(recorded_dt.max()),
            "first_actual_command_interval_s": float(dt[0]),
            "first_recorded_control_interval_s": float(recorded_dt[0]),
            "native_q_min_rad": q.min(axis=0).tolist(), "native_q_max_rad": q.max(axis=0).tolist(),
            "native_abs_dq_max_rad_s": abs(dq).max(axis=0).tolist(),
            "semantics": "Command slew uses original command-row simulation times, not Gazebo application acknowledgements; this does not identify physical braking acceleration"}


def analyze(folder, launch_dir, *, ulog_loader=None):
    folder = Path(folder).resolve(strict=True)
    names = ("stage_result.json", "controller_metadata.json", "controller_task.json", "hover_reference.json",
        "native_trace.csv", "native_trace_manifest.json", "flight_samples.csv", "native_ground.json", "native_ground.csv",
        "native_ground_manifest.json", "controller_diagnostics.jsonl", "control_trace.csv", "feedback_trace.jsonl", "estimated_feedback.json", "px4_parameters_before.json",
        "px4_parameters_after.json", "diagnostic_parameter_session.json", "diagnostic_parameters_before.json",
        "diagnostic_parameters_applied.json", "diagnostic_parameters_restored.json", "diagnostic_restore_journal_completed.json",
        "diagnostic_odometry.json", "diagnostic_parameter_storage_save.log", "px4.ulg")
    paths = {name: folder/name for name in names}
    require(all(path.is_file() for path in paths.values()), "Missing original upper research artifact")
    stage, metadata, task = (read_json(paths[name]) for name in ("stage_result.json", "controller_metadata.json", "controller_task.json"))
    from task import validate_task_config, validate_stage_gates
    require(validate_task_config(task) == task, "Mixed task does not match the reviewed preset")
    require(task == metadata.get("task") and task["stage"] in ("tip_hold", "small", "normal", "fast"), "Upper research task mismatch")
    condition = condition_for_task(task)
    require(condition is not None and condition.get("research_profile") == "precision_ev_mixed_mpc_v3", "Reviewed mixed v3 condition is required")
    _, pid, pid_hash = frozen_research_pid(task)
    for label, document in (("stage", stage), ("metadata", metadata)):
        require(document.get("complete") is True and document.get("landed_and_disarmed") is True
                and document.get("stage") == task["stage"] and document.get("diagnostic_only") is False
                and document.get("ground_truth_aided") is True and document.get("eligible_as_stage_gate") is False
                and document.get("acceptance_pending") == "full_ulog_audit", "Only completed provisional research results can be accepted: "+label)
        fixed_audit.verify_condition(document, condition, label)
    require(stage.get("passed") is True and stage.get("success") is True and metadata.get("diagnostic_parameters_restored") is True
            and metadata.get("diagnostic_restore_pending") is False and metadata.get("external_force_disturbance") is False,
            "Original upper run failed or cleanup incomplete")
    require(metadata.get("feedback_contract") == condition["feedback_contract"], "Upper state feedback contract missing")
    require(metadata.get("allocator_active") is True, "Mixed stage requires active allocation")
    protocol = validate_mixed_mpc_protocol(task.get("mixed_mpc_protocol"))
    require(metadata.get("mixed_mpc_protocol") == stage.get("mixed_mpc_protocol") == protocol,
            "Mixed execution protocol differs across original records")
    require(metadata.get("acceleration_feedforward_policy") == "zero", "Mixed model requires zero acceleration FF")
    require(protocol is not None and metadata.get("mixed_mpc_protocol_sha256") ==
            stage.get("mixed_mpc_protocol_sha256") == protocol_hash(protocol), "Missing or mismatched mixed protocol digest")
    accepted = load_accepted_response_model(task)
    binding = accepted["binding"]
    model_archives = verify_model_archive(launch_dir, metadata, binding)
    provenance = metadata["provenance"]
    require(validate_stage_gates(task, provenance) == provenance.get("stage_gates"),
            "Executed stage gates differ from their independently checked artifacts")
    controller_details = verify_controller_details(metadata, task, protocol, accepted["model"], pid)
    for key in ("model_sha256", "source_urdf_sha256"):
        require(metadata.get(key) == provenance.get(key) == stage.get(key), "Physical model identity mismatch")
    require(metadata["model_sha256"] == condition["model_sha256"], "Frozen condition belongs to another physical model")
    artifacts = fixed_audit.source_and_launch_evidence(launch_dir, folder, metadata, task)
    artifacts.update(model_archives)
    artifacts.update(fixed_audit.verified_artifacts(folder, stage))
    documents = {key: read_json(folder/name) for key, name in {
        "session": "diagnostic_parameter_session.json", "journal": "diagnostic_restore_journal_completed.json",
        "before": "diagnostic_parameters_before.json", "applied": "diagnostic_parameters_applied.json",
        "restored": "diagnostic_parameters_restored.json", "px4_before": "px4_parameters_before.json", "px4_after": "px4_parameters_after.json"}.items()}
    parameters = fixed_audit.verify_parameter_session(documents, metadata, condition, pid)
    observer = read_json(paths["diagnostic_odometry.json"])
    require(observer.get("purpose") == "research" and observer.get("diagnostic_only") is False and observer.get("ideal_data") is True
            and observer.get("requested_rate_hz") == 50. and observer.get("gaussian_noise") == 0.
            and observer.get("model_snapshot_unmodified") is True and observer.get("physical_model_changes") == []
            and all(observer.get(key) is True for key in ("aiding_confirmed", "input_stream_confirmed", "observer_loaded")),
            "Research ideal-EV observer did not complete")
    fixed_audit.verify_condition(observer, condition, "observer")
    artifacts.update(fixed_audit.verified_artifacts(folder, observer))
    for document, filename in ((documents["session"], "diagnostic_parameters.py"), (observer, "diagnostic_odometry.py")):
        require(document.get("source_sha256") == provenance["source_files_sha256"].get("simulations/x650_wholebody_mpc_v1/"+filename),
                "Executed observer/session source differs from archive")
    for name in ("estimated_feedback.py", "run_case.py"):
        source = ROOT/"simulations/x650_wholebody_mpc_v1"/name
        require(provenance["source_files_sha256"].get(source.relative_to(ROOT).as_posix()) == precision.sha(source),
                "Upper feedback/command audit uses a different source version: "+name)
    reference = read_json(paths["hover_reference.json"])
    target = precision.verify_reference(reference, precision.DEFAULT_REFERENCE_SOURCE)
    require(reference.get("frame_mode") == "raw_ev_world" and reference["world"]["source_sha256"] == condition["world_sdf_sha256"],
            "Upper raw-EV reference/origin contract differs")
    require(np.max(abs(vector(metadata["reference_base_enu_m"], 3, "upper reference anchor")-target)) <= 1e-8,
            "Upper anchor differs from the frozen world target")
    feedback_document = read_json(paths["estimated_feedback.json"])
    require(feedback_document == metadata.get("estimated_feedback")
            and feedback_document.get("kind") == "x650_px4_estimated_feedback"
            and feedback_document.get("schema_version") == 2
            and feedback_document.get("profile") == "px4_estimated_acceleration_raw_ev_world_v2"
            and feedback_document.get("frame_mode") == "raw_ev_world"
            and feedback_document.get("truth_feedback") is False and feedback_document.get("truth_clock") is False
            and feedback_document.get("reference") == reference
            and feedback_document.get("max_age_s") == {"state": .10, "flags": 1.25, "joint": .25, "wall": 4., "flags_wall": 10.},
            "Frozen estimated-feedback binding/age contract differs from executed metadata")
    window = [float(metadata[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    require(metadata.get("requested_duration_s") == task["duration_s"]
            and np.isfinite(window).all() and abs(window[1]-window[0]-task["duration_s"]) <= .05, "Mixed stage lacks full declared measurement")
    manifest = read_json(paths["native_trace_manifest.json"])
    require(manifest.get("complete") is True and manifest.get("sha256") == precision.sha(paths["native_trace.csv"]), "Native capture checksum failed")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(manifest.get(key) == metadata[key], "Native capture belongs to another physical model")
    precision.capture_window_covers(manifest, window)
    native = precision.native_states(paths["native_trace.csv"], window)
    if ulog_loader is None:
        from pyulog import ULog
        ulog_loader = ULog
    ulog = ulog_loader(str(paths["px4.ulg"]))
    configuration = fixed_audit.effective_configuration(ulog, native, reference, window, fixed_audit.records(documents["applied"]))
    control = rows(paths["control_trace.csv"])
    with paths["feedback_trace.jsonl"].open(encoding="utf-8") as stream:
        feedback = [json.loads(line) for line in stream if line.strip()]
    feedback_report, snapshots = feedback_check_mixed(control, feedback, reference, window)
    acceleration = audit_mixed_acceleration_feedback(control, feedback, ulog,
        load_native_acceleration(paths["native_trace.csv"], window), window)
    require(snapshots[0]["extra_reset_counters"] == feedback_document.get("extra_reset_counters_at_bind"),
            "Estimator velocity/quaternion counters changed since feedback binding")
    for topic, keys, expected in (("vehicle_local_position", ("vxy_reset_counter", "vz_reset_counter"), snapshots[0]["extra_reset_counters"]),
                                  ("vehicle_attitude", ("quat_reset_counter",), snapshots[0]["extra_reset_counters"])):
        dataset = precision.dataset(ulog, topic)
        held = precision.resample(precision.stamp(dataset), precision.fields(dataset, keys), native["t"], .024, hold=True)
        for index, key in enumerate(keys):
            require(np.all(held[:, index] == expected[key]), "Full ULog velocity/quaternion reset changed: "+key)
    commands = upper_commands(ulog, native, reference, control, window, pid, response=False)
    require(np.max(abs(fields(selected_rows(control, window)[0], "command_acceleration_", "xyz"))) <= 1e-12,
            "Mixed stage sent nonzero acceleration FF")
    proxy = velocity_proxy(ulog, native, reference)
    flight = flight_states(paths["flight_samples.csv"], window)
    ground = read_json(paths["native_ground.json"])
    fixed_audit.verify_condition(ground, condition, "ground")
    require(ground.get("diagnostic_only") is False and ground.get("ground_truth_aided") is True
            and ground.get("eligible_as_stage_gate") is False and ground.get("acceptance_pending") == "full_ulog_audit", "Ground evidence is not original provisional research")
    ground_report = fixed_audit.native_ground_check(folder, ground)
    artifacts.update(fixed_audit.verified_artifacts(folder, ground))
    physical = joint_audit.native_arrays(rows(paths["native_trace.csv"]), window[0], window[1]-window[0], target)
    position = precision.metrics(native["t"], native["position"]-target, "m")
    measured_control, _ = selected_rows(control, window)
    joint_commands = fields(measured_control, "qd", ("1_cmd", "2_cmd"))
    base_commands = fields(measured_control, "base_velocity_target_", "xyz")
    velocity_limits = np.asarray(protocol["common_controller_config"]["velocity_limit"])
    common_checks = {
        "tilt_at_most_8deg": float(native["tilt"].max()) <= protocol["acceptance"]["tilt_max_deg"],
        "joint_commands_within_common_caps": bool(np.all(abs(joint_commands) <= velocity_limits[3:]+1e-8)),
        "base_commands_within_common_caps": bool(np.all(abs(base_commands) <= velocity_limits[:3]+1e-8))}
    stage_report = mixed_stage_metrics(folder, metadata, task, protocol, window, physical, common_checks)
    chain_execution = None
    if "prediction_chain" in protocol["mpc_controller_config"]:
        from audit_x650_prediction_chain import audit_prediction_chain_execution
        chain_execution = audit_prediction_chain_execution(folder, control, metadata, protocol, window)
        artifacts.update(chain_execution["artifacts"])
    artifacts.update({str(path): precision.sha(path) for path in paths.values()})
    for source in (Path(__file__), Path(mixed_feedback.__file__), Path(upper.__file__), Path(fixed_audit.__file__), Path(precision.__file__), Path(joint_audit.__file__),
                   Path(task["pid_profile"]), Path(task["research_profile"]["world_sdf"])):
        artifacts[str(source.resolve())] = precision.sha(source)
    report = {"kind": AUDIT_KIND, "schema_version": 3, "source_run": folder.name,
        "source_run_directory": str(folder), "stage": task["stage"], "simulation_backend": "gazebo_px4", "complete": True,
        "passed": stage_report["passed"], "research_acceptance_verified": stage_report["passed"],
        "operating_condition": condition, "operating_condition_sha256": canonical_hash(condition), "pid_parameters_sha256": pid_hash,
        "model_sha256": metadata["model_sha256"], "source_urdf_sha256": metadata["source_urdf_sha256"],
        "feedback_contract": condition["feedback_contract"], "control_interface": condition["control_interface"],
        "ground_truth_aided": True, "full_window_ulog_verified": True, "parameter_restore_verified": True,
        "upper_command_verified": True, "feedback_contract_verified": True, "window_sim_s": window,
        "controller": task["controller"], "source_stage": task["stage"],
        "acceleration_feedforward": "zero", "controller_settings_audit": controller_details,
        "mixed_mpc_protocol": protocol, "mixed_mpc_protocol_sha256": protocol_hash(protocol),
        "accepted_response_binding": binding, "acceleration_feedback_audit": acceleration,
        **({"prediction_chain_execution_audit": chain_execution} if chain_execution is not None else {}),
        "configuration": configuration, "parameter_session": parameters, "feedback_state_audit": feedback_report,
        "upper_command_audit": commands, "feedback_velocity_proxy": proxy, "flight_state_audit": flight,
        "ground_acceptance": ground_report, "stage_acceptance": stage_report, "position_native_world_enu": position,
        "inputs_sha256": artifacts, "generator_sha256": precision.sha(__file__), "no_fitted_origin_phase_or_mean": True,
        "gps_performance_claim": False, "hardware_performance_claim": False, "external_force_rejection_claim": False,
        "mpc_performance_claim": task["controller"] == "mpc" and stage_report["passed"], "original_evidence_rewritten": False}
    return report, artifacts


def archive_audit_sources(output, report, artifacts):
    """Certificates bind immutable source bytes, not evolving worktree files."""
    import accepted_response_model, mixed_mpc_protocol, task as task_module
    source_paths = {Path(name).resolve() for name in artifacts if Path(name).suffix == ".py"}
    source_paths.update(Path(module.__file__).resolve() for module in
                        (task_module, mixed_mpc_protocol, accepted_response_model))
    for package in (ROOT/"control_packages/x650_wholebody_v1/x650_wholebody",):
        source_paths.update(path.resolve() for path in package.glob("*.py"))
    manifest = {}
    archive = output/"audit_sources.zip"
    with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED) as zipped:
        for path in sorted(source_paths):
            require(path.is_relative_to(ROOT) and path.is_file(), "Audit source outside declared repository")
            member, raw = path.relative_to(ROOT).as_posix(), path.read_bytes()
            digest = hashlib.sha256(raw).hexdigest()
            if str(path) in artifacts:
                require(artifacts[str(path)] == digest, "Audit source changed while freezing its bytes")
            zipped.writestr(member, raw)
            manifest[member] = {"source_path": str(path), "sha256": digest}
    retained = {name: digest for name, digest in artifacts.items() if Path(name).resolve() not in source_paths}
    retained[str(archive)] = precision.sha(archive)
    report["analysis_source_archive"] = {"path": str(archive), "sha256": precision.sha(archive), "members": manifest}
    report["inputs_sha256"] = retained
    return retained


def audit(run_dir, launch_dir, output, *, ulog_loader=None):
    output = Path(output).resolve()
    require(not output.exists(), "Use a new upper audit directory; original evidence is never overwritten")
    output.mkdir(parents=True)
    try:
        report, artifacts = analyze(run_dir, launch_dir, ulog_loader=ulog_loader)
        artifacts = archive_audit_sources(output, report, artifacts)
    except Exception as exc:
        failure = {"kind": AUDIT_KIND, "complete": False, "passed": False,
            "research_acceptance_verified": False, "error": repr(exc), "original_evidence_rewritten": False,
            "generator_sha256": precision.sha(__file__)}
        (output/AUDIT_FILENAME).write_text(json.dumps(failure, indent=2)+"\n", encoding="utf-8")
        raise
    report_path = output/AUDIT_FILENAME
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    require(report["passed"], "Upper physical stage failed its predeclared criteria; no certificate issued")
    artifacts = {**artifacts, str(report_path): precision.sha(report_path)}
    common = {key: report[key] for key in ("operating_condition", "operating_condition_sha256", "pid_parameters_sha256",
        "model_sha256", "source_urdf_sha256", "feedback_contract", "control_interface", "full_window_ulog_verified",
        "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified",
        "mixed_mpc_protocol", "mixed_mpc_protocol_sha256", "controller", "source_stage", "accepted_response_binding",
        "acceleration_feedforward")}
    common.update(kind="x650_native_stage_evidence", schema_version=3, simulation_backend="gazebo_px4", complete=True,
        passed=True, success=True, landed_and_disarmed=True, diagnostic_only=False, ground_truth_aided=True,
        eligible_as_stage_gate=True, research_acceptance_verified=True, source_run=report["source_run"],
        source_run_directory=report["source_run_directory"], artifacts=artifacts, gps_performance_claim=False,
        hardware_performance_claim=False, acceptance_source="Independent full native/ULog/PVA/feedback/restoration audit; original provisional records retained")
    for name, stage, metrics in (("research_stage.json", report["stage"], report["stage_acceptance"]),
                                  ("research_ground.json", "native_ground", report["ground_acceptance"])):
        (output/name).write_text(json.dumps({**common, "stage": stage, "acceptance": metrics}, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    if report["controller"] == "pid_ff":
        (output/"pid_baseline.json").write_text(json.dumps({**common, "stage": "pid_baseline",
            "source_stage": report["stage"], "acceptance": report["stage_acceptance"]}, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--launch-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir, args.launch_dir, args.output)
    print(json.dumps({"passed": report["passed"], "stage": report["stage"], "source_run": report["source_run"],
                      "output": str(args.output), "feedback_velocity_proxy": report["feedback_velocity_proxy"]}, indent=2))


if __name__ == "__main__":
    main()
