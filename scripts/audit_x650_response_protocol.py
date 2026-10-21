#!/usr/bin/env python3
"""Full native/ULog/PVA/feedback audit of the frozen fresh 60-second protocol.

The old audit remains byte-for-byte intact. Its shared numerical and provenance
checks are reused explicitly; this module owns the new schedule and duration.
"""
from __future__ import annotations
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit_x650_upper_stage as upper
from audit_x650_upper_stage import *
from simulations.x650_wholebody_mpc_v1.response_protocol import (
    validate_response_protocol, response_protocol_hash, protocol_velocity)


def verify_protocol_documents(task, metadata, stage):
    protocol = validate_response_protocol(task.get("response_probe_protocol"))
    require(protocol is not None, "Frozen response protocol missing")
    digest = response_protocol_hash(protocol)
    for label, doc in (("metadata", metadata), ("stage", stage), ("provenance", metadata["provenance"])):
        require(doc.get("response_probe_protocol") == protocol
                and doc.get("response_probe_protocol_sha256") == digest,
                "Frozen response protocol changed: "+label)
    require(task["stage"] == "response_probe" and task["controller"] == "pid_ff",
            "Formal response only permits folded-arm pid_ff excitation")
    return protocol


def verify_protocol_commands(control, reference, window, protocol):
    selected, _ = upper.selected_rows(control, window)
    t = np.asarray([float(row["t_s"]) for row in selected])
    logical = fields(selected, "base_velocity_target_", "xyz")
    acceleration = fields(selected, "command_acceleration_", "xyz")
    require(np.max(abs(acceleration)) <= 1e-12, "Formal response requires zero acceleration feedforward")
    active = t < protocol["return_anchor_start_s"]
    expected = np.asarray([protocol_velocity(float(value), protocol) for value in t])
    require(np.any(active) and np.any(~active), "Complete excitation and anchor-return are required")
    require(np.max(abs(logical[active]-expected[active])) <= 1e-9,
            "Formal input differs from frozen pulses or zero-velocity rest segments")
    returning = np.clip(protocol["return_anchor"]["position_gain_s_inv"]*
        (np.asarray(reference["target_world_enu_m"])-fields(selected, "feedback_p", "xyz")),
        -protocol["return_anchor"]["velocity_limit_m_s"], protocol["return_anchor"]["velocity_limit_m_s"])
    require(np.max(abs(logical[~active]-returning[~active])) <= 1e-9,
            "Formal anchor return differs from its frozen controller")
    return {"verified": True, "protocol_sha256": response_protocol_hash(protocol),
            "role": protocol["role"], "prediction_window_s": protocol["prediction_window_s"],
            "anchor_return_excluded_from_model_scoring": True, "zero_acceleration_feedforward": True}


def analyze(folder, launch_dir, *, ulog_loader=None):
    folder = Path(folder).resolve(strict=True)
    names = ("stage_result.json", "controller_metadata.json", "controller_task.json", "hover_reference.json",
        "native_trace.csv", "native_trace_manifest.json", "flight_samples.csv", "native_ground.json", "native_ground.csv",
        "native_ground_manifest.json", "control_trace.csv", "feedback_trace.jsonl", "estimated_feedback.json", "px4_parameters_before.json",
        "px4_parameters_after.json", "diagnostic_parameter_session.json", "diagnostic_parameters_before.json",
        "diagnostic_parameters_applied.json", "diagnostic_parameters_restored.json", "diagnostic_restore_journal_completed.json",
        "diagnostic_odometry.json", "diagnostic_parameter_storage_save.log", "px4.ulg")
    paths = {name: folder/name for name in names}
    require(all(path.is_file() for path in paths.values()), "Missing original upper research artifact")
    stage, metadata, task = (read_json(paths[name]) for name in ("stage_result.json", "controller_metadata.json", "controller_task.json"))
    require(task == metadata.get("task") and task["stage"] == "response_probe", "Upper research task mismatch")
    protocol = verify_protocol_documents(task, metadata, stage)
    condition = condition_for_task(task)
    require(condition is not None and condition.get("research_profile") == "precision_ev_upper_v2", "Reviewed v2 condition is required")
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
    require(metadata.get("allocator_active") is (task["stage"] == "tip_hold"), "Unexpected upper allocation stage")
    provenance = metadata["provenance"]
    for key in ("model_sha256", "source_urdf_sha256"):
        require(metadata.get(key) == provenance.get(key) == stage.get(key), "Physical model identity mismatch")
    require(metadata["model_sha256"] == condition["model_sha256"], "Frozen condition belongs to another physical model")
    artifacts = fixed_audit.source_and_launch_evidence(launch_dir, folder, metadata, task)
    if protocol['role'] == 'holdout':
        launch = read_json(Path(launch_dir)/'launch.json')
        archive = launch.get('frozen_response_candidate_archive', {})
        archived = Path(archive.get('path', '')).resolve()
        digest = task.get('frozen_response_candidate_sha256')
        require(archived == (Path(launch_dir)/'frozen_response_candidate.json').resolve()
                and archived.is_file() and precision.sha(archived) == digest
                and archive.get('sha256') == digest
                and archive.get('source_path') == task.get('frozen_response_candidate'),
                'Holdout launch did not preserve the exact preflight candidate bytes')
        artifacts[str(archived)] = digest
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
            and feedback_document.get("schema_version") == 1
            and feedback_document.get("profile") == "px4_estimated_raw_ev_world_v1"
            and feedback_document.get("frame_mode") == "raw_ev_world"
            and feedback_document.get("truth_feedback") is False and feedback_document.get("truth_clock") is False
            and feedback_document.get("reference") == reference
            and feedback_document.get("max_age_s") == {"state": .10, "flags": 1.25, "joint": .25, "wall": 4., "flags_wall": 10.},
            "Frozen estimated-feedback binding/age contract differs from executed metadata")
    window = [float(metadata[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    require(task.get("duration_s") == protocol["duration_s"] and metadata.get("requested_duration_s") == protocol["duration_s"]
            and np.isfinite(window).all() and abs(window[1]-window[0]-protocol["duration_s"]) <= .05, "Upper stage lacks full frozen protocol measurement")
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
    feedback_report, snapshots = feedback_check(control, feedback, reference, window)
    require(snapshots[0]["extra_reset_counters"] == feedback_document.get("extra_reset_counters_at_bind"),
            "Estimator velocity/quaternion counters changed since feedback binding")
    for topic, keys, expected in (("vehicle_local_position", ("vxy_reset_counter", "vz_reset_counter"), snapshots[0]["extra_reset_counters"]),
                                  ("vehicle_attitude", ("quat_reset_counter",), snapshots[0]["extra_reset_counters"])):
        dataset = precision.dataset(ulog, topic)
        held = precision.resample(precision.stamp(dataset), precision.fields(dataset, keys), native["t"], .024, hold=True)
        for index, key in enumerate(keys):
            require(np.all(held[:, index] == expected[key]), "Full ULog velocity/quaternion reset changed: "+key)
    commands = upper.upper_commands(ulog, native, reference, control, window, pid, response=False)
    protocol_commands = verify_protocol_commands(control, reference, window, protocol)
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
    common_checks = {
        "tilt_at_most_8deg": float(native["tilt"].max()) <= 8.,
        "joint_commands_within_common_caps": bool(np.all(abs(joint_commands) <= np.array([.45, .35])+1e-8)),
        "base_commands_within_common_caps": bool(np.all(abs(base_commands) <= np.array([.55, .35, .45])+1e-8))}
    checks = {**common_checks, "folded_q_at_most_003rad": float(abs(physical["q"]).max()) <= .03,
              "joint_speed_at_most_005rad_s": float(abs(physical["dq"]).max()) <= .05,
              "body_within_runtime_15m_envelope": position["peak_3d"] <= 1.5}
    stage_report = {"passed": all(checks.values()), "criteria": checks,
        "scope": "Complete folded-arm velocity excitation and return; dynamic model accuracy is a separate train/holdout identification"}
    artifacts.update({str(path): precision.sha(path) for path in paths.values()})
    for source in (Path(__file__), Path(upper.__file__),
                   ROOT/"simulations/x650_wholebody_mpc_v1/response_protocol.py", Path(fixed_audit.__file__), Path(precision.__file__), Path(joint_audit.__file__),
                   Path(task["pid_profile"]), Path(task["research_profile"]["world_sdf"])):
        artifacts[str(source.resolve())] = precision.sha(source)
    report = {"kind": "x650_precision_response_protocol_audit", "schema_version": 1, "source_run": folder.name,
        "source_run_directory": str(folder), "stage": task["stage"], "simulation_backend": "gazebo_px4", "complete": True,
        "passed": stage_report["passed"], "research_acceptance_verified": stage_report["passed"],
        "operating_condition": condition, "operating_condition_sha256": canonical_hash(condition), "pid_parameters_sha256": pid_hash,
        "model_sha256": metadata["model_sha256"], "source_urdf_sha256": metadata["source_urdf_sha256"],
        "feedback_contract": condition["feedback_contract"], "control_interface": condition["control_interface"],
        "ground_truth_aided": True, "full_window_ulog_verified": True, "parameter_restore_verified": True,
        "upper_command_verified": True, "feedback_contract_verified": True, "window_sim_s": window,
        "configuration": configuration, "parameter_session": parameters, "feedback_state_audit": feedback_report,
        "upper_command_audit": commands, "response_protocol_commands": protocol_commands,
        "response_probe_protocol": protocol, "response_probe_protocol_sha256": response_protocol_hash(protocol), "feedback_velocity_proxy": proxy, "flight_state_audit": flight,
        "ground_acceptance": ground_report, "stage_acceptance": stage_report, "position_native_world_enu": position,
        "inputs_sha256": artifacts, "generator_sha256": precision.sha(__file__), "no_fitted_origin_phase_or_mean": True,
        "gps_performance_claim": False, "hardware_performance_claim": False, "external_force_rejection_claim": False,
        "mpc_performance_claim": False, "original_evidence_rewritten": False}
    return report, artifacts


def audit(run_dir, launch_dir, output, *, ulog_loader=None):
    output = Path(output).resolve()
    require(not output.exists(), "Use a new upper audit directory; original evidence is never overwritten")
    output.mkdir(parents=True)
    try:
        report, artifacts = analyze(run_dir, launch_dir, ulog_loader=ulog_loader)
    except Exception as exc:
        failure = {"kind": "x650_precision_response_protocol_audit", "complete": False, "passed": False,
            "research_acceptance_verified": False, "error": repr(exc), "original_evidence_rewritten": False,
            "generator_sha256": precision.sha(__file__)}
        (output/"upper_stage_audit.json").write_text(json.dumps(failure, indent=2)+"\n", encoding="utf-8")
        raise
    report_path = output/"upper_stage_audit.json"
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    require(report["passed"], "Upper physical stage failed its predeclared criteria; no certificate issued")
    artifacts = {**artifacts, str(report_path): precision.sha(report_path)}
    common = {key: report[key] for key in ("operating_condition", "operating_condition_sha256", "pid_parameters_sha256",
        "model_sha256", "source_urdf_sha256", "feedback_contract", "control_interface", "full_window_ulog_verified",
        "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified",
        "response_probe_protocol", "response_probe_protocol_sha256")}
    common.update(kind="x650_native_stage_evidence", schema_version=2, simulation_backend="gazebo_px4", complete=True,
        passed=True, success=True, landed_and_disarmed=True, diagnostic_only=False, ground_truth_aided=True,
        eligible_as_stage_gate=True, research_acceptance_verified=True, source_run=report["source_run"],
        source_run_directory=report["source_run_directory"], artifacts=artifacts, gps_performance_claim=False,
        hardware_performance_claim=False, acceptance_source="Independent full native/ULog/PVA/feedback/restoration audit; original provisional records retained")
    for name, stage, metrics in (("research_stage.json", report["stage"], report["stage_acceptance"]),
                                  ("research_ground.json", "native_ground", report["ground_acceptance"])):
        (output/name).write_text(json.dumps({**common, "stage": stage, "acceptance": metrics}, indent=2, allow_nan=False)+"\n", encoding="utf-8")
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
