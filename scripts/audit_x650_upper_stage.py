#!/usr/bin/env python3
"""Independent full-window acceptance of precision-EV upper-v2 commissioning.

Creates new evidence only. The old fixed-position audit and every original run
remain unchanged. Native physical velocities, PX4 state proxies, sent PVA, and
received/internal PX4 targets have distinct recorded semantics and clocks.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1",
                  ROOT/"control_packages/x650_wholebody_v1"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
import audit_x650_precision_stage as fixed_audit
import analyze_x650_hover_precision as precision
import analyze_x650_joint_probe as joint_audit
from research_condition import canonical_hash, condition_for_task, frozen_research_pid
from simulations.x650_wholebody_mpc_v1.estimated_feedback import NED_TO_ENU, FRD_TO_FLU
from x650_wholebody.model import ArmModel, rotation_matrix, tip_world

read_json, require = fixed_audit.read_json, fixed_audit.require
PROXY_AXIS_RMS_LIMIT = np.array([.015, .015, .010])
PVA_DELIVERY_MAX_S, CLOCK_BOUNDARY_S, PVA_FLOAT_TOLERANCE = .10, .008, 2e-5


def rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def vector(value, length, label):
    value = np.asarray(value, float)
    require(value.shape == (length,) and np.isfinite(value).all(), "Invalid finite vector: "+label)
    return value


def fields(records, prefix, axes="ned"):
    result = np.asarray([[float(record[prefix+axis]) for axis in axes] for record in records])
    require(result.shape == (len(records), len(axes)) and np.isfinite(result).all(), "Nonfinite recorded field: "+prefix)
    return result


def selected_rows(control, window):
    selected = [row for row in control if row["phase"] == "measure"
                and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    t = np.asarray([float(row["sim_s"]) for row in selected])
    require(len(t) >= 2 and np.isfinite(t).all() and np.all(np.diff(t) > 0)
            and t[0] <= window[0]+.05 and t[-1] >= window[1]-.05 and np.diff(t).max() <= .05+1e-9,
            "Upper command trace does not cover complete 30-second window")
    require(np.max(abs(t-np.asarray([float(row["t_s"]) for row in selected])-window[0])) <= 1e-8,
            "Upper task/simulation time origin changed")
    return selected, t


def feedback_check(control, feedback_lines, reference, window):
    selected, times = selected_rows(control, window)
    entries = [entry for entry in feedback_lines if entry.get("phase") == "measure"
               and window[0]-1e-9 <= float(entry["sim_s"]) <= window[1]+1e-9]
    require(len(entries) == len(selected), "Every control row needs exactly one feedback snapshot")
    prior, maximum_age, snapshots = {}, {}, []
    prior_joint_stamp, joint_age_range = None, [float("inf"), -float("inf")]
    bound = reference["px4_reference_at_bind"]
    first_extra = None
    for row, entry, now in zip(selected, entries, times):
        require(abs(float(entry["sim_s"])-now) <= 1e-9 and abs(float(entry["t_s"])-float(row["t_s"])) <= 1e-9,
                "Feedback/control snapshot pairing changed")
        diagnostic = entry.get("feedback")
        require(isinstance(diagnostic, dict) and diagnostic.get("truth_feedback") is False
                and diagnostic.get("truth_clock") is False and diagnostic.get("feedback_profile") == "px4_estimated_raw_ev_world_v1"
                and diagnostic.get("snapshot_protocol") == "latest_asynchronous_receipt_age_guarded"
                and abs(float(diagnostic["sim_time_s"])-now) <= 1e-8, "Upper feedback is not the reviewed PX4 receipt snapshot")
        callback_map = diagnostic["callbacks"]
        require(set(callback_map) == {"position", "attitude", "angular_velocity", "estimator_flags", "joints", "clock"},
                "Missing independently aged estimator callbacks")
        for name, callback in callback_map.items():
            seq = callback["sequence"]
            require(isinstance(seq, int) and not isinstance(seq, bool) and seq >= 1, "Invalid feedback sequence")
            received_sim, received_wall, age, wall_age = (float(callback[key]) for key in
                ("received_sim_s", "received_wall_s", "sim_age_s", "wall_age_s"))
            # Pinned PX4 EKF2 publishes status flags once a second or on a
            # change. DDS's 5 Hz configuration is only an upper rate bound.
            limit = 1.25 if name == "estimator_flags" else .25 if name == "joints" else .10
            wall_limit = 10. if name == "estimator_flags" else 4.
            require(np.isfinite([received_sim, received_wall, age, wall_age]).all()
                    and abs(now-received_sim-age) <= 1e-8 and -1e-9 <= age <= limit+1e-9 and -1e-6 <= wall_age <= wall_limit,
                    "Invalid/stale callback receipt age: "+name)
            previous = prior.get(name)
            if previous:
                require(seq >= previous[0] and received_sim >= previous[1]-1e-9 and received_wall >= previous[2]-1e-9,
                        "Feedback callback sequence/time regressed: "+name)
                if seq == previous[0]:
                    require(received_sim == previous[1] and received_wall == previous[2], "Held receipt mutated without callback")
            prior[name] = (seq, received_sim, received_wall)
            maximum_age[name] = max(maximum_age.get(name, 0.), age)
        require(abs(float(diagnostic["state_stamp_s"])-callback_map["angular_velocity"]["received_sim_s"]) <= 1e-9,
                "Controller state derivative stamp is not angular callback receipt time")
        joint_stamp, joint_age = float(diagnostic["joint_stamp_s"]), float(diagnostic["joint_source_age_s"])
        require(np.isfinite([joint_stamp, joint_age]).all() and abs(now-joint_stamp-joint_age) <= 1e-8
                and abs(joint_age) <= .25+1e-9 and (prior_joint_stamp is None or joint_stamp >= prior_joint_stamp-1e-9),
                "Joint source timestamp is stale, inconsistent or regressed")
        prior_joint_stamp = joint_stamp
        joint_age_range = [min(joint_age_range[0], joint_age), max(joint_age_range[1], joint_age)]
        require(int(float(row["feedback_snapshot_sequence"])) == callback_map["position"]["sequence"], "Control used another feedback sequence")
        snapshot, state = diagnostic["px4_snapshot"], diagnostic["control_state"]
        require(np.max(abs(vector(state["q_rad"], 2, "measured joints")-fields([row], "q", "12")[0])) <= 1e-9
                and np.max(abs(vector(state["dq_rad_s"], 2, "measured joint rates")-fields([row], "qd", "12")[0])) <= 1e-9,
                "Controller joint feedback differs from the paired snapshot")
        pn, vn = vector(snapshot["position_ned_m"], 3, "PX4 p"), vector(snapshot["velocity_ned_m_s"], 3, "PX4 v")
        pw = NED_TO_ENU@pn
        pw[2] += bound["ref_alt"]
        vw = NED_TO_ENU@vn
        require(np.max(abs(vector(state["position_enu_m"], 3, "feedback p")-pw)) <= 1e-9
                and np.max(abs(vector(state["velocity_enu_m_s"], 3, "feedback v")-vw)) <= 1e-9,
                "PX4 feedback world conversion does not use frozen raw-EV origin")
        for prefix, expected, axes in (("feedback_position_", pn, "ned"), ("feedback_velocity_", vn, "ned"),
                                      ("feedback_p", pw, "xyz"), ("feedback_v", vw, "xyz")):
            require(np.max(abs(fields([row], prefix, axes)[0]-expected)) <= 1e-9, "Control row did not use logged snapshot: "+prefix)
        quaternion = vector(snapshot["attitude_wxyz"], 4, "PX4 q")
        require(abs(np.linalg.norm(quaternion)-1.) <= 1e-3, "Nonunit PX4 attitude")
        rotation = np.asarray(state["rotation_flu_to_enu"], float)
        require(rotation.shape == (3, 3) and np.isfinite(rotation).all()
                and np.max(abs(rotation-NED_TO_ENU@rotation_matrix(quaternion)@FRD_TO_FLU)) <= 1e-9,
                "PX4 attitude conversion changed")
        require(np.max(abs(vector(state["omega_flu_rad_s"], 3, "feedback rates")
                    -FRD_TO_FLU@vector(snapshot["angular_velocity_frd_rad_s"], 3, "PX4 rates"))) <= 1e-9,
                "PX4 angular-rate conversion changed")
        require(all(snapshot["valid"].get(key) is True for key in
                    ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control"))
                and snapshot.get("dead_reckoning") is False, "PX4 feedback valid/dead-reckoning guard failed")
        flags = snapshot["estimator_flags"]
        require(all(flags.get(key) is bool(value) for key, value in fixed_audit.EV_FLAGS.items())
                and flags.get("cs_inertial_dead_reckoning") is False and flags.get("cs_wind_dead_reckoning") is False,
                "PX4 feedback fusion/dead-reckoning flags changed")
        origin = snapshot["origin"]
        for key, tolerance in (("ref_lat", 1e-11), ("ref_lon", 1e-11), ("ref_alt", 1e-6)):
            require(abs(float(origin[key])-bound[key]) <= tolerance, "Feedback origin changed: "+key)
        for key in ("xy_global", "z_global", "xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
            require(origin[key] == bound[key], "Feedback origin/reset changed: "+key)
        extra = snapshot["extra_reset_counters"]
        require(set(extra) == {"vxy_reset_counter", "vz_reset_counter", "quat_reset_counter"}, "Extra reset counters missing")
        if first_extra is None:
            first_extra = extra.copy()
        require(extra == first_extra, "Velocity/quaternion reset in feedback window")
        for topic in ("position", "attitude", "angular_velocity", "estimator_flags"):
            raw_times = diagnostic["px4_raw_timestamps"][topic]
            require(set(raw_times) == {"timestamp", "timestamp_sample"}
                    and all(isinstance(value, int) and not isinstance(value, bool) and value >= 0
                            for value in raw_times.values()), "Raw PX4 diagnostic timestamps invalid")
        snapshots.append(snapshot)
    return {"verified": True, "feedback_contract": "px4_estimator_receipt_enu_v1", "sample_count": len(selected),
            "maximum_callback_receipt_age_s": maximum_age, "extra_reset_counters": first_extra,
            "joint_source_age_range_s": joint_age_range,
            "truth_feedback": False, "truth_clock": False,
            "timestamp_semantics": "Asynchronous held PX4 callbacks; individual simulation receipt ages; DDS epochs are diagnostic only"}, snapshots


def trajectory_matrix(ulog):
    dataset = precision.dataset(ulog, "trajectory_setpoint")
    names = tuple(f"{prefix}[{index}]" for prefix in ("position", "velocity", "acceleration") for index in range(3))+("yaw",)
    return precision.stamp(dataset), precision.fields(dataset, names)


def upper_commands(ulog, native, reference, control, window, pid, *, response):
    selected, times = selected_rows(control, window)
    require(all(row.get("pva_sent") in ("True", "true", "1") and int(float(row["sent_coordinate_frame"])) == 1
                and int(float(row["sent_type_mask"])) == 2048 for row in selected), "Upper stage must actually send complete local-NED PVA")
    p, v, a = (fields(selected, "sent_"+part+"_target_") for part in ("position", "velocity", "acceleration"))
    yaw = np.asarray([float(row["sent_yaw_ned"]) for row in selected])
    require(np.isfinite(yaw).all() and np.max(abs(yaw-float(reference["fixed_yaw_ned_rad"]))) <= 1e-9,
            "Upper commands changed the bound fixed heading")
    fb = fields(selected, "feedback_position_")
    logical = fields(selected, "base_velocity_target_", "xyz")
    gains = np.array([pid["MPC_XY_P"], pid["MPC_XY_P"], pid["MPC_Z_P"]])
    reconstructed = gains*(p-fb)+v
    require(np.max(abs(reconstructed-logical@NED_TO_ENU.T)) <= 1e-8,
            "Sent PVA does not reconstruct logical velocity from the same feedback snapshot")
    require(np.max(abs(a-fields(selected, "command_acceleration_", "xyz")@NED_TO_ENU.T)) <= 1e-9,
            "Sent acceleration differs from the logged world command")
    if response:
        require(np.max(abs(a)) <= 1e-12, "Response identification requires zero acceleration feedforward")
        from simulations.x650_wholebody_mpc_v1.run_case import response_probe_velocity
        excited = np.asarray([response_probe_velocity(float(row["t_s"]), 30.) for row in selected])
        active = np.asarray([float(row["t_s"]) < 14. for row in selected])
        require(np.max(abs(logical[active]-excited[active])) <= 1e-9, "Response input differs from frozen world-axis pulse schedule")
        returning = np.clip(1.5*(np.asarray(reference["target_world_enu_m"])-fields(selected, "feedback_p", "xyz")), -.12, .12)
        require(np.max(abs(logical[~active]-returning[~active])) <= 1e-9,
                "Response return segment does not hold its declared fixed world anchor")
    all_sent = [row for row in control if row.get("pva_sent") in ("True", "true", "1")]
    sent_t = np.asarray([float(row["sim_s"]) for row in all_sent])
    require(np.isfinite(sent_t).all() and np.all(np.diff(sent_t) > 0), "Actual send times must be strictly increasing")
    sent = np.column_stack([*[fields(all_sent, "sent_"+part+"_target_") for part in ("position", "velocity", "acceleration")],
                            np.asarray([float(row["sent_yaw_ned"]) for row in all_sent])])
    rt, received = trajectory_matrix(ulog)
    mask = (rt >= window[0]-1e-9)&(rt <= window[1]+1e-9)
    require(np.isfinite(received[mask]).all(), "Measurement received PVA cannot contain ignored/nonfinite fields")
    require(mask.sum() >= 2 and np.diff(rt[mask]).max() <= PVA_DELIVERY_MAX_S+1e-9
            and rt[mask][0] <= window[0]+PVA_DELIVERY_MAX_S and rt[mask][-1] >= window[1]-PVA_DELIVERY_MAX_S,
            "Received trajectory setpoints do not cover complete stage")
    delays = []
    for receive_time, value in zip(rt[mask], received[mask]):
        candidates = np.flatnonzero((sent_t <= receive_time+CLOCK_BOUNDARY_S+1e-9)
                                   &(sent_t >= receive_time-PVA_DELIVERY_MAX_S-1e-9))
        matches = candidates[np.max(abs(sent[candidates]-value), axis=1) <= PVA_FLOAT_TOLERANCE] if len(candidates) else []
        require(len(matches) > 0, "A received PVA has no matching bounded-causal actual send")
        delays.append(float(receive_time-sent_t[matches[-1]]))
    for send_time, value in zip(times, np.column_stack((p, v, a, yaw))):
        candidates = np.flatnonzero((rt >= send_time-CLOCK_BOUNDARY_S-1e-9)&(rt <= send_time+PVA_DELIVERY_MAX_S+1e-9))
        # NAV_TAKEOFF/LAND outside the measurement can legitimately carry
        # ignored NaNs. Such rows never count as a matching complete PVA.
        require(len(candidates) and np.any(np.max(abs(received[candidates]-value), axis=1) <= PVA_FLOAT_TOLERANCE),
                "A measurement PVA send is absent from the received trajectory log")
    internal = precision.dataset(ulog, "vehicle_local_position_setpoint")
    it = precision.stamp(internal)
    imask = (it >= window[0]-1e-9)&(it <= window[1]+1e-9)
    query = it[imask]
    iv = precision.fields(internal, ("vx", "vy", "vz"))[imask]
    # ULog may contain repeated publication stamps; retain the final held value
    # exactly, without averaging, moving time, or creating extra observations.
    unique = np.r_[np.diff(query) > 0, True]
    query, iv = query[unique], iv[unique]
    require(len(query) >= 2 and query[0] <= window[0]+.05 and query[-1] >= window[1]-.05
            and np.diff(query).max() <= .05+1e-9, "Internal PX4 velocity targets lack complete coverage")
    # Retain bounded causal context rather than discarding a real held target
    # at the window boundary. resample validates the actual held values only;
    # a queried pre-window NAV NaN remains an error and is never filled in.
    context = rt >= window[0]-PVA_DELIVERY_MAX_S-1e-9
    held = precision.resample(rt[context], received[context], query, PVA_DELIVERY_MAX_S, hold=True)
    lpos = precision.dataset(ulog, "vehicle_local_position")
    estimate_p = precision.resample(precision.stamp(lpos), precision.fields(lpos, ("x", "y", "z")), query, .024)
    received_u = gains*(held[:, :3]-estimate_p)+held[:, 3:6]
    reconstructed_error = (iv-received_u)@NED_TO_ENU.T
    reconstructed_metrics = precision.metrics(query, reconstructed_error, "m/s")
    logical_at = precision.resample(sent_t, fields(all_sent, "base_velocity_target_", "xyz"), query, .10, hold=True)
    logical_metrics = precision.metrics(query, iv@NED_TO_ENU.T-logical_at, "m/s")
    require(np.all(np.asarray(reconstructed_metrics["axis_rmse"]) <= PROXY_AXIS_RMS_LIMIT),
            "Internal PX4 target differs from received-PVA plus same-time estimated position reconstruction")
    require(np.all(np.asarray(logical_metrics["axis_rmse"]) <= PROXY_AXIS_RMS_LIMIT),
            "Internal PX4 target differs materially from original-time logical velocity, including real delivery delay")
    limits_respected = bool(np.all(np.linalg.norm(received_u[:, :2], axis=1) <= pid["MPC_XY_VEL_MAX"]+1e-6)
        and np.all(received_u[:, 2] >= -pid["MPC_Z_VEL_MAX_UP"]-1e-6)
        and np.all(received_u[:, 2] <= pid["MPC_Z_VEL_MAX_DN"]+1e-6))
    require(limits_respected, "Reconstructed received velocity exceeds recorded PX4 speed limits")
    return {"verified": True, "control_interface": "raw_ev_world_pva_v2", "measurement_sends": len(selected),
        "received_rows": int(mask.sum()), "source_snapshot_identity_max_abs_m_s": float(np.max(abs(reconstructed-logical@NED_TO_ENU.T))),
        "received_pva_matches_actual_sends": True, "all_measurement_sends_observed": True,
        "causal_delivery_allowance_s": PVA_DELIVERY_MAX_S, "clock_boundary_allowance_s": CLOCK_BOUNDARY_S,
        "pva_float_tolerance": PVA_FLOAT_TOLERANCE, "matched_delivery_range_s": [min(delays), max(delays)],
        "internal_minus_received_pva_reconstruction": reconstructed_metrics,
        "internal_minus_original_time_logical_velocity": logical_metrics,
        "axis_rms_acceptance_limit_m_s": PROXY_AXIS_RMS_LIMIT.tolist(),
        "reconstructed_velocity_within_actual_px4_limits": limits_respected,
        "saturation_scope": "Speed-bound check and internal/reconstructed residuals; no unsupported claim that every inner-loop actuator avoided saturation",
        "zero_acceleration_feedforward": bool(np.max(abs(a)) <= 1e-12),
        "time_alignment": "Original send simulation timestamps and raw ULog HRT; no estimated time shift or delay compensation"}


def velocity_proxy(ulog, native, reference):
    lpos = precision.dataset(ulog, "vehicle_local_position")
    values = precision.resample(precision.stamp(lpos), precision.fields(lpos, ("vx", "vy", "vz")), native["t"], .024)
    estimated = values@NED_TO_ENU.T
    metrics = precision.metrics(native["t"], estimated-native["velocity"], "m/s")
    passed = bool(np.all(np.asarray(metrics["axis_rmse"]) <= PROXY_AXIS_RMS_LIMIT))
    require(passed, "PX4-EV velocity proxy exceeds predeclared per-axis RMS error bounds")
    return {"verified": True, "passed": True, "metrics": metrics, "axis_rms_limit_m_s": PROXY_AXIS_RMS_LIMIT.tolist(),
        "scope": "PX4 estimator velocity proxy for native physical velocity; peaks are reported without an added peak gate",
        "source_clock": "Raw ULog vehicle_local_position timestamp_sample at scale 1e-6; native physics simulation time",
        "resampling": "Bounded interpolation at original native timestamps; no fitted offset, phase or delay"}


def flight_states(path, window):
    selected = [row for row in rows(path) if row["phase"] == "measure"
                and window[0]-1e-9 <= float(row["sim_time_s"]) <= window[1]+1e-9]
    t = np.asarray([float(row["sim_time_s"]) for row in selected])
    require(len(t) >= 2 and t[0] <= window[0]+.06 and t[-1] >= window[1]-.06
            and np.all(np.diff(t) > 0) and np.diff(t).max() <= .06+1e-9,
            "Measurement flight-state log lacks full coverage")
    require(all(row["failsafe"] == "0" and row["armed"] == "1" and row["landed"] == "0" and row["nav_state"] == "14"
                for row in selected), "Upper measurement was not continuously armed/airborne/offboard without failsafe")
    return {"verified": True, "samples": len(t), "maximum_interval_s": float(np.diff(t).max())}


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
    require(task == metadata.get("task") and task["stage"] in ("response_probe", "tip_hold"), "Upper research task mismatch")
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
    require(task.get("duration_s") == 30. and metadata.get("requested_duration_s") == 30.
            and np.isfinite(window).all() and abs(window[1]-window[0]-30.) <= .05, "Upper stage lacks full 30-second measurement")
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
    commands = upper_commands(ulog, native, reference, control, window, pid, response=task["stage"] == "response_probe")
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
    if task["stage"] == "response_probe":
        checks = {**common_checks, "folded_q_at_most_003rad": float(abs(physical["q"]).max()) <= .03,
                  "joint_speed_at_most_005rad_s": float(abs(physical["dq"]).max()) <= .05,
                  "body_within_runtime_15m_envelope": position["peak_3d"] <= 1.5}
        stage_report = {"passed": all(checks.values()), "criteria": checks,
            "scope": "Complete folded-arm velocity excitation and return; dynamic model accuracy is a separate train/holdout identification"}
    else:
        require(metadata.get("folded_before_landing") is True
                and metadata["physical_profile"].get("source_report_sha256") == provenance.get("preparation_report_sha256"),
                "Tip commissioning requires verified folded landing and hash-bound physical profile")
        model = ArmModel(**metadata["physical_profile"])
        center = tip_world(metadata["reference_base_enu_m"], metadata["reference_rotation"], [0., 0.], model)
        tip_metrics = precision.metrics(window[0]+physical["t"], physical["tip"]-center, "m")
        require(all(np.max(abs(np.asarray([float(row["tip_target_world_"+axis+"_m"]) for axis in "xyz"])-center)) <= 1e-8
                    for row in rows(paths["flight_samples.csv"]) if row["phase"] == "measure"), "Tip-hold fixed target differs from independently reconstructed anchor")
        checks = {**common_checks, "tip_rms_at_most_005m": tip_metrics["rmse_3d"] <= .05,
                  "tip_peak_at_most_010m": tip_metrics["peak_3d"] <= .10,
                  "physical_joint_limits": bool(np.all(physical["q"] >= model.joint_lower) and np.all(physical["q"] <= model.joint_upper))}
        stage_report = {"passed": all(checks.values()), "criteria": checks, "native_tip_error": tip_metrics,
            "scope": "Fixed endpoint upper-PID commissioning, without injected force disturbance or MPC performance claim"}
    artifacts.update({str(path): precision.sha(path) for path in paths.values()})
    for source in (Path(__file__), Path(fixed_audit.__file__), Path(precision.__file__), Path(joint_audit.__file__),
                   Path(task["pid_profile"]), Path(task["research_profile"]["world_sdf"])):
        artifacts[str(source.resolve())] = precision.sha(source)
    report = {"kind": "x650_precision_upper_stage_audit", "schema_version": 2, "source_run": folder.name,
        "source_run_directory": str(folder), "stage": task["stage"], "simulation_backend": "gazebo_px4", "complete": True,
        "passed": stage_report["passed"], "research_acceptance_verified": stage_report["passed"],
        "operating_condition": condition, "operating_condition_sha256": canonical_hash(condition), "pid_parameters_sha256": pid_hash,
        "model_sha256": metadata["model_sha256"], "source_urdf_sha256": metadata["source_urdf_sha256"],
        "feedback_contract": condition["feedback_contract"], "control_interface": condition["control_interface"],
        "ground_truth_aided": True, "full_window_ulog_verified": True, "parameter_restore_verified": True,
        "upper_command_verified": True, "feedback_contract_verified": True, "window_sim_s": window,
        "configuration": configuration, "parameter_session": parameters, "feedback_state_audit": feedback_report,
        "upper_command_audit": commands, "feedback_velocity_proxy": proxy, "flight_state_audit": flight,
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
        failure = {"kind": "x650_precision_upper_stage_audit", "complete": False, "passed": False,
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
        "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")}
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
