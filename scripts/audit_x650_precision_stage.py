#!/usr/bin/env python3
"""Postflight acceptance for new precision-EV hover and joint commissioning.

Only creates a new audit directory. Runner evidence remains provisional and is
never rewritten. A completed native flight, full-window ULog, exact applied and
restored typed parameters, and the new operating-condition contract are all
required before a separately named research stage certificate can be issued.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for directory in (Path(__file__).resolve().parent, ROOT/"simulations/x650_wholebody_mpc_v1"):
    sys.path.insert(0, str(directory))
import analyze_x650_hover_precision as precision
import analyze_x650_joint_probe as joint_audit
from diagnostic_parameters import PRECISION_PROFILE, PID_CANDIDATES, PRECISION_REQUIRED, load_parameter_evidence
from research_condition import canonical_hash, condition_for_task, frozen_research_pid

CONTROL_NAMES = tuple(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
EXPECTED_WRITES = {**PRECISION_PROFILE, **PID_CANDIDATES["velocity_high"]}
EV_FLAGS = {"cs_ev_pos": 1, "cs_ev_vel": 1, "cs_ev_hgt": 1,
            "cs_gnss_pos": 0, "cs_gnss_vel": 0, "cs_gps_hgt": 0, "cs_baro_hgt": 0}


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require(value, message):
    if not value:
        raise ValueError(message)


def records(values):
    """Decode actual PARAM_VALUE bytes; never trust a copied numeric field."""
    require(isinstance(values, dict) and bool(values), "Missing typed parameter records")
    result = {}
    for name, record in values.items():
        kind, hexadecimal = record.get("param_type"), record.get("raw_value_hex")
        require(kind in (6, 9) and isinstance(hexadecimal, str), "Invalid typed record: "+name)
        raw = bytes.fromhex(hexadecimal)
        require(len(raw) == 4, "Invalid parameter byte count: "+name)
        value = struct.unpack("<i" if kind == 6 else "<f", raw)[0]
        require(math.isfinite(value) and not isinstance(record.get("value"), bool)
                and value == record.get("value"), "Parameter bytes/value disagree: "+name)
        result[name] = {"param_type": kind, "raw_value_hex": raw.hex(), "value": value}
    return result


def expected_record(name, value):
    kind = 6 if name in ("EKF2_EV_CTRL", "EKF2_EV_NOISE_MD", "EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "EKF2_HGT_REF") else 9
    raw = struct.pack("<i" if kind == 6 else "<f", value)
    return {"param_type": kind, "raw_value_hex": raw.hex(),
            "value": struct.unpack("<i" if kind == 6 else "<f", raw)[0]}


def verify_condition(document, condition, label):
    require(document.get("operating_condition") == condition
            and document.get("operating_condition_sha256") == canonical_hash(condition),
            "Operating condition mismatch: "+label)


def verify_parameter_session(documents, metadata, condition, expected_pid):
    """Applied values are flight values; restored values only certify cleanup."""
    session, journal = documents["session"], documents["journal"]
    require(session.get("purpose") == "research" and session.get("diagnostic_only") is False,
            "A new research-purpose parameter session is required")
    verify_condition(session, condition, "parameter session")
    require(journal.get("purpose") == "research" and journal.get("diagnostic_only") is False,
            "Completed restore journal is not from a research-purpose session")
    verify_condition(journal, condition, "restore journal")
    require(session.get("mutable_parameters") == EXPECTED_WRITES
            and journal.get("allowlist") == EXPECTED_WRITES, "The exact 11-parameter write allowlist is required")
    for key in ("applied_verified", "restored_verified", "runtime_restored_verified", "restore_persistent_storage_verified"):
        require(session.get(key) is True, "Parameter session verification missing: "+key)
    before, applied, restored = (records(documents[key]) for key in ("before", "applied", "restored"))
    require(set(before) == set(applied) == set(restored), "Parameter readback scopes differ")
    require(before == restored == records(session.get("restore_post_save_readback"))
            == records(journal.get("restored")), "Exact typed parameter restoration failed")
    for key, value in (("before", before), ("applied", applied), ("restored", restored)):
        require(records(session.get(key)) == value, "Session/readback file mismatch: "+key)
    require(records(journal.get("original")) == before, "Restore journal original values differ")
    require(journal.get("status") == "restored_readback_and_synchronous_storage_save_verified"
            and journal.get("restored_verified") is True, "Completed restoration journal missing")
    for saved in (session.get("restore_storage_save", {}), journal.get("storage_save", {})):
        require(saved.get("returncode") == 0 and saved.get("synchronous_default_save") is True,
                "Synchronous restored parameter storage save was not verified")
    for name, original in before.items():
        wanted = expected_record(name, EXPECTED_WRITES[name]) if name in EXPECTED_WRITES else original
        require(applied[name] == wanted, "Unexpected applied/protected parameter: "+name)
    for name, value in PRECISION_REQUIRED.items():
        require(applied.get(name) == expected_record(name, value), "Nonzero EV/IMU measurement offset or delay: "+name)
    require(set(EXPECTED_WRITES) <= set(before), "Write allowlist is absent from readbacks")
    successful = {"apply": set(), "restore": set()}
    attempts = session.get("write_attempts", [])
    require(isinstance(attempts, list) and attempts, "Missing actual write attempts")
    for entry in attempts:
        name, phase = entry.get("name"), entry.get("phase")
        require(name in EXPECTED_WRITES and phase in successful, "Out-of-scope parameter write")
        wanted = applied[name] if phase == "apply" else before[name]
        require(records({name: entry.get("desired", {})})[name] == wanted, "Write target differs from reviewed applied/restore values")
        if entry.get("verified") is True:
            require(records({name: entry.get("readback", {})})[name] == wanted, "Successful write readback differs")
            successful[phase].add(name)
    require(all(names == set(EXPECTED_WRITES) for names in successful.values()),
            "Not all 11 parameters have successful application and restoration")
    require(len(CONTROL_NAMES) == 42 and set(CONTROL_NAMES) <= set(before), "Missing canonical 42 control parameters")
    for label in ("px4_before", "px4_after"):
        snapshot = documents[label]
        require(snapshot.get("contract_matches") is True and not snapshot.get("error"), "Canonical parameter contract failed: "+label)
        require(snapshot.get("model_sha256") == condition.get("model_sha256"), "Canonical parameter snapshot belongs to another model: "+label)
        raw = records(snapshot["parameters"])
        require({name: raw[name] for name in CONTROL_NAMES} == {name: before[name] for name in CONTROL_NAMES},
                "Canonical 42 boot/restored parameters differ: "+label)
    actual = {name: float(applied[name]["value"]) for name in CONTROL_NAMES}
    require(actual == expected_pid and metadata.get("actual_pid_parameters") == actual,
            "Applied 42 control parameters differ from frozen PID / controller metadata")
    require(canonical_hash(actual) == condition["pid_parameters_sha256"], "Applied PID identity differs from operating condition")
    return {"verified": True, "applied_control_42": actual, "pid_parameters_sha256": canonical_hash(actual),
            "exact_typed_restore_verified": True, "synchronous_storage_save_verified": True,
            "write_allowlist": EXPECTED_WRITES.copy(), "write_attempt_count": len(attempts),
            "all_other_recorded_parameters_protected": True}


def effective_configuration(ulog, native, reference, window, applied):
    report = precision.full_experiment_ulog_guard(ulog, native, reference, window)
    values = report["effective_parameters"]["parameters_at_measurement_start"]
    for name, record in applied.items():
        if name.startswith(("MC_", "MPC_", "EKF2_")):
            require(name in values and math.isclose(values[name], record["value"], rel_tol=2e-6, abs_tol=1e-7),
                    "ULog effective parameter differs from applied readback: "+name)
    flags = precision.dataset(ulog, "estimator_status_flags")
    t, raw = precision.stamp(flags), precision.fields(flags, tuple(EV_FLAGS))
    desired = np.array(list(EV_FLAGS.values()))
    held = precision.resample(t, raw, native["t"], 1.5, hold=True)
    inside = (t >= window[0]-1e-9)&(t <= window[1]+1e-9)
    require(not np.any(held != desired) and not np.any(raw[inside] != desired),
            "EV/GPS/barometer fusion differs inside the full research measurement window")
    report.update(fusion_flags_entire_window=EV_FLAGS.copy(), all_applied_parameters_match_ulog=True,
                  scope="Entire declared research stage; 20 s hover or 30 s joint probe; no omitted tail")
    return report


def fixed_position_commands(ulog, native, reference, control_rows, window):
    """Check both recorded sends and every received PX4 position target."""
    selected = [row for row in control_rows if window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9
                and row["phase"] == "measure"]
    require(len(selected) >= 2, "Missing fixed-position command window")
    t = np.asarray([float(row["sim_s"]) for row in selected])
    require(t[0] <= window[0]+.05 and t[-1] >= window[1]-.05 and np.all(np.diff(t) > 0)
            and np.max(np.diff(t)) <= .05+1e-9, "Fixed-position command coverage/gap failed")
    require(all(row.get("fixed_world_position_sent") in ("True", "true", "1") for row in selected),
            "A research sample did not send the fixed position reference")
    declared = np.asarray(reference["target_local_ned_m"], float)
    sent = np.asarray([[float(row["fixed_position_target_"+axis]) for axis in "ned"] for row in selected])
    require(np.isfinite(sent).all() and np.max(abs(sent-declared)) <= 1e-9, "Sent fixed target changed")
    require(all(abs(float(row[key+axis])) <= 1e-12 for row in selected
                for key in ("base_velocity_target_", "command_acceleration_") for axis in "xyz"),
            "Upper velocity/acceleration command was active in fixed-position research mode")
    sp = precision.dataset(ulog, "vehicle_local_position_setpoint")
    st, positions = precision.stamp(sp), precision.fields(sp, ("x", "y", "z"))
    held = precision.resample(st, positions, native["t"], .3, hold=True)
    inside = (st >= window[0]-1e-9)&(st <= window[1]+1e-9)
    all_targets = np.vstack((held, positions[inside]))
    require(np.isfinite(all_targets).all() and np.max(np.linalg.norm(all_targets-declared, axis=1)) <= .002,
            "PX4 received position target differs from fixed declared target")
    require(np.max(np.ptp(all_targets, axis=0)) <= 1e-6, "PX4 received position target was not constant")
    return {"verified": True, "type_mask": 2552, "position_only_plus_yaw": True,
            "control_max_interval_s": float(np.diff(t).max()), "sent_rows": len(selected),
            "received_target_max_error_m": float(np.linalg.norm(all_targets-declared, axis=1).max()),
            "received_target_axis_range_m": np.ptp(all_targets, axis=0).tolist()}


def source_and_launch_evidence(launch_dir, folder, metadata, task):
    launch_dir = Path(launch_dir).resolve(strict=True)
    launch = read_json(launch_dir/"launch.json")
    require(launch.get("complete") is True and launch.get("runtime_closed") is True and launch.get("exit_code") == 0,
            "An owned completed runtime-closed launch is required")
    require([Path(p).resolve() for p in launch.get("run_directories", [])] == [folder],
            "Launch does not name exactly this flight")
    require(launch.get("task") == task == read_json(launch_dir/"task.json"), "Launch/flight task mismatch")
    provenance = metadata["provenance"]
    require(launch.get("provenance") == provenance, "Launch/flight source provenance mismatch")
    expected = provenance.get("source_files_sha256", {})
    require(isinstance(expected, dict) and bool(expected), "Missing flight source hashes")
    with zipfile.ZipFile(launch_dir/"controller_sources.zip") as archive:
        require(len(archive.namelist()) == len(set(archive.namelist())) and set(archive.namelist()) == set(expected),
                "Archived source scope differs from recorded provenance")
        for name, digest in expected.items():
            require(hashlib.sha256(archive.read(name)).hexdigest() == digest, "Archived source checksum mismatch: "+name)
    for name in ("hover_reference.py", "diagnostic_parameters.py", "diagnostic_odometry.py", "research_condition.py"):
        path = ROOT/"simulations/x650_wholebody_mpc_v1"/name
        require(expected.get(path.relative_to(ROOT).as_posix()) == precision.sha(path),
                "Audit contract source differs from flight archive: "+name)
    return {str(path): precision.sha(path) for path in (launch_dir/"launch.json", launch_dir/"task.json", launch_dir/"controller_sources.zip")}


def verified_artifacts(folder, document):
    values = document.get("artifacts", {})
    require(isinstance(values, dict) and bool(values), "Missing original artifact hashes")
    result = {}
    for name, digest in values.items():
        path = (folder/name).resolve(strict=True)
        require(path.is_relative_to(folder) and path.is_file() and precision.sha(path) == digest,
                "Original run artifact checksum mismatch: "+name)
        result[str(path)] = digest
    return result


def native_ground_check(folder, document):
    require(document.get("passed") is True and document.get("complete") is True
            and document.get("stage") == "native_ground", "Native ground result failed/incomplete")
    interval = np.asarray(document.get("interval_sim_s"), float)
    require(interval.shape == (2,) and np.isfinite(interval).all() and abs(np.diff(interval)[0]-5.) <= .008,
            "Ground evidence is not the complete five-second protocol")
    manifest = read_json(folder/"native_ground_manifest.json")
    require(manifest.get("complete") is True and manifest.get("file") == "native_ground.csv"
            and manifest.get("sha256") == precision.sha(folder/"native_ground.csv"), "Ground capture completion/checksum failed")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(manifest.get(key) == document.get(key), "Ground capture belongs to another model")
    with (folder/"native_ground.csv").open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if interval[0]-1e-9 <= float(r["sim_s"]) <= interval[1]+1e-9]
    data = joint_audit.native_arrays(rows, float(interval[0]), float(interval[1]-interval[0]), [0., 0., 0.])
    velocity = joint_audit.array(rows, ["base_v"+a for a in "xyz"])
    checks = {"tilt_at_most_2deg": float(data["tilt"].max()) <= 2.,
              "speed_at_most_003m_s": float(np.linalg.norm(velocity, axis=1).max()) <= .03,
              "folded_q_at_most_001rad": float(abs(data["q"]).max()) <= .01,
              "joint_speed_at_most_002rad_s": float(abs(data["dq"]).max()) <= .02}
    require(all(checks.values()), "Independent ground physical thresholds failed")
    return {"passed": True, "interval_sim_s": interval.tolist(), "criteria": checks,
            "sample_count": len(data["t"]), "maximum_native_interval_s": float(np.diff(data["t"]).max())}


def joint_stage_check(folder, metadata, window, target, control_rows):
    duration = float(metadata["requested_duration_s"])
    with (folder/"native_trace.csv").open(newline="", encoding="utf-8") as stream:
        data = joint_audit.native_arrays(list(csv.DictReader(stream)), window[0], duration, target)
    _, _, sections = joint_audit.analyze_arrays(data, duration)
    whole = sections["whole"]
    ranges = np.asarray(whole["q_actual_minmax_rad"])
    checks = {
        "both_joints_move_both_directions": bool(np.all(ranges[:, 0] <= -.035) and np.all(ranges[:, 1] >= .035)),
        "command_sign_is_correct": all(value is not None and value >= .8 for value in whole["sign_cosine"]),
        "q_tracking_rmse_at_most_002rad": max(whole["q_tracking_rmse_rad"]) <= .02,
        "q_tracking_peak_at_most_005rad": max(whole["q_tracking_peak_rad"]) <= .05,
        "q_amplitude_within_small_probe": float(np.max(abs(ranges))) <= .12,
        "joint_speeds_at_most_02rad_s": max(whole["joint_speed_abs_max_rad_s"]) <= .2,
        "body_displacement_at_most_03m": whole["body_anchor_peak_m"] <= .3,
        "tilt_at_most_15deg": whole["tilt_peak_deg"] <= 15.,
    }
    commands = joint_audit.control_audit(control_rows, window[0], duration)
    geometry = joint_audit.native_tip_audit(data, metadata["physical_profile"])
    return {"passed": all(checks.values()), "criteria": checks, "segments": sections,
            "joint_command_audit": commands, "native_tip_geometry": geometry,
            "precision_hover_threshold_applied_to_joint_probe": False,
            "scope": "Original small joint-direction/tracking/body-stability envelope; no claim of external-force rejection or whole-body endpoint compensation"}


def flight_sample_check(path, window, target):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["phase"] == "measure"
                and window[0]-1e-9 <= float(row["sim_time_s"]) <= window[1]+1e-9]
    require(len(rows) >= 2, "Missing measurement flight-state samples")
    t = np.asarray([float(row["sim_time_s"]) for row in rows])
    require(t[0] <= window[0]+.06 and t[-1] >= window[1]-.06 and np.all(np.diff(t) > 0)
            and np.diff(t).max() <= .06+1e-9, "Flight-state samples do not cover full measurement interval")
    require(all(row["failsafe"] == "0" and row["armed"] == "1" and row["landed"] == "0"
                and row["nav_state"] == "14" for row in rows), "Measurement did not remain armed/airborne/offboard without failsafe")
    targets = np.asarray([[float(row["target_world_"+a+"_m"]) for a in "xyz"] for row in rows])
    require(np.isfinite(targets).all() and np.max(abs(targets-target)) <= 1e-9, "Flight recorder fixed target changed")
    return {"verified": True, "sample_count": len(rows), "max_interval_s": float(np.diff(t).max()),
            "armed_airborne_offboard_without_failsafe": True}


def _analyze(folder, launch_dir, *, ulog_loader=None):
    paths = {name: folder/name for name in ("stage_result.json", "controller_metadata.json", "controller_task.json",
        "hover_reference.json", "native_trace.csv", "native_trace_manifest.json", "flight_samples.csv", "native_ground.json",
        "native_ground.csv", "native_ground_manifest.json", "control_trace.csv", "px4_parameters_before.json",
        "px4_parameters_after.json", "diagnostic_parameter_session.json", "diagnostic_parameters_before.json",
        "diagnostic_parameters_applied.json", "diagnostic_parameters_restored.json", "diagnostic_restore_journal_completed.json",
        "diagnostic_odometry.json", "diagnostic_parameter_storage_save.log", "px4.ulg")}
    require(all(path.is_file() for path in paths.values()), "A required original research artifact is missing")
    stage, metadata, task = (read_json(paths[name]) for name in ("stage_result.json", "controller_metadata.json", "controller_task.json"))
    require(task == metadata.get("task"), "Task file differs from flight metadata")
    condition = condition_for_task(task)
    require(condition is not None and task.get("stage") in ("hover_folded", "joint_probe"), "New precision research task required")
    _, expected_pid, pid_digest = frozen_research_pid(task)
    for label, document in (("stage", stage), ("metadata", metadata)):
        require(document.get("complete") is True and document.get("landed_and_disarmed") is True,
                "Incomplete/failed native flight cannot receive research acceptance: "+label)
        require(document.get("diagnostic_only") is False and document.get("ground_truth_aided") is True
                and document.get("eligible_as_stage_gate") is False
                and document.get("acceptance_pending") == "full_ulog_audit", "Only new provisional research results can be audited: "+label)
        verify_condition(document, condition, label)
        require(document.get("stage") == task["stage"], "Stage identity mismatch")
    require(stage.get("success") is True and stage.get("passed") is True, "Original native stage failed")
    require(metadata.get("diagnostic_parameters_restored") is True and metadata.get("diagnostic_restore_pending") is False,
            "Parameter restoration was not completed")
    require(metadata.get("allocator_active") is False and metadata.get("external_force_disturbance") is False,
            "Research v1 permits only fixed-position commissioning without upper allocation or injected forces")
    provenance = metadata["provenance"]
    require(metadata["model_sha256"] == condition["model_sha256"] == provenance["model_sha256"], "Model/condition identity mismatch")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(stage.get(key) == metadata.get(key) == provenance.get(key), "Original model provenance mismatch: "+key)
    artifacts = source_and_launch_evidence(launch_dir, folder, metadata, task)
    artifacts.update(verified_artifacts(folder, stage))
    documents = {key: read_json(folder/name) for key, name in {
        "session": "diagnostic_parameter_session.json", "journal": "diagnostic_restore_journal_completed.json",
        "before": "diagnostic_parameters_before.json", "applied": "diagnostic_parameters_applied.json",
        "restored": "diagnostic_parameters_restored.json", "px4_before": "px4_parameters_before.json", "px4_after": "px4_parameters_after.json"}.items()}
    parameter_report = verify_parameter_session(documents, metadata, condition, expected_pid)
    observer = read_json(paths["diagnostic_odometry.json"])
    require(observer.get("purpose") == "research" and observer.get("ideal_data") is True
            and observer.get("diagnostic_only") is False and observer.get("requested_rate_hz") == 50.
            and observer.get("gaussian_noise") == 0. and observer.get("model_snapshot_unmodified") is True
            and observer.get("physical_model_changes") == [] and observer.get("aiding_confirmed") is True
            and observer.get("input_stream_confirmed") is True and observer.get("observer_loaded") is True,
            "Research ideal-EV observer contract did not complete")
    verify_condition(observer, condition, "observer")
    for document, filename in ((documents["session"], "diagnostic_parameters.py"), (observer, "diagnostic_odometry.py")):
        require(document.get("source_sha256") == provenance["source_files_sha256"].get("simulations/x650_wholebody_mpc_v1/"+filename),
                "Observer/session source differs from archived executed source: "+filename)
    artifacts.update(verified_artifacts(folder, observer))
    reference = read_json(paths["hover_reference.json"])
    target = precision.verify_reference(reference, precision.DEFAULT_REFERENCE_SOURCE)
    require(reference.get("frame_mode") == "raw_ev_world"
            and reference["world"]["source_sha256"] == condition["world_sdf_sha256"], "Research coordinate/world contract differs")
    require(np.allclose(metadata.get("reference_base_enu_m"), target, rtol=0, atol=1e-9), "Fixed body anchor differs from declared target")
    duration = 20. if task["stage"] == "hover_folded" else 30.
    window = [float(metadata[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    require(task.get("duration_s") == duration and metadata.get("requested_duration_s") == duration
            and np.isfinite(window).all() and abs(window[1]-window[0]-duration) <= .05,
            "Research stage does not cover its full declared 20/30 second window")
    require("experiment_end_sim_s" not in metadata or abs(metadata["experiment_end_sim_s"]-window[1]) <= 1e-8,
            "Unexpected appended experiment outside research window")
    manifest = read_json(paths["native_trace_manifest.json"])
    require(manifest.get("complete") is True and manifest.get("sha256") == precision.sha(paths["native_trace.csv"]),
            "Native capture completion/hash mismatch")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(manifest.get(key) == metadata[key], "Native capture model mismatch")
    precision.capture_window_covers(manifest, window)
    native = precision.native_states(paths["native_trace.csv"], window)
    if ulog_loader is None:
        from pyulog import ULog
        ulog_loader = ULog
    ulog = ulog_loader(str(paths["px4.ulg"]))
    configuration = effective_configuration(ulog, native, reference, window, records(documents["applied"]))
    with paths["control_trace.csv"].open(newline="", encoding="utf-8") as stream:
        control_rows = list(csv.DictReader(stream))
    command_report = fixed_position_commands(ulog, native, reference, control_rows, window)
    flight_report = flight_sample_check(paths["flight_samples.csv"], window, target)
    chain = precision.ulog_chain(ulog, native, reference, precision.DEFAULT_REFERENCE_SOURCE)
    require(chain.get("fixed_world_target_verified_within_2mm") is True, "PX4 target/world discrepancy exceeds tolerance")
    position = precision.metrics(native["t"], native["position"]-target, "m")
    ground = read_json(paths["native_ground.json"])
    verify_condition(ground, condition, "ground")
    require(ground.get("diagnostic_only") is False and ground.get("ground_truth_aided") is True
            and ground.get("eligible_as_stage_gate") is False and ground.get("acceptance_pending") == "full_ulog_audit",
            "Only new provisional research ground evidence can receive acceptance")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(ground.get(key) == metadata[key], "Ground model provenance differs")
    artifacts.update(verified_artifacts(folder, ground))
    ground_report = native_ground_check(folder, ground)
    if task["stage"] == "hover_folded":
        with paths["native_trace.csv"].open(newline="", encoding="utf-8") as stream:
            folded = joint_audit.native_arrays(list(csv.DictReader(stream)), window[0], duration, target)
        checks = {"position_rms_at_most_002m": position["rmse_3d"] <= .02,
                  "position_peak_at_most_005m": position["peak_3d"] <= .05,
                  "tilt_at_most_15deg": float(native["tilt"].max()) <= 15.,
                  "folded_q_at_most_001rad": float(abs(folded["q"]).max()) <= .01,
                  "joint_speed_at_most_002rad_s": float(abs(folded["dq"]).max()) <= .02}
        stage_report = {"passed": all(checks.values()), "criteria": checks,
                        "scope": "20-second fixed-position precision-EV folded-arm hover"}
    else:
        stage_report = joint_stage_check(folder, metadata, window, target, control_rows)
    artifacts.update({str(path): precision.sha(path) for path in paths.values()})
    for path in (Path(task["pid_profile"]), Path(task["research_profile"]["world_sdf"]), Path(__file__),
                 Path(precision.__file__), Path(joint_audit.__file__)):
        artifacts[str(path.resolve())] = precision.sha(path)
    report = {"kind": "x650_precision_research_stage_audit", "schema_version": 1,
        "source_run": folder.name, "source_run_directory": str(folder), "stage": task["stage"],
        "complete": True, "passed": stage_report["passed"], "simulation_backend": "gazebo_px4",
        "operating_condition": condition, "operating_condition_sha256": canonical_hash(condition),
        "pid_parameters_sha256": pid_digest, "model_sha256": metadata["model_sha256"],
        "source_urdf_sha256": metadata["source_urdf_sha256"], "window_sim_s": window,
        "target_world_enu_m": target.tolist(), "ground_truth_aided": True,
        "full_window_ulog_verified": True, "parameter_restore_verified": True,
        "configuration": configuration, "parameter_session": parameter_report,
        "fixed_position_command_audit": command_report, "flight_state_audit": flight_report, "ulog_control_chain": chain,
        "position_native_world_enu": position,
        "native_motion_velocity_world_enu": precision.metrics(native["t"], native["velocity"], "m/s"),
        "tilt_peak_deg": float(native["tilt"].max()), "ground_acceptance": ground_report,
        "stage_acceptance": stage_report, "inputs_sha256": artifacts,
        "generator_sha256": precision.sha(__file__), "no_fitted_origin_phase_or_mean": True,
        "gps_performance_claim": False, "hardware_performance_claim": False,
        "whole_body_compensation_claim": False, "external_force_rejection_claim": False,
        "original_evidence_rewritten": False}
    return report, artifacts


def audit(run_dir, launch_dir, output, *, ulog_loader=None):
    folder, output = Path(run_dir).resolve(strict=True), Path(output).resolve()
    require(not output.exists(), "Use a new audit directory; original and previous evidence are preserved")
    output.mkdir(parents=True)
    try:
        report, artifacts = _analyze(folder, launch_dir, ulog_loader=ulog_loader)
    except Exception as exc:
        failure = {"kind": "x650_precision_research_stage_audit", "source_run": folder.name,
                   "complete": False, "passed": False, "research_acceptance_verified": False,
                   "error": repr(exc), "generator_sha256": precision.sha(__file__), "original_evidence_rewritten": False}
        (output/"precision_stage_audit.json").write_text(json.dumps(failure, indent=2)+"\n", encoding="utf-8")
        raise
    report["research_acceptance_verified"] = bool(report["passed"])
    report_path = output/"precision_stage_audit.json"
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    if not report["passed"]:
        raise ValueError("Completed research flight did not pass predeclared stage criteria; no acceptance certificates issued")
    artifacts = {**artifacts, str(report_path): precision.sha(report_path)}
    common = {key: report[key] for key in ("operating_condition", "operating_condition_sha256", "pid_parameters_sha256",
              "model_sha256", "source_urdf_sha256", "full_window_ulog_verified", "parameter_restore_verified")}
    common.update(kind="x650_native_stage_evidence", schema_version=1, simulation_backend="gazebo_px4",
                  complete=True, passed=True, success=True, landed_and_disarmed=True, diagnostic_only=False,
                  ground_truth_aided=True, eligible_as_stage_gate=True, research_acceptance_verified=True,
                  source_run=folder.name, source_run_directory=str(folder), artifacts=artifacts,
                  gps_performance_claim=False, hardware_performance_claim=False,
                  acceptance_source="Independent postflight full-window ULog and native physics audit; provisional source records unchanged")
    for name, stage_name, acceptance in (("research_stage.json", report["stage"], report["stage_acceptance"]),
                                         ("research_ground.json", "native_ground", report["ground_acceptance"])):
        (output/name).write_text(json.dumps({**common, "stage": stage_name, "acceptance": acceptance}, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--launch-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir, args.launch_dir, args.output)
    print(json.dumps({"passed": report["passed"], "stage": report["stage"], "source_run": report["source_run"],
                      "output": str(args.output), "position_native_world_enu": report["position_native_world_enu"]}, indent=2))


if __name__ == "__main__":
    main()
