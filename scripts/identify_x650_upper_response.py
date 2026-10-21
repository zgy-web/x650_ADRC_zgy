"""Identify a zero-delay unit-gain upper velocity response from two native runs.

Only training data select each first-order time constant. A different completed
run is held out. Original command and truth sample timestamps are retained;
commands start at their recorded sim_s and are held until the next command.
The optional delay search is a rejection diagnostic, never a time alignment or
a fitted delay silently added to the accepted zero-delay model.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import minimize_scalar


AXES = "xyz"
ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Criteria:
    tau_min_s: float = .01
    tau_max_s: float = 1.5
    minimum_unique_samples: int = 120
    maximum_sample_gap_s: float = .06
    minimum_pulse_duration_s: float = .60
    minimum_command_amplitude_xy_m_s: float = .04
    minimum_command_amplitude_z_m_s: float = .025
    minimum_velocity_span_fraction: float = .50
    maximum_nrmse: float = .20
    maximum_rmse_xy_m_s: float = .015
    maximum_rmse_z_m_s: float = .010
    maximum_error_fraction: float = .35
    maximum_error_floor_m_s: float = .002
    maximum_bias_fraction: float = .10
    maximum_bias_floor_m_s: float = .001
    maximum_peak_error_fraction: float = .25
    maximum_peak_error_floor_m_s: float = .002
    maximum_cross_axis_rms_fraction: float = .30
    maximum_cross_axis_rms_floor_m_s: float = .003
    minimum_tau_sensitivity_fraction: float = .02
    diagnostic_delay_max_s: float = .12
    diagnostic_delay_step_s: float = .005
    maximum_material_delay_s: float = .030
    material_delay_improvement_fraction: float = .20
    material_delay_improvement_m_s: float = .001
    maximum_joint_angle_rad: float = .03
    maximum_joint_speed_rad_s: float = .05


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def json_read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require_hash(value, label):
    if not isinstance(value, str) or len(value) != 64 or set(value)-set("0123456789abcdef"):
        raise ValueError("Missing or invalid SHA-256: "+label)
    return value


def safe_sibling(folder, name):
    path = (folder/name).resolve(strict=True)
    if not path.is_relative_to(folder):
        raise ValueError("Run artifact escapes its run directory: "+str(name))
    return path


def pid_values(snapshot):
    result = {}
    for key, item in snapshot.get("parameters", {}).items():
        if key.startswith(("MC_", "MPC_")):
            value = item.get("value")
            if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value):
                raise ValueError("Invalid actual PID readback: "+key)
            result[key] = float(value)
    if len(result) < 8 or not all(result.get(key,0.) > 0 for key in ("MPC_XY_P","MPC_Z_P")):
        raise ValueError("Actual MC/MPC readback is incomplete")
    return result


def load_run(path,expected_stage="response_probe",*,audit_path=None):
    if audit_path is not None:
        return load_research_run(path, audit_path, expected_stage=expected_stage)
    folder = Path(path).resolve(strict=True)
    stage = json_read(folder/"stage_result.json")
    if (stage.get("diagnostic_only") is True or stage.get("operating_condition") is not None
            or stage.get("eligible_as_stage_gate") is False):
        raise ValueError("Diagnostic/provisional research runs require explicit accepted research audit inputs")
    if (stage.get("kind") != "x650_native_stage_evidence" or stage.get("stage") != expected_stage
            or stage.get("simulation_backend") != "gazebo_px4"
            or any(stage.get(k) is not True for k in ("complete","passed","success","landed_and_disarmed"))):
        raise ValueError("A completed and landed native "+expected_stage+" run is required")
    artifacts = stage.get("artifacts")
    required = {"control_trace.csv","px4_parameters_before.json","px4_parameters_after.json"}
    if not isinstance(artifacts,dict) or not required.issubset(artifacts):
        raise ValueError("Stage evidence lacks raw control/parameter artifact hashes")
    sources = {str(folder/"stage_result.json"):sha(folder/"stage_result.json")}
    for name, expected in artifacts.items():
        artifact = safe_sibling(folder,name)
        if sha(artifact) != require_hash(expected,name):
            raise ValueError("Native artifact hash mismatch: "+name)
        sources[str(artifact)] = expected
    metadata_path = safe_sibling(folder,stage.get("metadata","controller_metadata.json"))
    meta = json_read(metadata_path)
    if (meta.get("diagnostic_only") is True or stage.get("diagnostic_only") is True
            or meta.get("task", {}).get("research_profile") is not None
            or meta.get("operating_condition") is not None or stage.get("eligible_as_stage_gate") is False):
        raise ValueError("Diagnostic/provisional research runs require explicit accepted research audit inputs")
    sources[str(metadata_path)] = sha(metadata_path)
    if (meta.get("complete") is not True or meta.get("landed_and_disarmed") is not True
            or meta.get("stage") != expected_stage or meta.get("controller") != "pid_ff"):
        raise ValueError("Response probe metadata is incomplete or belongs to another controller")
    provenance = meta.get("provenance",{})
    identity = {key:require_hash(provenance.get(key),key) for key in (
        "model_sha256","source_urdf_sha256","airframe_sha256","flight_profile_sha256","preparation_report_sha256")}
    for key in ("model_sha256","source_urdf_sha256"):
        if stage.get(key) != identity[key] or meta.get(key) != identity[key]:
            raise ValueError("Run model identity disagrees across native evidence")
    snapshots = [json_read(folder/f"px4_parameters_{phase}.json") for phase in ("before","after")]
    for snapshot in snapshots:
        if (snapshot.get("contract_matches") is not True
                or snapshot.get("model_sha256") != identity["model_sha256"]
                or snapshot.get("airframe_sha256") != identity["airframe_sha256"] or snapshot.get("error")):
            raise ValueError("Actual PX4 parameter contract did not pass")
    pid = pid_values(snapshots[0])
    if pid_values(snapshots[1]) != pid:
        raise ValueError("MC/MPC parameters changed during the response run")
    actual = meta.get("actual_pid_parameters")
    if not isinstance(actual,dict) or {k:float(v) for k,v in actual.items()} != pid:
        raise ValueError("Controller metadata differs from the actual PID readback")
    identity["pid_parameters_sha256"] = json_hash(pid)
    csv_path = folder/"control_trace.csv"
    with csv_path.open(newline="",encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = {"sim_s","t_s","phase","truth_stamp_s","q1","q2","qd1","qd2"}
        fields.update("base_v"+a for a in AXES)
        fields.update("base_velocity_target_"+a for a in AXES)
        if not fields.issubset(reader.fieldnames or []):
            raise ValueError("Control trace lacks native command/state timestamps or velocity fields")
        raw = [row for row in reader if row["phase"] == "measure" and float(row["t_s"]) >= 0.]
    if len(raw) < 10:
        raise ValueError("No complete response measurement window")
    def values(name):
        out = np.array([float(row[name]) for row in raw])
        if not np.isfinite(out).all():
            raise ValueError("Nonfinite control trace column: "+name)
        return out
    command_times, task_times, truth_times = values("sim_s"), values("t_s"), values("truth_stamp_s")
    origins = command_times-task_times
    if np.ptp(origins) > 1e-6 or np.any(np.diff(command_times) <= 0.) or np.any(np.diff(truth_times) < -1e-9):
        raise ValueError("Recorded command/state time base is inconsistent or regressed")
    ages = command_times-truth_times
    if np.min(ages) < -.001 or np.max(ages) > .25+1e-9:
        raise ValueError("Native truth age violates the declared run contract")
    origin = float(np.median(origins))
    commands = np.column_stack([values("base_velocity_target_"+a) for a in AXES])
    velocities = np.column_stack([values("base_v"+a) for a in AXES])
    joints = np.column_stack([values(k) for k in ("q1","q2","qd1","qd2")])
    # Duplicated truth stamps contain no new observation. They are never
    # averaged or assigned the later command's time. Keep every command event.
    keep = np.r_[True,np.diff(truth_times) > 1e-9]
    for index in np.flatnonzero(~keep):
        if not np.allclose(velocities[index],velocities[index-1],atol=1e-12,rtol=0):
            raise ValueError("One truth timestamp contains conflicting velocity measurements")
    observed_times, observed_v = truth_times[keep]-origin, velocities[keep]
    fingerprint = json_hash({"command_times":np.round(command_times-origin,9).tolist(),
                             "commands":np.round(commands,9).tolist(),
                             "truth_times":np.round(observed_times,9).tolist(),
                             "velocities":np.round(observed_v,9).tolist()})
    return {"folder":str(folder),"identity":identity,"pid_parameters":pid,"artifacts":sources,
            "command_csv_sha256":sha(csv_path),"normalized_measurement_sha256":fingerprint,
            "command_times":command_times-origin,"commands":commands,
            "truth_times":observed_times,"velocities":observed_v,"joints":joints,
            "metadata":meta,"stage_evidence":stage,"control_rows":raw,
            "max_truth_age_s":float(np.max(ages)),"duplicate_truth_rows":int((~keep).sum())}


def load_research_run(path, audit_path, *, expected_stage="response_probe"):
    """Load a separately accepted v2 run; restored values are never flight PID."""
    for directory in (ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1"):
        if str(directory) not in sys.path:
            sys.path.insert(0, str(directory))
    import audit_x650_precision_stage as existing_audit
    from research_condition import validate_operating_condition
    folder, supplied = Path(path).resolve(strict=True), Path(audit_path).resolve(strict=True)
    audit_folder = supplied if supplied.is_dir() else supplied.parent
    certificate_path, report_path = audit_folder/"research_stage.json", audit_folder/"upper_stage_audit.json"
    if supplied.is_file() and supplied not in (certificate_path, report_path):
        raise ValueError("Research audit input must name its directory, research_stage.json or upper_stage_audit.json")
    certificate, report = json_read(certificate_path), json_read(report_path)
    verified = ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
                "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")
    if (certificate.get("kind") != "x650_native_stage_evidence" or certificate.get("stage") != expected_stage
            or certificate.get("eligible_as_stage_gate") is not True or certificate.get("diagnostic_only") is not False
            or certificate.get("ground_truth_aided") is not True
            or report.get("kind") != "x650_precision_upper_stage_audit" or report.get("stage") != expected_stage
            or any(document.get(key) is not True for document in (certificate, report) for key in verified)):
        raise ValueError("Complete independently accepted upper-EV research evidence is required")
    for document in (certificate, report):
        if (document.get("simulation_backend") != "gazebo_px4" or document.get("source_run") != folder.name
                or Path(document.get("source_run_directory", "")).resolve() != folder):
            raise ValueError("Research acceptance belongs to a different run directory")
    condition = validate_operating_condition(report.get("operating_condition"))
    if (condition.get("schema_version") != 2 or condition.get("research_profile") != "precision_ev_upper_v2"
            or condition.get("control_interface") != "raw_ev_world_pva_v2"
            or condition.get("feedback_contract") != "px4_estimator_receipt_enu_v1"):
        raise ValueError("Response identification requires the reviewed upper-v2 feedback/command contract")
    for document in (certificate, report):
        if (document.get("operating_condition") != condition
                or document.get("operating_condition_sha256") != json_hash(condition)
                or document.get("pid_parameters_sha256") != condition["pid_parameters_sha256"]):
            raise ValueError("Research condition/PID identity disagrees across acceptance files")
    proxy = report.get("feedback_velocity_proxy", {})
    if (proxy.get("verified") is not True or proxy.get("passed") is not True
            or proxy.get("axis_rms_limit_m_s") != [.015, .015, .010]
            or np.asarray(proxy.get("metrics", {}).get("axis_rmse", [])).shape != (3,)
            or not np.isfinite(proxy["metrics"]["axis_rmse"]).all()
            or np.any(np.asarray(proxy["metrics"]["axis_rmse"]) > [.015, .015, .010])):
        raise ValueError("Research PX4 velocity proxy lacks its passed predeclared RMS check")
    artifacts = certificate.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        raise ValueError("Research acceptance needs absolute hashed raw artifacts")
    sources = {str(certificate_path): sha(certificate_path)}
    for name, digest in artifacts.items():
        artifact = Path(name)
        if not artifact.is_absolute() or not artifact.is_file() or sha(artifact) != require_hash(digest, name):
            raise ValueError("Accepted research artifact checksum/path mismatch: "+name)
        resolved = str(artifact.resolve())
        if resolved in sources and sources[resolved] != digest:
            raise ValueError("Conflicting research artifact identity")
        sources[resolved] = digest
    if sources.get(str(report_path)) != sha(report_path):
        raise ValueError("Research certificate does not bind its complete upper audit")
    if any(sources.get(str(Path(name).resolve())) != digest for name, digest in report.get("inputs_sha256", {}).items()):
        raise ValueError("Research certificate does not bind every independently audited input")
    required = ("stage_result.json", "controller_metadata.json", "controller_task.json", "control_trace.csv",
                "feedback_trace.jsonl", "estimated_feedback.json", "native_trace.csv", "native_trace_manifest.json", "px4.ulg",
                "diagnostic_parameter_session.json", "diagnostic_parameters_before.json", "diagnostic_parameters_applied.json",
                "diagnostic_parameters_restored.json", "diagnostic_restore_journal_completed.json",
                "px4_parameters_before.json", "px4_parameters_after.json")
    for name in required:
        if sources.get(str(folder/name)) != sha(folder/name):
            raise ValueError("Research acceptance misses this run's original artifact: "+name)
    meta, task, stage = (json_read(folder/name) for name in ("controller_metadata.json", "controller_task.json", "stage_result.json"))
    if (task != meta.get("task") or task.get("stage") != expected_stage or meta.get("stage") != expected_stage
            or meta.get("controller") != "pid_ff" or meta.get("complete") is not True
            or meta.get("landed_and_disarmed") is not True or stage.get("complete") is not True
            or meta.get("diagnostic_only") is not False or meta.get("ground_truth_aided") is not True
            or meta.get("eligible_as_stage_gate") is not False or meta.get("acceptance_pending") != "full_ulog_audit"):
        raise ValueError("Original research run is incomplete, diagnostic or relabeled")
    if (meta.get("operating_condition") != condition or meta.get("operating_condition_sha256") != json_hash(condition)
            or meta.get("feedback_contract") != condition["feedback_contract"]):
        raise ValueError("Original research metadata has another operating/feedback condition")
    documents = {label: json_read(folder/name) for label, name in {
        "session": "diagnostic_parameter_session.json", "journal": "diagnostic_restore_journal_completed.json",
        "before": "diagnostic_parameters_before.json", "applied": "diagnostic_parameters_applied.json",
        "restored": "diagnostic_parameters_restored.json", "px4_before": "px4_parameters_before.json",
        "px4_after": "px4_parameters_after.json"}.items()}
    applied = existing_audit.records(documents["applied"])
    pid = {name: float(applied[name]["value"]) for name in existing_audit.CONTROL_NAMES}
    if len(pid) != 42 or json_hash(pid) != condition["pid_parameters_sha256"]:
        raise ValueError("Actual applied 42 PID values do not match research identity")
    existing_audit.verify_parameter_session(documents, meta, condition, pid)
    provenance = meta.get("provenance", {})
    identity = {key: require_hash(provenance.get(key), key) for key in
        ("model_sha256", "source_urdf_sha256", "airframe_sha256", "flight_profile_sha256", "preparation_report_sha256")}
    if any(document.get(key) != identity[key] for document in (meta, stage, certificate, report)
           for key in ("model_sha256", "source_urdf_sha256")):
        raise ValueError("Research physical identity disagrees across raw and accepted evidence")
    identity.update(pid_parameters_sha256=json_hash(pid), operating_condition=condition,
                    operating_condition_sha256=json_hash(condition), feedback_contract=condition["feedback_contract"],
                    control_interface=condition["control_interface"])
    window = np.asarray(report.get("window_sim_s"), float)
    if (window.shape != (2,) or not np.isfinite(window).all() or abs(window[1]-window[0]-30.) > .05
            or not np.allclose(window, [meta["task_start_sim_s"], meta["task_end_sim_s"]], atol=1e-8, rtol=0)
            or meta.get("requested_duration_s") != 30. or expected_stage != "response_probe"):
        raise ValueError("Research response identification needs its complete 30-second probe")
    csv_path = folder/"control_trace.csv"
    with csv_path.open(newline="", encoding="utf-8") as stream:
        raw = [row for row in csv.DictReader(stream) if row["phase"] == "measure"
               and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    def column(name):
        values = np.asarray([float(row[name]) for row in raw])
        if not np.isfinite(values).all():
            raise ValueError("Nonfinite research command field: "+name)
        return values
    command_times = column("sim_s")
    task_times = column("t_s")
    if (len(raw) < 2 or np.any(np.diff(command_times) <= 0) or np.max(np.diff(command_times)) > .05+1e-9
            or command_times[0] > window[0]+.05 or command_times[-1] < window[1]-.05
            or np.max(abs(command_times-task_times-window[0])) > 1e-8):
        raise ValueError("Research command timestamps do not cover the declared original window")
    commands = np.column_stack([column("base_velocity_target_"+axis) for axis in AXES])
    acceleration = np.column_stack([column("command_acceleration_"+axis) for axis in AXES])
    if np.max(abs(acceleration)) > 1e-12:
        raise ValueError("Zero-FF response identification cannot accept an active acceleration input")
    native_path = folder/"native_trace.csv"
    manifest = json_read(folder/"native_trace_manifest.json")
    if manifest.get("complete") is not True or manifest.get("sha256") != sha(native_path):
        raise ValueError("Research response native trace is incomplete or changed")
    with native_path.open(newline="", encoding="utf-8") as stream:
        native_rows = [row for row in csv.DictReader(stream) if window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    if len(native_rows) < 2 or any(row.get(flag) != "1" for row in native_rows for flag in ("valid_base", "valid_joint_state")):
        raise ValueError("Research response requires valid independent physics samples")
    def native_column(name):
        values = np.asarray([float(row[name]) for row in native_rows])
        if not np.isfinite(values).all():
            raise ValueError("Nonfinite independent research velocity/joint field: "+name)
        return values
    native_times = native_column("sim_s")
    if (native_times[0] > window[0]+.008 or native_times[-1] < window[1]-.008
            or np.any(np.diff(native_times) <= 0) or np.max(np.diff(native_times)) > .008+1e-9):
        raise ValueError("Independent native response window is incomplete or sparse")
    velocities = np.column_stack([native_column("base_v"+axis) for axis in AXES])
    joints = np.column_stack([native_column(name) for name in
        ("arm_joint_1_q", "arm_joint_2_q", "arm_joint_1_qd", "arm_joint_2_qd")])
    fingerprint = json_hash({"command_times": np.round(command_times-window[0], 9).tolist(),
        "commands": np.round(commands, 9).tolist(), "truth_times": np.round(native_times-window[0], 9).tolist(),
        "velocities": np.round(velocities, 9).tolist()})
    return {"folder": str(folder), "identity": identity, "pid_parameters": pid, "artifacts": sources,
        "command_csv_sha256": sha(csv_path), "normalized_measurement_sha256": fingerprint,
        "command_times": command_times-window[0], "commands": commands, "truth_times": native_times-window[0],
        "velocities": velocities, "joints": joints, "metadata": meta, "stage_evidence": certificate,
        "control_rows": raw, "max_truth_age_s": 0., "duplicate_truth_rows": 0,
        "observation_contract": "Independent native physical world-ENU velocity at original simulation times",
        "command_contract": "Logical velocity commands verified against sent PVA and PX4 received/internal targets by the complete upper audit",
        "feedback_velocity_proxy": report["feedback_velocity_proxy"], "research_audit": str(report_path)}


def predict_velocity(tau, command_times, commands, observation_times, initial_velocity, *, diagnostic_delay_s=0.):
    """Exact event-driven ZOH prediction at the original truth sample times."""
    event_times = np.asarray(command_times)+diagnostic_delay_s
    observations = np.asarray(observation_times)
    output = np.empty(len(observations))
    output[0] = initial_velocity
    previous_time, current = float(observations[0]),float(initial_velocity)
    index = int(np.searchsorted(event_times,previous_time,side="right")-1)
    held = 0. if index < 0 else float(commands[index])
    next_index = index+1
    for sample in range(1,len(observations)):
        finish = float(observations[sample])
        while next_index < len(event_times) and event_times[next_index] <= finish:
            event = float(event_times[next_index])
            if event > previous_time:
                a = np.exp(-(event-previous_time)/tau)
                current = a*current+(1-a)*held
            previous_time,held = max(previous_time,event),float(commands[next_index])
            next_index += 1
        a = np.exp(-(finish-previous_time)/tau)
        current = a*current+(1-a)*held
        output[sample],previous_time = current,finish
    return output


def axis_data(run, axis, criteria):
    start,end = 2.+4*axis,6.+4*axis
    mask = (run["truth_times"] >= start-1e-9)&(run["truth_times"] <= end+1e-9)
    times,v = run["truth_times"][mask],run["velocities"][mask,axis]
    cmd_mask = (run["command_times"] >= start-1e-9)&(run["command_times"] < end-1e-9)
    u = run["commands"][cmd_mask,axis]
    cmd_t = run["command_times"][cmd_mask]
    reasons = []
    if len(times) < criteria.minimum_unique_samples or not len(u):
        return {"passed":False,"reasons":["Insufficient unique truth samples / command coverage"],"axis":AXES[axis]}
    amplitude = float((np.max(u)-np.min(u))/2)
    minimum = criteria.minimum_command_amplitude_z_m_s if axis == 2 else criteria.minimum_command_amplitude_xy_m_s
    durations = np.diff(np.r_[cmd_t,end])
    positive = float(durations[u >= .8*max(float(np.max(u)),1e-12)].sum())
    negative = float(durations[u <= .8*min(float(np.min(u)),-1e-12)].sum())
    if amplitude < minimum or np.min(u) >= -minimum or np.max(u) <= minimum:
        reasons.append("Positive/negative velocity excitation amplitude is insufficient")
    if min(positive,negative) < criteria.minimum_pulse_duration_s:
        reasons.append("Positive/negative pulse duration is insufficient")
    maximum_gap = float(np.max(np.diff(times)))
    if maximum_gap > criteria.maximum_sample_gap_s+1e-9 or times[0] > start+.06 or times[-1] < end-.06:
        reasons.append("Observation gap or axis-window coverage is insufficient")
    if np.ptp(v) < criteria.minimum_velocity_span_fraction*np.ptp(u):
        reasons.append("Actual measured velocity excitation is insufficient")
    return {"passed":not reasons,"reasons":reasons,"axis":AXES[axis],"times":times,"velocity":v,
            "all_velocity":run["velocities"][mask],"amplitude_m_s":amplitude,
            "unique_samples":len(times),"maximum_sample_gap_s":maximum_gap,
            "positive_pulse_s":positive,"negative_pulse_s":negative,"window_s":[start,end]}


def evaluate(run, axis, data, tau, criteria):
    prediction = predict_velocity(tau,run["command_times"],run["commands"][:,axis],data["times"],data["velocity"][0])
    error = prediction-data["velocity"]
    amplitude = data["amplitude_m_s"]
    rmse = float(np.sqrt(np.mean(error**2)))
    nrmse = rmse/max(amplitude,1e-12)
    peak_error = float(max(abs(np.max(prediction)-np.max(data["velocity"])),
                           abs(np.min(prediction)-np.min(data["velocity"]))))
    cross = np.delete(data["all_velocity"],axis,axis=1)
    cross_rms = float(np.max(np.sqrt(np.mean(cross**2,axis=0))))
    delays = np.arange(0.,criteria.diagnostic_delay_max_s+criteria.diagnostic_delay_step_s/2,criteria.diagnostic_delay_step_s)
    delayed_rmse = []
    for delay in delays:
        candidate = predict_velocity(tau,run["command_times"],run["commands"][:,axis],data["times"],
                                     data["velocity"][0],diagnostic_delay_s=float(delay))
        delayed_rmse.append(float(np.sqrt(np.mean((candidate-data["velocity"])**2))))
    best_index = int(np.argmin(delayed_rmse))
    improvement = rmse-delayed_rmse[best_index]
    material_delay = (delays[best_index] > criteria.maximum_material_delay_s+1e-9
                      and improvement >= criteria.material_delay_improvement_m_s
                      and improvement >= criteria.material_delay_improvement_fraction*max(rmse,1e-12))
    absolute_limit = criteria.maximum_rmse_z_m_s if axis == 2 else criteria.maximum_rmse_xy_m_s
    reasons = []
    if nrmse > criteria.maximum_nrmse or rmse > absolute_limit:
        reasons.append("Free-run RMSE exceeds the fixed validation threshold")
    if np.max(abs(error)) > criteria.maximum_error_fraction*amplitude+criteria.maximum_error_floor_m_s:
        reasons.append("Maximum free-run residual is too large")
    if abs(float(np.mean(error))) > criteria.maximum_bias_fraction*amplitude+criteria.maximum_bias_floor_m_s:
        reasons.append("Persistent velocity bias is not explained by the unit-gain model")
    if peak_error > criteria.maximum_peak_error_fraction*amplitude+criteria.maximum_peak_error_floor_m_s:
        reasons.append("Measured response peaks are not reproduced")
    if cross_rms > criteria.maximum_cross_axis_rms_fraction*amplitude+criteria.maximum_cross_axis_rms_floor_m_s:
        reasons.append("Cross-axis motion is too large for the diagonal response model")
    if material_delay:
        reasons.append("A material unmodeled command delay improves residuals; zero-delay model rejected")
    return {"passed":not reasons,"reasons":reasons,"rmse_m_s":rmse,"nrmse_command_half_amplitude":nrmse,
            "maximum_error_m_s":float(np.max(abs(error))),"bias_m_s":float(np.mean(error)),
            "peak_error_m_s":peak_error,"cross_axis_rms_m_s":cross_rms,
            "delay_diagnostic":{"best_delay_s":float(delays[best_index]),"zero_delay_rmse_m_s":rmse,
                                "best_rmse_m_s":delayed_rmse[best_index],"improvement_m_s":improvement,
                                "material_unmodeled_delay":bool(material_delay),"applied_to_output_model":False}}


def analyze_runs(training_path,holdout_path,criteria=None,*,train_audit=None,holdout_audit=None):
    criteria = Criteria() if criteria is None else criteria
    if (train_audit is None) != (holdout_audit is None):
        raise ValueError("Research training and holdout audit inputs must be supplied together")
    training,holdout = load_run(training_path,audit_path=train_audit),load_run(holdout_path,audit_path=holdout_audit)
    if (training["folder"] == holdout["folder"]
            or training["command_csv_sha256"] == holdout["command_csv_sha256"]
            or training["normalized_measurement_sha256"] == holdout["normalized_measurement_sha256"]):
        raise ValueError("Training and holdout must be independent runs, not the same or copied measurement data")
    if training["identity"] != holdout["identity"]:
        raise ValueError("Training/holdout model, geometry, propulsion or actual PID hashes differ")
    reasons = []
    for label,run in (("training",training),("holdout",holdout)):
        if (np.max(abs(run["joints"][:,:2])) > criteria.maximum_joint_angle_rad
                or np.max(abs(run["joints"][:,2:])) > criteria.maximum_joint_speed_rad_s):
            reasons.append(label+": Arm was not held near the declared folded identification configuration")
    axes,times = [],[]
    for axis in range(3):
        train_data,hold_data = axis_data(training,axis,criteria),axis_data(holdout,axis,criteria)
        axis_report = {"axis":AXES[axis],"training_excitation":{k:v for k,v in train_data.items() if k not in {"times","velocity","all_velocity"}},
                       "holdout_excitation":{k:v for k,v in hold_data.items() if k not in {"times","velocity","all_velocity"}}}
        if not train_data["passed"] or not hold_data["passed"]:
            axis_report.update(passed=False,base_time_constant_s=None)
            times.append(None)
            axes.append(axis_report)
            continue
        def objective(logtau):
            prediction = predict_velocity(np.exp(logtau),training["command_times"],training["commands"][:,axis],
                                          train_data["times"],train_data["velocity"][0])
            return float(np.mean((prediction-train_data["velocity"])**2))
        fit = minimize_scalar(objective,bounds=(math.log(criteria.tau_min_s),math.log(criteria.tau_max_s)),
                              method="bounded",options={"xatol":1e-7})
        tau = float(np.exp(fit.x))
        train_metrics,hold_metrics = evaluate(training,axis,train_data,tau,criteria),evaluate(holdout,axis,hold_data,tau,criteria)
        fitted_prediction = predict_velocity(tau,training["command_times"],training["commands"][:,axis],
                                              train_data["times"],train_data["velocity"][0])
        sensitivity = min(float(np.sqrt(np.mean((predict_velocity(tau*factor,training["command_times"],
            training["commands"][:,axis],train_data["times"],train_data["velocity"][0])-fitted_prediction)**2)))
            for factor in (.5,2.))/max(train_data["amplitude_m_s"],1e-12)
        identifiable = (bool(fit.success) and criteria.tau_min_s*1.05 < tau < criteria.tau_max_s/1.05
                        and sensitivity >= criteria.minimum_tau_sensitivity_fraction)
        axis_report.update(base_time_constant_s=tau,fit_source="training only",fit_success=bool(fit.success),
                           tau_identifiable=identifiable,neighbor_prediction_change_fraction=sensitivity,
                           training=train_metrics,holdout=hold_metrics,
                           passed=identifiable and train_metrics["passed"] and hold_metrics["passed"])
        times.append(tau)
        axes.append(axis_report)
    passed = not reasons and all(axis["passed"] for axis in axes)
    for axis in axes:
        if not axis["passed"]:
            reasons.append(axis["axis"]+": Excitation, identification or independent validation did not pass")
    def run_summary(run):
        return {key:run[key] for key in ("folder","command_csv_sha256","normalized_measurement_sha256","max_truth_age_s","duplicate_truth_rows")}
    report = {"kind":"x650_native_stage_evidence","schema_version":1,"stage":"response_identification",
              "simulation_backend":"gazebo_px4","complete":True,"passed":passed,"success":passed,
              **training["identity"],"base_time_constant_s":times,"identified":passed,
              "criteria":asdict(criteria),"reasons":reasons,"axes":axes,
              "criteria_status":"Predeclared engineering acceptance; not a proof the plant is exactly first order or these constants are globally optimal",
              "training":run_summary(training),"holdout":run_summary(holdout),
              "method":"Exact variable-dt event-driven command ZOH; one free-run unit-gain zero-delay first-order model per axis",
              "parameter_selection":"Tau fitted on training only; independent holdout never updates tau, gain, bias or phase",
              "time_alignment":"Commands at original recorded sim_s; velocity at original truth_stamp_s; no measured curve shifted",
              "command_timestamp_scope":"Logical command-generation sim_s; not a direct actuator receipt timestamp",
              "gain":[1.,1.,1.],"bias_m_s":[0.,0.,0.],"command_delay_s":0.,
              "joint_response_identified":False,"full_body_dynamics_identified":False,
              "artifacts":{**training["artifacts"],**holdout["artifacts"]},
              "identifier_source_sha256":sha(__file__)}
    model = {"kind":"x650_upper_velocity_response_model","schema_version":1,"identified":passed,"passed":passed,
             **training["identity"],"base_time_constant_s":times,"gain":[1.,1.,1.],"bias_m_s":[0.,0.,0.],
             "command_delay_s":0.,"response_identified":passed,"joint_response_identified":False,
             "fit_source":"Independent native response_probe training/holdout runs",
             "training_control_trace_sha256":training["command_csv_sha256"],
             "holdout_control_trace_sha256":holdout["command_csv_sha256"],
             "evidence":"response_identification.json","identifier_source_sha256":sha(__file__),
             "scope":"Folded-arm closed-loop base velocity response; no angular/arm dynamics calibration or real-time claim"}
    if train_audit is not None:
        common = {"schema_version": 2, "diagnostic_only": False, "ground_truth_aided": True,
            "eligible_as_stage_gate": bool(passed), "research_acceptance_verified": bool(passed),
            "full_window_ulog_verified": True, "parameter_restore_verified": True,
            "upper_command_verified": True, "feedback_contract_verified": True,
            "gps_performance_claim": False, "hardware_performance_claim": False,
            "observation_contract": training["observation_contract"], "command_contract": training["command_contract"],
            "feedback_velocity_proxy": {"training": training["feedback_velocity_proxy"], "holdout": holdout["feedback_velocity_proxy"]},
            "acceleration_feedforward_contract": "Zero acceleration feedforward during identification; active FF is outside this model validation",
            "time_alignment": "Original logical send simulation times and independent native physical simulation times; no fitted origin or delay",
            "scope": "Precision-EV upper-v2 folded-arm physical base response with independently audited PX4 estimator feedback and zero acceleration FF"}
        report.update(common)
        model.update(common)
    return model,report


def write_identification(training,holdout,output,*,train_audit=None,holdout_audit=None):
    output = Path(output)
    if output.exists():
        raise ValueError("Use a new output directory; identification evidence is never overwritten")
    try:
        model,report = analyze_runs(training,holdout,train_audit=train_audit,holdout_audit=holdout_audit)
    except (ValueError,KeyError,OSError,TypeError) as exc:
        model = {"kind":"x650_upper_velocity_response_model","identified":False,"passed":False,
                 "base_time_constant_s":None,"reason":str(exc)}
        report = {"kind":"x650_native_stage_evidence","stage":"response_identification","complete":False,
                  "passed":False,"success":False,"simulation_backend":"gazebo_px4",
                  "reasons":[str(exc)],"criteria":asdict(Criteria()),"artifacts":{}}
    output.mkdir(parents=True,exist_ok=False)
    model_path = output/"response_model.json"
    model_path.write_text(json.dumps(model,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    report["artifacts"]["response_model.json"] = sha(model_path)
    (output/"response_identification.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train",type=Path,required=True)
    parser.add_argument("--holdout",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--train-audit",type=Path)
    parser.add_argument("--holdout-audit",type=Path)
    args = parser.parse_args(argv)
    if (args.train_audit is None) != (args.holdout_audit is None):
        parser.error("--train-audit and --holdout-audit must be supplied together")
    report = write_identification(args.train,args.holdout,args.output,
                                  train_audit=args.train_audit,holdout_audit=args.holdout_audit)
    print(json.dumps({"output":str(args.output),"passed":report["passed"],
                      "base_time_constant_s":report.get("base_time_constant_s"),"reasons":report["reasons"]},indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
