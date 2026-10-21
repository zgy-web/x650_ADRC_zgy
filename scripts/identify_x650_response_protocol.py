#!/usr/bin/env python3
"""Preregister, train, freeze and independently validate the mixed response model.

No accepted model is emitted by training. Validation has no optimizer and uses
an untouched role-specific run whose preflight task binds the frozen candidate.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
import zipfile
import hashlib
import numpy as np
from scipy.optimize import least_squares, minimize_scalar
ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1"):
    if str(directory) not in sys.path: sys.path.insert(0, str(directory))
import identify_x650_upper_response as old
from identify_x650_upper_response import sha, json_hash, json_read, require_hash, predict_velocity
from compare_x650_response_models import predict_second_order
from simulations.x650_wholebody_mpc_v1.response_protocol import (
    build_response_protocol, validate_response_protocol, response_protocol_hash, protocol_velocity)
AXES = "xyz"


def load_protocol_run(path, audit_path, *, expected_stage="response_probe"):
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
            or report.get("kind") != "x650_precision_response_protocol_audit" or report.get("stage") != expected_stage
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
    protocol = validate_response_protocol(task.get("response_probe_protocol"))
    if protocol is None:
        raise ValueError("Explicit frozen response protocol required")
    for document in (meta, stage, certificate, report):
        if (document.get("response_probe_protocol") != protocol or
                document.get("response_probe_protocol_sha256") != response_protocol_hash(protocol)):
            raise ValueError("Formal response protocol identity differs across evidence")
    window = np.asarray(report.get("window_sim_s"), float)
    if (window.shape != (2,) or not np.isfinite(window).all() or abs(window[1]-window[0]-protocol["duration_s"]) > .05
            or not np.allclose(window, [meta["task_start_sim_s"], meta["task_end_sim_s"]], atol=1e-8, rtol=0)
            or meta.get("requested_duration_s") != protocol["duration_s"] or expected_stage != "response_probe"):
        raise ValueError("Research response identification needs its complete frozen protocol probe")
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
    return {"folder": str(folder), "identity": identity, "protocol": protocol, "pid_parameters": pid, "artifacts": sources,
        "command_csv_sha256": sha(csv_path), "normalized_measurement_sha256": fingerprint,
        "command_times": command_times-window[0], "commands": commands, "truth_times": native_times-window[0],
        "velocities": velocities, "joints": joints, "metadata": meta, "stage_evidence": certificate,
        "control_rows": raw, "max_truth_age_s": 0., "duplicate_truth_rows": 0,
        "observation_contract": "Independent native physical world-ENU velocity at original simulation times",
        "command_contract": "Logical velocity commands verified against sent PVA and PX4 received/internal targets by the complete upper audit",
        "feedback_velocity_proxy": report["feedback_velocity_proxy"], "research_audit": str(report_path)}

def interval_mask(times, window):
    return (times >= window[0]-1e-9) & (times <= window[1]+1e-9)


def analysis_sources():
    paths = [Path(__file__), ROOT/'scripts/audit_x650_response_protocol.py',
             ROOT/'scripts/audit_x650_upper_stage.py', ROOT/'scripts/audit_x650_precision_stage.py',
             ROOT/'scripts/analyze_x650_hover_precision.py', ROOT/'scripts/analyze_x650_joint_probe.py',
             ROOT/'scripts/identify_x650_upper_response.py', ROOT/'scripts/compare_x650_response_models.py',
             ROOT/'scripts/run_x650_formal_response.py', ROOT/'scripts/run_x650_precision_candidates.py',
             ROOT/'simulations/x650_wholebody_mpc_v1/response_protocol.py']
    return {str(path.resolve()): sha(path) for path in paths}


def save_new(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(document, indent=2, allow_nan=False)+'\n')


def valid_created_time(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and 0. < value <= time.time())


def freeze_plan(output, source_paths, tasks, px4_binary_path, preflight):
    """Create the immutable declaration before either new flight is launched.

    tasks maps roles to JSON file paths OR task dictionaries. Templates deliberately
    omit the candidate-only holdout fields, which cannot exist before training.
    """
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Existing preregistration cannot be overwritten')
    if set(tasks) != {'train', 'holdout'}:
        raise ValueError('Both role-specific task templates are required')
    sources = analysis_sources()
    sources.update({str(Path(path).resolve(strict=True)): sha(path) for path in source_paths})
    from simulations.x650_wholebody_mpc_v1.task import source_files_for_task, condition_for_task
    preflight_document = json_read(preflight) if isinstance(preflight, (str, Path)) else preflight
    provenance = preflight_document.get('provenance', preflight_document)
    if 'preflight' in provenance:
        provenance = provenance['preflight'].get('provenance', provenance['preflight'])
    declared_sources = provenance.get('source_files_sha256', {})
    if not isinstance(declared_sources, dict) or not declared_sources:
        raise ValueError('Preregistration requires nonempty preflight controller source evidence')
    for relative, expected in declared_sources.items():
        source = (ROOT/relative).resolve(strict=True)
        if not source.is_relative_to(ROOT) or sha(source) != expected:
            raise ValueError('Preflight source digest/path mismatch: '+relative)
        sources[str(source)] = expected
    templates = {}
    for role, value in tasks.items():
        task = json_read(value) if isinstance(value, (str, Path)) else value
        expected = build_response_protocol(role)
        if task.get('response_probe_protocol') != expected:
            raise ValueError('Task does not bind the complete reviewed '+role+' protocol')
        if condition_for_task(task) != provenance.get('operating_condition'):
            raise ValueError('Task operating condition differs from preflight evidence')
        if any(task.get(key) is not None for key in ('frozen_response_candidate', 'frozen_response_candidate_sha256')):
            raise ValueError('Preregistration templates cannot already bind a fitted candidate')
        for source in source_files_for_task(task):
            relative = source.relative_to(ROOT).as_posix()
            if declared_sources.get(relative) != sha(source):
                raise ValueError('Preflight omitted or changed a task controller source: '+relative)
            sources[str(source.resolve())] = sha(source)
        templates[role] = {key: value for key, value in task.items()
                          if key not in ('frozen_response_candidate', 'frozen_response_candidate_sha256')}
    binary = Path(px4_binary_path).resolve(strict=True)
    archive_path = output.with_suffix('.sources.zip')
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    members = {}
    with zipfile.ZipFile(archive_path, 'x', zipfile.ZIP_DEFLATED) as archive:
        for index, (source, expected) in enumerate(sorted(sources.items())):
            payload = Path(source).read_bytes()
            if hashlib.sha256(payload).hexdigest() != expected:
                raise ValueError('Source changed while freezing immutable archive: '+source)
            member = '%04d/' % index+Path(source).name
            archive.writestr(member, payload)
            members[member] = {'source_path': source, 'sha256': expected}
    document = {'kind': 'x650_response_preregistration', 'schema_version': 1,
        'created_unix': time.time(), 'protocols': {role: build_response_protocol(role) for role in ('train', 'holdout')},
        'task_templates': templates, 'source_sha256': sources, 'analysis_sources_sha256': analysis_sources(),
        'source_archive': {'path': str(archive_path), 'sha256': sha(archive_path), 'members': members},
        'px4_binary_path': str(binary), 'px4_binary_sha256': sha(binary),
        'preflight': preflight_document,
        'eligible_as_stage_gate': False, 'scope': 'Frozen input/analysis plan only; no experiment outcome claimed'}
    save_new(output, document)
    return document


def check_plan(path):
    path = Path(path).resolve(strict=True)
    plan = json_read(path)
    if (plan.get('kind') != 'x650_response_preregistration' or plan.get('schema_version') != 1
            or not valid_created_time(plan.get('created_unix'))
            or plan.get('protocols') != {role: build_response_protocol(role) for role in ('train', 'holdout')}
            or plan.get('analysis_sources_sha256') != analysis_sources()):
        raise ValueError('Frozen preregistration/protocol/analysis source identity changed')
    for source, expected in plan['source_sha256'].items():
        if not Path(source).is_file() or sha(source) != require_hash(expected, source):
            raise ValueError('Preregistered source changed: '+source)
    if sha(plan['px4_binary_path']) != plan['px4_binary_sha256']:
        raise ValueError('Preregistered PX4 binary changed')
    archive = plan['source_archive']
    if sha(archive['path']) != archive['sha256']:
        raise ValueError('Preregistered immutable source archive changed')
    with zipfile.ZipFile(archive['path']) as zipped:
        if set(zipped.namelist()) != set(archive['members']):
            raise ValueError('Preregistered source archive member list changed')
        for member, record in archive['members'].items():
            if (hashlib.sha256(zipped.read(member)).hexdigest() != record['sha256']
                    or plan['source_sha256'].get(record['source_path']) != record['sha256']):
                raise ValueError('Archived source bytes differ from frozen manifest')
    return plan


def verify_plan(path, *, task=None):
    """Side-effect-free source/binary/template gate for the durable flight driver."""
    plan = check_plan(path)
    if task is not None:
        protocol = validate_response_protocol(task.get('response_probe_protocol'))
        if protocol is None:
            raise ValueError('An explicit frozen protocol is required')
        stripped = {key: value for key, value in task.items()
                    if key not in ('frozen_response_candidate', 'frozen_response_candidate_sha256')}
        if stripped != plan['task_templates'][protocol['role']]:
            raise ValueError('Flight task differs from its preregistered template')
        if protocol['role'] == 'holdout':
            candidate_path = task.get('frozen_response_candidate')
            if (not candidate_path or sha(candidate_path) != task.get('frozen_response_candidate_sha256')):
                raise ValueError('Holdout needs its exact frozen candidate before launch')
            candidate = json_read(candidate_path)
            if (candidate.get('preregistration_sha256') != sha(path)
                    or candidate.get('training_passed') is not True
                    or candidate.get('analysis_sources_sha256') != analysis_sources()
                    or candidate.get('protocols') != plan['protocols']):
                raise ValueError('Candidate does not belong to this preregistered plan')
            check_candidate(candidate, plan, path)
    return plan


def launch_start(run):
    paths = [Path(path) for path in run['artifacts'] if Path(path).name == 'launch.json']
    if len(paths) != 1:
        raise ValueError('Exactly one hash-bound launch.json required')
    value = json_read(paths[0])
    return datetime.fromisoformat(value['started_utc'].replace('Z', '+00:00')).timestamp()


def launch_finish(run):
    paths = [Path(path) for path in run['artifacts'] if Path(path).name == 'launch.json']
    if len(paths) != 1:
        raise ValueError('Exactly one hash-bound completed launch.json required')
    value = json_read(paths[0])
    if value.get('complete') is not True or value.get('runtime_closed') is not True:
        raise ValueError('Training launch must have completed with runtime closed')
    return datetime.fromisoformat(value['finished_utc'].replace('Z', '+00:00')).timestamp()


def check_candidate(candidate, plan, preregistration):
    if (candidate.get('kind') != 'x650_frozen_response_candidate' or candidate.get('schema_version') != 1
            or candidate.get('training_passed') is not True or candidate.get('identified') is not False
            or candidate.get('eligible_as_stage_gate') is not False
            or not valid_created_time(candidate.get('created_unix'))
            or candidate['created_unix'] <= plan['created_unix']
            or candidate.get('preregistration_sha256') != sha(preregistration)
            or candidate.get('analysis_sources_sha256') != analysis_sources()
            or candidate.get('protocols') != plan['protocols']
            or sha(candidate['training_report_path']) != candidate.get('training_report_sha256')):
        raise ValueError('Training candidate identity/protocol/analysis/report changed')
    training = json_read(candidate['training_report_path'])
    if (training.get('training_passed') is not True or training.get('parameters') != candidate['parameters']
            or training.get('identity') != candidate['identity'] or training.get('training') != candidate['training']
            or training.get('preregistration_sha256') != sha(preregistration)
            or training.get('analysis_sources_sha256') != analysis_sources()):
        raise ValueError('Candidate does not match its completed accepted training-only report')
    for path, expected in training['artifacts'].items():
        if sha(path) != expected:
            raise ValueError('Training evidence changed since fitting: '+path)
    if not plan['created_unix'] < launch_start(training) <= launch_finish(training) < candidate['created_unix']:
        raise ValueError('Candidate was not frozen after its completed preregistered training flight')
    return training


def check_run_plan(run, plan, role, candidate=None, candidate_path=None):
    if run['protocol'] != plan['protocols'][role]:
        raise ValueError('Wrong frozen experiment role/protocol')
    task = dict(run['metadata']['task'])
    if role == 'train':
        if any(task.get(key) is not None for key in ('frozen_response_candidate', 'frozen_response_candidate_sha256')):
            raise ValueError('Training cannot carry a fitted candidate')
        for key in ('frozen_response_candidate', 'frozen_response_candidate_sha256'):
            task.pop(key, None)
    else:
        if candidate is None or candidate_path is None:
            raise ValueError('Holdout requires a preflight-bound frozen training candidate')
        if (Path(task.pop('frozen_response_candidate', '')).resolve() != Path(candidate_path).resolve()
                or task.pop('frozen_response_candidate_sha256', None) != sha(candidate_path)):
            raise ValueError('Holdout task did not bind this exact frozen candidate before flight')
        if launch_start(run) <= candidate['created_unix']:
            raise ValueError('Holdout was started before its training candidate was frozen')
        launches = [Path(path) for path in run['artifacts'] if Path(path).name == 'launch.json']
        launch = json_read(launches[0])
        archive = launch.get('frozen_response_candidate_archive', {})
        archived = Path(archive.get('path', ''))
        if (archive.get('sha256') != sha(candidate_path) or not archived.is_file()
                or sha(archived) != sha(candidate_path)
                or run['artifacts'].get(str(archived.resolve())) != sha(candidate_path)
                or Path(archive.get('source_path', '')).resolve() != Path(candidate_path).resolve()):
            raise ValueError('Holdout launch lacks the exact hash-bound candidate archive')
    if task != plan['task_templates'][role]:
        raise ValueError('Executed task differs from the preregistered template')
    if launch_start(run) <= plan['created_unix']:
        raise ValueError('Experiment was started before preregistration')
    for relative, digest in run['metadata'].get('provenance', {}).get('source_files_sha256', {}).items():
        source = str((ROOT/relative).resolve())
        if plan['source_sha256'].get(source) != digest:
            raise ValueError('Executed controller source was not preregistered: '+relative)
    return True


def prepare_run(run):
    protocol = validate_response_protocol(run['protocol'])
    t = np.asarray(run['truth_times'], float)
    mask = interval_mask(t, protocol['prediction_window_s'])
    t, v = t[mask], np.asarray(run['velocities'], float)[mask]
    ct, u = np.asarray(run['command_times'], float), np.asarray(run['commands'], float)
    if (len(t) < 1000 or t[0] > .008 or t[-1] < 54.-.008 or v.shape != (len(t), 3)
            or u.shape != (len(ct), 3) or np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > .008+1e-9
            or np.any(np.diff(ct) <= 0) or not all(np.isfinite(x).all() for x in (t, v, ct, u))):
        raise ValueError('Complete original native 0-54 s trajectory and ordered commands required')
    commanded = ct < 54.
    expected = np.array([protocol_velocity(float(x), protocol) for x in ct[commanded]])
    if np.max(abs(u[commanded]-expected)) > 1e-9:
        raise ValueError('Actual logical commands violate the predeclared role sequence')
    quiet = interval_mask(t, (0., 1.))
    qcommand = interval_mask(ct, (0., 1.))
    if quiet.sum() < 100 or not qcommand.any():
        raise ValueError('Initial quiet segment is incomplete')
    centered = t[quiet]-np.mean(t[quiet])
    slope = centered@v[quiet]/float(centered@centered)
    peak = np.max(abs(v[quiet]), axis=0)
    if np.max(abs(u[qcommand])) > 1e-10 or np.max(peak) > .01 or np.max(abs(slope)) > .01:
        raise ValueError('Fixed a0=0 failed the frozen initial quiet check')
    joints = np.asarray(run['joints'], float)
    if np.max(abs(joints[:, :2])) > .03 or np.max(abs(joints[:, 2:])) > .05:
        raise ValueError('Arm was not within the frozen folded-arm identification limits')
    initialization = {'v0_m_s': v[0].tolist(), 'a0_m_s2': [0., 0., 0.], 'a0_fitted': False,
        'native_start_s': float(t[0]), 'quiet_window_s': [0., 1.], 'quiet_velocity_peak_m_s': peak.tolist(),
        'quiet_velocity_slope_m_s2': slope.tolist(), 'quiet_passed': True, 'state_reset_between_pulses': False}
    return {'times': t, 'velocities': v, 'command_times': ct, 'commands': u,
            'protocol': protocol, 'initialization': initialization}


def predict(data, axis, parameters, diagnostic_delay_s=0.):
    t, ct, u = data['times'], data['command_times'], data['commands'][:, axis]
    v0 = float(data['velocities'][0, axis])
    if axis < 2:
        return predict_second_order(parameters['wn_rad_s'], parameters['zeta'], ct+diagnostic_delay_s,
                                    u, t, v0, initial_acceleration=0.)[:, 0]
    return predict_velocity(parameters['tau_s'], ct, u, t, v0, diagnostic_delay_s=diagnostic_delay_s)


def score_window(data, axis):
    start, end = data['protocol']['axis_windows_s'][axis]
    return [max(1., start), end]


def excitation(data, axis):
    start, end = score_window(data, axis)
    mask = interval_mask(data['times'], (start, end))
    cmask = (data['command_times'] >= start-1e-9)&(data['command_times'] < end-1e-9)
    commands, velocity = data['commands'][cmask, axis], data['velocities'][mask, axis]
    amp = data['protocol']['amplitude_m_s'][axis]
    reasons = []
    if mask.sum() < 120 or not len(commands): reasons.append('Insufficient unique native samples or command coverage')
    elif np.min(commands) > -.99*amp or np.max(commands) < .99*amp: reasons.append('Missing positive or negative excitation')
    if len(velocity) and np.ptp(velocity) < .5*(2*amp): reasons.append('Actual velocity span is below frozen excitation threshold')
    return {'passed': not reasons, 'reasons': reasons, 'samples': int(mask.sum()),
            'velocity_span_m_s': float(np.ptp(velocity)) if len(velocity) else None,
            'command_amplitude_m_s': amp, 'window_s': [start, end]}


def fit_axis(data, axis):
    mask = interval_mask(data['times'], score_window(data, axis))
    truth = data['velocities'][:, axis]
    if axis < 2:
        lower, upper = np.log([.3, .1]), np.log([30., 5.])
        def residual(logparams):
            values = np.exp(logparams)
            return (predict(data, axis, {'wn_rad_s': values[0], 'zeta': values[1]})-truth)[mask]
        fits = [least_squares(residual, np.log(start), bounds=(lower, upper),
            ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=150) for start in ((2., .5), (5., 1.), (12., 2.))]
        converged = [fit for fit in fits if fit.success and np.isfinite(fit.fun).all()]
        if not converged: raise ValueError('Training optimizer failed: '+AXES[axis])
        fit = min(converged, key=lambda item: float(item.fun@item.fun))
        parameters = dict(zip(('wn_rad_s', 'zeta'), map(float, np.exp(fit.x))))
        logvalues = fit.x
    else:
        lower, upper = np.log([.01]), np.log([1.5])
        def objective(value):
            residual = (predict(data, axis, {'tau_s': math.exp(value)})-truth)[mask]
            return float(np.mean(residual**2))
        fit = minimize_scalar(objective, method='bounded', bounds=(float(lower[0]), float(upper[0])), options={'xatol': 1e-8})
        if not fit.success: raise ValueError('Training scalar optimizer failed')
        parameters, logvalues = {'tau_s': float(math.exp(fit.x))}, np.array([fit.x])
    prediction = predict(data, axis, parameters)
    mse = float(np.mean((prediction[mask]-truth[mask])**2))
    boundary = np.minimum((logvalues-lower)/(upper-lower), (upper-logvalues)/(upper-lower))
    sensitivity = {}
    for name, value in parameters.items():
        changes = []
        for factor in (.8, 1.2):
            changed = dict(parameters); changed[name] = value*factor
            neighbor_mse = float(np.mean((predict(data, axis, changed)[mask]-truth[mask])**2))
            changes.append((neighbor_mse-mse)/max(mse, 1e-16))
        sensitivity[name] = min(changes)
    identifiable = bool(np.all(boundary > 1e-4) and min(sensitivity.values()) >= .02)
    return parameters, {'fit_source': 'training only', 'fit_window_s': score_window(data, axis),
        'fit_success': True, 'identifiable': identifiable, 'training_mse_m2_s2': mse,
        'log_bound_distance_fraction': boundary.tolist(), 'relative_mse_increase_at_20percent_parameter_change': sensitivity}


def segment_metrics(data, axis, prediction, window):
    mask = interval_mask(data['times'], window)
    if mask.sum() < 2: raise ValueError('Scoring segment has insufficient samples')
    actual, expected = data['velocities'][mask, axis], prediction[mask]
    error = expected-actual
    amp = data['protocol']['amplitude_m_s'][axis]
    rms = float(np.sqrt(np.mean(error**2)))
    cross = float(np.max(np.sqrt(np.mean(np.delete(data['velocities'][mask], axis, axis=1)**2, axis=0))))
    values = {'rmse_m_s': rms, 'nrmse_command_half_amplitude': rms/amp,
        'maximum_error_m_s': float(np.max(abs(error))), 'bias_m_s': float(np.mean(error)),
        'peak_error_m_s': float(max(abs(np.max(expected)-np.max(actual)), abs(np.min(expected)-np.min(actual)))),
        'cross_axis_rms_m_s': cross}
    criteria = {'rmse': rms <= (.010 if axis == 2 else .015), 'nrmse': rms/amp <= .20,
        'maximum_error': values['maximum_error_m_s'] <= .35*amp+.002,
        'bias': abs(values['bias_m_s']) <= .10*amp+.001,
        'peak_error': values['peak_error_m_s'] <= .25*amp+.002,
        'cross_axis': cross <= .30*amp+.003}
    return {'window_s': list(window), 'samples': int(mask.sum()), **values,
            'criteria': criteria, 'passed': all(criteria.values())}


def evaluate_axis(data, axis, parameters):
    prediction = predict(data, axis, parameters)
    whole = segment_metrics(data, axis, prediction, score_window(data, axis))
    start, end = data['protocol']['axis_windows_s'][axis]
    pulses = data['protocol']['local_pulses']
    segments = []
    for index, pulse in enumerate(pulses):
        a, b = start+pulse['start_s'], start+pulse['end_s']
        c = start+pulses[index+1]['start_s'] if index+1 < len(pulses) else end
        segments.append({'kind': 'active_pulse', 'index': index+1,
                         **segment_metrics(data, axis, prediction, (a, b))})
        segments.append({'kind': 'zero_velocity_rest', 'index': index+1,
                         **segment_metrics(data, axis, prediction, (b, c))})
    mask = interval_mask(data['times'], score_window(data, axis))
    errors = []
    delays = np.arange(0., .1201, .005)
    for delay in delays:
        delayed = predict(data, axis, parameters, diagnostic_delay_s=float(delay))
        errors.append(float(np.sqrt(np.mean((delayed[mask]-data['velocities'][mask, axis])**2))))
    best = int(np.argmin(errors)); improvement = whole['rmse_m_s']-errors[best]
    material = bool(delays[best] > .030+1e-9 and improvement >= .001
                    and improvement >= .20*max(whole['rmse_m_s'], 1e-12))
    excite = excitation(data, axis)
    reasons = [key+' whole-window criterion failed' for key, ok in whole['criteria'].items() if not ok]
    reasons += [item['kind']+' '+str(item['index'])+' failed: '+','.join(k for k,v in item['criteria'].items() if not v)
                for item in segments if not item['passed']]
    reasons += excite['reasons']
    if material: reasons.append('Material unmodeled command delay; zero-delay model rejected')
    return {'passed': not reasons, 'reasons': reasons, 'excitation': excite,
        'whole_axis_window': whole, 'pulse_and_zero_rest_segments': segments,
        'delay_diagnostic': {'best_delay_s': float(delays[best]), 'zero_delay_rmse_m_s': whole['rmse_m_s'],
            'best_rmse_m_s': errors[best], 'improvement_m_s': improvement,
            'material_unmodeled_delay': material, 'applied_to_model': False}}, prediction


def summary(run, data):
    return {**{key: run[key] for key in ('folder', 'command_csv_sha256', 'normalized_measurement_sha256')},
            'initialization': data['initialization'], 'protocol_sha256': response_protocol_hash(run['protocol'])}


def fit_training(train, train_audit, preregistration, output):
    output = Path(output).resolve()
    if output.exists(): raise ValueError('Use a new training output directory')
    output.mkdir(parents=True)
    try:
        plan = check_plan(preregistration)
        run = load_protocol_run(train, train_audit)
        check_run_plan(run, plan, 'train')
        if not launch_start(run) <= launch_finish(run) <= time.time():
            raise ValueError('Training launch chronology is invalid or still in progress')
        data = prepare_run(run)
        parameters, axes = {}, []
        for axis, name in enumerate(AXES):
            parameters[name], fit = fit_axis(data, axis)
            metrics, _ = evaluate_axis(data, axis, parameters[name])
            axes.append({'axis': name, 'parameters': parameters[name], 'fit': fit, 'training': metrics,
                         'passed': fit['identifiable'] and metrics['passed']})
        passed = all(axis['passed'] for axis in axes)
        report = {'kind': 'x650_response_training_report', 'schema_version': 1, 'complete': True,
            'passed': passed, 'training_passed': passed, 'identified': False, 'eligible_as_stage_gate': False,
            'parameters': parameters, 'axes': axes, 'training': summary(run, data), 'identity': run['identity'],
            'preregistration_path': str(Path(preregistration).resolve()), 'preregistration_sha256': sha(preregistration),
            'analysis_sources_sha256': analysis_sources(), 'artifacts': run['artifacts'],
            'parameter_selection': 'Only the fresh training run is read; no holdout input is accepted by this command'}
        report_path = output/'training_report.json'; save_new(report_path, report)
        if passed:
            candidate = {'kind': 'x650_frozen_response_candidate', 'schema_version': 1,
                'training_passed': True, 'identified': False, 'eligible_as_stage_gate': False,
                'created_unix': time.time(), 'parameters': parameters, 'training': report['training'],
                'identity': run['identity'], 'protocols': plan['protocols'],
                'preregistration_path': str(Path(preregistration).resolve()), 'preregistration_sha256': sha(preregistration),
                'training_report_path': str(report_path), 'training_report_sha256': sha(report_path),
                'analysis_sources_sha256': analysis_sources(),
                'scope': 'Training-only candidate frozen before holdout; not an accepted model or flight-stage certificate'}
            save_new(output/'frozen_response_candidate.json', candidate)
        return report
    except Exception as exc:
        if not (output/'training_report.json').exists():
            save_new(output/'training_report.json', {'kind': 'x650_response_training_report', 'complete': False,
                'passed': False, 'training_passed': False, 'identified': False, 'eligible_as_stage_gate': False,
                'error': repr(exc)})
        raise


def validate_holdout(frozen_candidate, holdout, holdout_audit, preregistration, output):
    output = Path(output).resolve()
    if output.exists(): raise ValueError('Use a new validation output directory')
    output.mkdir(parents=True)
    try:
        plan = check_plan(preregistration)
        candidate = json_read(frozen_candidate)
        training = check_candidate(candidate, plan, preregistration)
        run = load_protocol_run(holdout, holdout_audit)
        check_run_plan(run, plan, 'holdout', candidate, frozen_candidate)
        if run['identity'] != candidate['identity']:
            raise ValueError('Training/holdout model/PID/operating identity differs')
        if any(run[key] == candidate['training'][key] for key in
               ('folder', 'command_csv_sha256', 'normalized_measurement_sha256')):
            raise ValueError('Holdout must be a fresh independently recorded experiment')
        data = prepare_run(run)
        axes, predictions = [], {}
        for axis, name in enumerate(AXES):
            result, predictions[name] = evaluate_axis(data, axis, candidate['parameters'][name])
            axes.append({'axis': name, 'parameters': candidate['parameters'][name], 'fit_source': 'frozen fresh training only',
                         'training': training['axes'][axis], 'holdout': result, 'passed': result['passed']})
        passed = all(axis['passed'] for axis in axes)
        report = {'kind': 'x650_native_stage_evidence', 'schema_version': 3, 'stage': 'response_identification',
            'simulation_backend': 'gazebo_px4', 'complete': True, 'passed': passed, 'success': passed,
            'identified': passed, 'eligible_as_stage_gate': passed, 'research_acceptance_verified': passed,
            'ground_truth_aided': True, 'diagnostic_only': False, **candidate['identity'],
            'full_window_ulog_verified': True, 'parameter_restore_verified': True,
            'upper_command_verified': True, 'feedback_contract_verified': True,
            'parameters': candidate['parameters'], 'model_structure': plan['protocols']['train']['model_structure'],
            'protocols': plan['protocols'], 'training': candidate['training'], 'holdout': summary(run, data),
            'axes': axes, 'reasons': [name+': '+reason for name, axis in zip(AXES, axes) for reason in axis['holdout']['reasons']],
            'frozen_candidate_path': str(Path(frozen_candidate).resolve()), 'frozen_candidate_sha256': sha(frozen_candidate),
            'preregistration_path': str(Path(preregistration).resolve()), 'preregistration_sha256': sha(preregistration),
            'analysis_sources_sha256': analysis_sources(), 'gain': [1., 1., 1.], 'command_delay_s': 0., 'bias_m_s': [0., 0., 0.],
            'acceleration_feedforward_contract': 'Zero acceleration feedforward only; no active FF validation',
            'prediction_contract': 'Exact original command ZOH; continuous 0-54s prediction, once measured v0 and fixed a0=0, no phase/bias fit or pulse reset',
            'artifacts': {**training['artifacts'], **run['artifacts'], str(Path(frozen_candidate).resolve()): sha(frozen_candidate),
                          str(Path(candidate['training_report_path']).resolve()): candidate['training_report_sha256'],
                          plan['source_archive']['path']: plan['source_archive']['sha256'],
                          str(Path(preregistration).resolve()): sha(preregistration)},
            'limitations': ['Folded-arm base velocity response only', 'No moving-arm, torque-level or hardware model validation',
                            'No MPC performance claim', 'Independent fresh holdout never adjusts model parameters']}
        if passed:
            model = {'kind': 'x650_mixed_upper_velocity_response_model', 'schema_version': 1,
                'identified': True, 'passed': True, 'response_identified': True, 'eligible_as_stage_gate': True,
                'research_acceptance_verified': True, 'diagnostic_only': False, 'ground_truth_aided': True,
                **candidate['identity'], 'model_structure': report['model_structure'], 'parameters': candidate['parameters'],
                'state_order': ['vx', 'ax', 'vy', 'ay', 'vz'], 'gain': [1., 1., 1.], 'command_delay_s': 0.,
                'bias_m_s': [0., 0., 0.], 'acceleration_feedforward_contract': report['acceleration_feedforward_contract'],
                'joint_response_identified': False, 'full_body_dynamics_identified': False,
                'evidence': 'response_identification.json', 'frozen_candidate_sha256': sha(frozen_candidate),
                'preregistration_sha256': sha(preregistration), 'analysis_sources_sha256': analysis_sources()}
            save_new(output/'response_model.json', model)
            report['artifacts']['response_model.json'] = sha(output/'response_model.json')
        save_new(output/'response_identification.json', report)
        # Compact original-time holdout trajectories for plotting; not a certificate.
        with (output/'holdout_predictions.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(['t_s']+[prefix+axis for prefix in ('native_v', 'predicted_v') for axis in AXES])
            writer.writerows([float(t), *data['velocities'][index].tolist(), *[float(predictions[a][index]) for a in AXES]]
                             for index, t in enumerate(data['times']))
        return report
    except Exception as exc:
        if not (output/'response_identification.json').exists():
            save_new(output/'response_identification.json', {'kind': 'x650_native_stage_evidence',
                'stage': 'response_identification', 'complete': False, 'passed': False, 'identified': False,
                'eligible_as_stage_gate': False, 'error': repr(exc)})
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    freeze = sub.add_parser('freeze')
    freeze.add_argument('--source', action='append', default=[], type=Path)
    for name in ('train-task', 'holdout-task', 'px4-binary', 'preflight', 'output'):
        freeze.add_argument('--'+name, required=True, type=Path)
    fit = sub.add_parser('fit')
    for name in ('train', 'train-audit', 'preregistration', 'output'): fit.add_argument('--'+name, required=True, type=Path)
    validate = sub.add_parser('validate')
    for name in ('frozen-candidate', 'holdout', 'holdout-audit', 'preregistration', 'output'):
        validate.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args(argv)
    if args.command == 'freeze':
        freeze_plan(args.output, args.source, {'train': args.train_task, 'holdout': args.holdout_task}, args.px4_binary, args.preflight)
        result = {'complete': True, 'preregistration_sha256': sha(args.output)}
    elif args.command == 'fit': result = fit_training(args.train, args.train_audit, args.preregistration, args.output)
    else: result = validate_holdout(args.frozen_candidate, args.holdout, args.holdout_audit, args.preregistration, args.output)
    print(json.dumps({'output': str(args.output), **{key: result[key] for key in ('complete', 'passed', 'identified', 'parameters', 'reasons', 'preregistration_sha256') if key in result}}, indent=2))
    return 0 if result.get('passed', True) else 2


if __name__ == '__main__':
    raise SystemExit(main())
