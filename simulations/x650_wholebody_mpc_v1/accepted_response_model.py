"""Read-only admission of the accepted, independently validated response model.

Historical source files may evolve only when their exact prior bytes remain in
the preregistered archive. Raw measurements, reports and analysis scripts must
still match their original files. This module never fits or repairs evidence.
"""
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time
import zipfile

from research_condition import (canonical_hash, condition_for_task, expected_gate_condition,
                                is_mixed_mpc_research)
from response_protocol import build_response_protocol

MODEL_KIND = "x650_mixed_upper_velocity_response_model"
STRUCTURE = {"x": "unit_gain_zero_delay_second_order", "y": "unit_gain_zero_delay_second_order",
             "z": "unit_gain_zero_delay_first_order"}
FF_CONTRACT = "Zero acceleration feedforward only; no active FF validation"
IDENTITY_KEYS = ("model_sha256", "source_urdf_sha256", "airframe_sha256", "flight_profile_sha256",
                 "preparation_report_sha256", "pid_parameters_sha256", "operating_condition",
                 "operating_condition_sha256", "feedback_contract", "control_interface")
VERIFIED = ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
            "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")
RAW_RUN_FILES = ("stage_result.json", "controller_metadata.json", "controller_task.json", "control_trace.csv",
                "feedback_trace.jsonl", "estimated_feedback.json", "native_trace.csv", "native_trace_manifest.json", "px4.ulg",
                "diagnostic_parameter_session.json", "diagnostic_parameters_before.json", "diagnostic_parameters_applied.json",
                "diagnostic_parameters_restored.json", "diagnostic_restore_journal_completed.json",
                "px4_parameters_before.json", "px4_parameters_after.json")


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    require(isinstance(value, dict), "Evidence must be a JSON object: "+str(path))
    return value


def valid_hash(value):
    return isinstance(value, str) and len(value) == 64 and not set(value)-set("0123456789abcdef")


def absolute_file(value):
    require(isinstance(value, str) and Path(value).is_absolute() and Path(value).is_file(),
            "An existing absolute evidence file is required: "+str(value))
    return Path(value).resolve()


def checked_file(path, digest):
    path = absolute_file(str(path))
    require(valid_hash(digest) and sha(path) == digest, "Evidence SHA-256 mismatch: "+str(path))
    return path


def validate_model_structure(model):
    require(isinstance(model, dict) and model.get("kind") == MODEL_KIND
            and type(model.get("schema_version")) is int and model["schema_version"] == 1
            and all(model.get(k) is True for k in ("identified", "passed", "response_identified",
                    "eligible_as_stage_gate", "research_acceptance_verified", "ground_truth_aided"))
            and model.get("diagnostic_only") is False,
            "Only a formally accepted mixed response model is admissible")
    require(model.get("model_structure") == STRUCTURE and model.get("state_order") == ["vx", "ax", "vy", "ay", "vz"]
            and canonical_hash(model.get("gain")) == canonical_hash([1., 1., 1.])
            and canonical_hash(model.get("bias_m_s")) == canonical_hash([0., 0., 0.])
            and canonical_hash(model.get("command_delay_s")) == canonical_hash(0.)
            and model.get("acceleration_feedforward_contract") == FF_CONTRACT
            and model.get("joint_response_identified") is False and model.get("full_body_dynamics_identified") is False
            and "base_time_constant_s" not in model,
            "Accepted response has an unsupported gain, delay, structure or scope")
    parameters = model.get("parameters")
    require(isinstance(parameters, dict) and set(parameters) == set("xyz"), "Exactly three response axes are required")
    bounds = {"wn_rad_s": (.3, 30.), "zeta": (.1, 5.), "tau_s": (.01, 1.5)}
    for axis in "xyz":
        expected = {"tau_s"} if axis == "z" else {"wn_rad_s", "zeta"}
        require(isinstance(parameters[axis], dict) and set(parameters[axis]) == expected,
                "Unsupported response parameters: "+axis)
        for name, value in parameters[axis].items():
            lo, hi = bounds[name]
            require(not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
                    and lo < value < hi, "Response parameter is unstable or outside frozen bounds: "+axis+"."+name)
    return deepcopy(parameters)


def runtime_px4_binary():
    from amsim.paths import px4_dir
    return px4_dir()/"build/px4_sitl_default/bin/px4"


def verify_source_archive(plan):
    archive = plan.get("source_archive", {})
    path = checked_file(archive.get("path"), archive.get("sha256"))
    sources, members = plan.get("source_sha256"), archive.get("members")
    require(isinstance(sources, dict) and sources and isinstance(members, dict) and members,
            "Preregistration must retain its immutable source manifest")
    archived = {}
    with zipfile.ZipFile(path) as zipped:
        require(len(zipped.namelist()) == len(set(zipped.namelist()))
                and set(zipped.namelist()) == set(members), "Immutable source archive member set changed")
        for member, record in members.items():
            source, digest = record["source_path"], record["sha256"]
            require(isinstance(source, str) and Path(source).is_absolute() and source not in archived
                    and valid_hash(digest) and sources.get(source) == digest
                    and hashlib.sha256(zipped.read(member)).hexdigest() == digest,
                    "Immutable source archive bytes differ from preregistration")
            archived[source] = digest
    require(archived == sources, "Immutable archive must cover every preregistered source")
    # The accepted analysis is intentionally untouched; historical-source
    # substitution only covers evolving controller/platform source files.
    analysis = plan.get("analysis_sources_sha256")
    require(isinstance(analysis, dict) and analysis, "Frozen analysis identity is missing")
    for source, digest in analysis.items():
        require(archived.get(source) == digest, "Analysis source was not preregistered")
        checked_file(source, digest)
    return archived


def verify_artifacts(artifacts, root, archived_sources=None):
    require(isinstance(artifacts, dict) and artifacts, "Hashed raw artifacts are required")
    verified = {}
    historical = archived_sources or {}
    for original, digest in artifacts.items():
        require(isinstance(original, str) and valid_hash(digest), "Invalid artifact path or hash")
        path = Path(original)
        path = (path if path.is_absolute() else Path(root)/path).resolve()
        if path.is_file() and sha(path) == digest:
            verified[str(path)] = digest
        else:
            require(path.suffix in (".py", ".msg") and historical.get(str(path)) == digest,
                    "Native evidence artifact mismatch: "+str(path))
            verified[str(path)] = digest
    return verified


def _same(left, right, keys, message):
    require(all(canonical_hash(left.get(k)) == canonical_hash(right.get(k)) for k in keys), message)


def _axes(document, holdout=False):
    axes = document.get("axes")
    require(isinstance(axes, list) and [a.get("axis") for a in axes] == list("xyz"), "All accepted axes must be present")
    for axis in axes:
        require(axis.get("passed") is True and axis.get("parameters") == document["parameters"][axis["axis"]],
                "Accepted axis parameters/status differ")
        training = axis.get("training")
        if holdout:
            require(axis.get("holdout", {}).get("passed") is True
                    and not axis["holdout"].get("reasons"), "Holdout axis was not accepted")
            require(isinstance(training, dict) and training.get("passed") is True, "Training axis was not accepted")
            training = training.get("training")
        else:
            require(axis.get("fit", {}).get("identifiable") is True, "Training fit was not identifiable")
        require(isinstance(training, dict) and training.get("passed") is True and not training.get("reasons"),
                "Training axis was not accepted")


def _stamp(launch, key):
    try:
        stamp = datetime.fromisoformat(launch[key].replace("Z", "+00:00")).timestamp()
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Missing valid launch chronology") from exc
    require(math.isfinite(stamp) and 0 < stamp <= time.time(), "Launch time is invalid")
    return stamp


def _run_chain(role, summary, files, plan, candidate, candidate_path):
    folder = Path(summary["folder"]).resolve()
    launches = []
    for path in files:
        if Path(path).name == "launch.json":
            value = read(path)
            if [Path(p).resolve() for p in value.get("run_directories", [])] == [folder]:
                launches.append((Path(path), value))
    require(len(launches) == 1, "Exactly one original launch is required for "+role)
    path, launch = launches[0]
    require(launch.get("complete") is True and launch.get("runtime_closed") is True and launch.get("exit_code") == 0,
            "Response launch did not complete and close")
    task = deepcopy(launch.get("task", {}))
    if role == "holdout":
        require(task.pop("frozen_response_candidate", None) == str(candidate_path)
                and task.pop("frozen_response_candidate_sha256", None) == sha(candidate_path),
                "Holdout did not bind this exact frozen candidate before flight")
        archived = launch.get("frozen_response_candidate_archive", {})
        require(archived.get("source_path") == str(candidate_path) and archived.get("sha256") == sha(candidate_path),
                "Holdout candidate archive identity differs")
        archived_path = checked_file(archived.get("path"), archived.get("sha256"))
        require(files.get(str(archived_path)) == sha(candidate_path), "Holdout candidate archive is not hash-bound")
    require(task == plan["task_templates"][role], "Response task differs from its preregistered role")
    task_path, archive = path.parent/"task.json", path.parent/"controller_sources.zip"
    require(files.get(str(task_path)) == sha(task_path) and read(task_path) == launch["task"]
            and files.get(str(archive)) == sha(archive), "Original launch task/source archive is missing")
    sources = launch.get("provenance", {}).get("source_files_sha256", {})
    require(isinstance(sources, dict) and sources, "Original launch source identity missing")
    with zipfile.ZipFile(archive) as zipped:
        require(len(zipped.namelist()) == len(set(zipped.namelist())) and set(zipped.namelist()) == set(sources),
                "Launch source archive member set differs")
        for member, digest in sources.items():
            matches = [value for source, value in plan["source_sha256"].items()
                       if Path(source).as_posix().endswith("/"+member)]
            require(matches == [digest] and hashlib.sha256(zipped.read(member)).hexdigest() == digest,
                    "Launch source is not the preregistered version")
    for name in RAW_RUN_FILES:
        require(str(folder/name) in files, "Response raw run artifact is missing: "+name)
    metadata = read(folder/"controller_metadata.json")
    require(metadata.get("task") == launch["task"] and metadata.get("provenance") == launch.get("provenance"),
            "Original measured metadata differs from launch task/provenance")
    require(read(folder/"controller_task.json") == launch["task"]
            and metadata.get("stage") == "response_probe" and metadata.get("controller") == "pid_ff"
            and metadata.get("complete") is True and metadata.get("landed_and_disarmed") is True
            and metadata.get("diagnostic_only") is False and metadata.get("ground_truth_aided") is True
            and metadata.get("eligible_as_stage_gate") is False and metadata.get("acceptance_pending") == "full_ulog_audit",
            "Original response metadata is incomplete, relabeled or diagnostic")
    _same(metadata.get("provenance", {}), candidate["identity"], IDENTITY_KEYS[:5], "Original physical model identity differs")
    from diagnostic_parameters import load_parameter_evidence
    actual = metadata.get("actual_pid_parameters")
    require(isinstance(actual, dict) and set(actual) == set(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in actual.values())
            and canonical_hash({key: float(value) for key, value in actual.items()}) == candidate["identity"]["pid_parameters_sha256"],
            "Original actual PID readback differs from the accepted response")
    manifest = read(folder/"native_trace_manifest.json")
    require(manifest.get("complete") is True and manifest.get("sha256") == files[str(folder/"native_trace.csv")],
            "Original native recorder completeness/hash differs")
    certificates = []
    for original in files:
        if Path(original).name == "research_stage.json":
            document = read(original)
            if Path(document.get("source_run_directory", "")).resolve() == folder:
                certificates.append((Path(original), document))
    require(len(certificates) == 1, "Exactly one independent physical acceptance is required: "+role)
    certificate_path, certificate = certificates[0]
    audit_path = certificate_path.parent/"upper_stage_audit.json"
    require(str(audit_path) in files, "Independent response audit is missing")
    audit = read(audit_path)
    require(certificate.get("kind") == "x650_native_stage_evidence"
            and audit.get("kind") == "x650_precision_response_protocol_audit"
            and certificate.get("eligible_as_stage_gate") is True,
            "Original response physical certificate kind/status differs")
    for document, artifact_key in ((certificate, "artifacts"), (audit, "inputs_sha256")):
        require(all(document.get(key) is True for key in VERIFIED)
                and document.get("stage") == "response_probe" and document.get("simulation_backend") == "gazebo_px4"
                and document.get("response_probe_protocol") == plan["protocols"][role]
                and document.get("response_probe_protocol_sha256") == canonical_hash(plan["protocols"][role])
                and document.get("operating_condition") == candidate["identity"]["operating_condition"],
                "Original physical response acceptance differs from its role/condition")
        bound = document.get(artifact_key)
        require(isinstance(bound, dict) and bound and all(files.get(str(Path(p).resolve())) == digest for p, digest in bound.items()),
                "Final response certificate omitted original physical audit inputs")
    require(files[str(folder/"control_trace.csv")] == summary["command_csv_sha256"], "Original command CSV differs")
    require(any(Path(p).suffix == ".ulg" and Path(p).is_relative_to(folder) for p in files), "Full original ULog is missing")
    start, finish = _stamp(launch, "started_utc"), _stamp(launch, "finished_utc")
    require(plan["created_unix"] < start <= finish, "Response was not flown after preregistration")
    if role == "holdout":
        require(candidate["created_unix"] < start, "Holdout preceded the parameter freeze")
    return start, finish


def load_accepted_response_model(cfg):
    """Return ``model`` and an explicit archival ``binding``; never refit."""
    require(is_mixed_mpc_research(cfg), "Accepted mixed response admission requires the v3 profile")
    model_path = absolute_file(cfg.get("response_model"))
    model = read(model_path)
    validate_model_structure(model)
    evidence_path = absolute_file(cfg.get("stage_evidence", {}).get("response_identification"))
    require(model.get("evidence") == "response_identification.json"
            and evidence_path == model_path.parent/"response_identification.json", "Model is not bound to its final certificate")
    evidence = read(evidence_path)
    require(evidence.get("kind") == "x650_native_stage_evidence" and evidence.get("schema_version") == 3
            and type(evidence.get("schema_version")) is int
            and evidence.get("stage") == "response_identification" and evidence.get("simulation_backend") == "gazebo_px4"
            and all(evidence.get(k) is True for k in VERIFIED+("identified", "eligible_as_stage_gate", "success", "ground_truth_aided"))
            and evidence.get("diagnostic_only") is False and evidence.get("reasons") == [], "Final response certificate was not accepted")
    condition = condition_for_task(cfg)
    expected, role = expected_gate_condition(condition, "response_identification", cfg["stage"])
    require(canonical_hash(model.get("operating_condition")) == canonical_hash(expected)
            and model.get("operating_condition_sha256") == canonical_hash(expected)
            and model.get("model_sha256") == expected["model_sha256"]
            and model.get("feedback_contract") == expected["feedback_contract"]
            and model.get("control_interface") == expected["control_interface"]
            and model.get("pid_parameters_sha256") == expected["pid_parameters_sha256"], "Accepted model belongs to another operating condition")
    require(all(valid_hash(model.get(key)) for key in IDENTITY_KEYS if key.endswith("sha256")),
            "Accepted physical/controller identity requires valid hashes")
    _same(model, evidence, IDENTITY_KEYS+("parameters", "model_structure", "gain", "bias_m_s", "command_delay_s",
          "acceleration_feedforward_contract", "frozen_candidate_sha256", "preregistration_sha256", "analysis_sources_sha256"),
          "Model and final response certificate disagree")
    artifacts = evidence.get("artifacts", {})
    require(artifacts.get("response_model.json") == sha(model_path), "Certificate does not bind these exact model bytes")
    plan_path = checked_file(evidence.get("preregistration_path"), evidence.get("preregistration_sha256"))
    plan = read(plan_path)
    protocols = {name: build_response_protocol(name) for name in ("train", "holdout")}
    require(plan.get("kind") == "x650_response_preregistration" and type(plan.get("schema_version")) is int and plan["schema_version"] == 1
            and canonical_hash(plan.get("protocols")) == canonical_hash(protocols) == canonical_hash(evidence.get("protocols"))
            and plan.get("analysis_sources_sha256") == model.get("analysis_sources_sha256")
            and isinstance(plan.get("created_unix"), (int, float)) and not isinstance(plan["created_unix"], bool)
            and 0 < plan["created_unix"] <= time.time(), "Final response differs from its preregistered experiment")
    archived = verify_source_archive(plan)
    binary = checked_file(plan["px4_binary_path"], plan["px4_binary_sha256"])
    require(sha(runtime_px4_binary()) == sha(binary), "Current PX4 binary differs from the identified binary")
    files = verify_artifacts(artifacts, evidence_path.parent, archived)
    require(files.get(str(plan_path)) == sha(plan_path)
            and files.get(str(Path(plan["source_archive"]["path"]).resolve())) == plan["source_archive"]["sha256"],
            "Final certificate does not bind its preregistration and source archive")
    candidate_path = checked_file(evidence.get("frozen_candidate_path"), evidence.get("frozen_candidate_sha256"))
    require(files.get(str(candidate_path)) == sha(candidate_path), "Frozen candidate is not in certified artifacts")
    candidate = read(candidate_path)
    require(candidate.get("kind") == "x650_frozen_response_candidate" and type(candidate.get("schema_version")) is int and candidate["schema_version"] == 1
            and candidate.get("training_passed") is True and candidate.get("identified") is False
            and candidate.get("eligible_as_stage_gate") is False and canonical_hash(candidate.get("parameters")) == canonical_hash(model["parameters"])
            and canonical_hash(candidate.get("protocols")) == canonical_hash(protocols) and candidate.get("training") == evidence.get("training")
            and candidate.get("preregistration_path") == str(plan_path)
            and candidate.get("preregistration_sha256") == sha(plan_path)
            and candidate.get("analysis_sources_sha256") == plan["analysis_sources_sha256"], "Accepted model differs from its training freeze")
    _same(candidate.get("identity", {}), model, IDENTITY_KEYS, "Candidate physical/controller identity differs")
    created = candidate.get("created_unix")
    require(isinstance(created, (int, float)) and not isinstance(created, bool) and math.isfinite(created)
            and plan["created_unix"] < created <= time.time(), "Frozen candidate creation time is invalid")
    training_path = checked_file(candidate.get("training_report_path"), candidate.get("training_report_sha256"))
    require(files.get(str(training_path)) == sha(training_path), "Training report is not certified")
    training = read(training_path)
    require(training.get("kind") == "x650_response_training_report" and type(training.get("schema_version")) is int and training["schema_version"] == 1
            and all(training.get(k) is True for k in ("complete", "passed", "training_passed"))
            and training.get("identified") is False and training.get("eligible_as_stage_gate") is False
            and training.get("identity") == candidate["identity"], "Training-only acceptance is missing")
    _same(training, candidate, ("parameters", "training", "preregistration_path", "preregistration_sha256", "analysis_sources_sha256"),
          "Frozen candidate differs from the original training report")
    training_files = verify_artifacts(training.get("artifacts"), training_path.parent, archived)
    require(all(files.get(path) == digest for path, digest in training_files.items()), "Final certificate omitted training evidence")
    _axes(training)
    _axes(evidence, holdout=True)
    require(all(evidence["training"][key] != evidence["holdout"][key] for key in
                ("folder", "command_csv_sha256", "normalized_measurement_sha256")), "Holdout is not a fresh independent record")
    for axis, reference in zip(evidence["axes"], training["axes"]):
        require(axis["training"] == reference, "Final certificate changed original training axis evidence")
    _, training_finish = _run_chain("train", evidence["training"], files, plan, candidate, candidate_path)
    require(training_finish < candidate["created_unix"], "Candidate was frozen before training completed")
    _run_chain("holdout", evidence["holdout"], files, plan, candidate, candidate_path)
    def record(path):
        return {"path": str(Path(path).resolve()), "sha256": sha(path)}
    binding = {"kind": "x650_accepted_response_binding", "schema_version": 1, "role": role,
        "model": record(model_path), "evidence": record(evidence_path), "preregistration": record(plan_path),
        "source_archive": record(plan["source_archive"]["path"]), "candidate": record(candidate_path),
        "training_report": record(training_path), "px4_binary": record(binary),
        "source_operating_condition_sha256": canonical_hash(expected),
        "target_operating_condition_sha256": canonical_hash(condition),
        "verified_artifacts": files, "archived_source_sha256": archived,
        "scope": "Nominal folded-arm base response only; not a moving-arm or MPC performance certificate"}
    return {"model": model, "binding": binding}
