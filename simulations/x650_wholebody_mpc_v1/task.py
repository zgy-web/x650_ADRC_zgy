"""Immutable X650 commissioning inputs and measured-evidence stage gates."""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
CONFIG_ENV = "X650_WHOLEBODY_TASK_CONFIG"
MODEL = "amsim_urdf_x650_2link_v2_flight"
STAGES = ("hover_folded", "joint_probe", "response_probe", "tip_hold", "small", "normal", "fast")
UPPER_RESEARCH_SOURCES = (
    "simulations/x650_wholebody_mpc_v1/estimated_feedback.py",
    "src/amsim/api/node.py",
    "vendor/px4_msgs/msg/VehicleLocalPosition.msg",
    "vendor/px4_msgs/msg/VehicleAttitude.msg",
    "vendor/px4_msgs/msg/VehicleAngularVelocity.msg",
    "vendor/px4_msgs/msg/EstimatorStatusFlags.msg",
    "scripts/configure_px4_estimator_telemetry.py",
    "scripts/configure_px4_upper_logging.py",
)
for folder in (Path(__file__).resolve().parent, ROOT/"src", ROOT/"control_packages/x650_wholebody_v1"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))
from diagnostic_parameters import validate_profile, validate_pid_candidate, assert_no_pending_journal, PRECISION_PROFILE
from research_condition import (validate_research_profile, condition_for_task, canonical_hash,
                                is_upper_research, expected_gate_condition, UPPER_RESEARCH_STAGES,
                                UPPER_RESEARCH_PROFILE_NAME, is_mixed_mpc_research,
                                MIXED_MPC_PROFILE_NAME, MIXED_MPC_STAGES)
from response_protocol import validate_response_protocol, protocol_annotation
from mixed_mpc_protocol import (validate_mixed_mpc_protocol, protocol_hash as mixed_protocol_hash,
                                protocol_annotation as mixed_protocol_annotation, controller_config)
from accepted_response_model import load_accepted_response_model


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_files_for_task(cfg):
    sources = [*sorted((ROOT/"control_packages/x650_wholebody_v1/x650_wholebody").glob("*.py")),
               *sorted((ROOT/"simulations/x650_wholebody_mpc_v1").glob("*.py")),
               ROOT/"simulations/x650_hover_v1/parameter_evidence.py"]
    if is_upper_research(cfg):
        for relative in UPPER_RESEARCH_SOURCES:
            source = ROOT/relative
            if not source.is_file():
                raise ValueError("Upper research feedback source evidence is missing: "+relative)
            sources.append(source)
    return list(dict.fromkeys(sources))


def build_task_config(stage="hover_folded", controller="pid_ff", *, response_probe_protocol=None, mixed_mpc_protocol=None):
    if stage not in STAGES or controller not in ("pid_ff", "mpc"):
        raise ValueError("Unsupported stage/controller")
    if stage in ("hover_folded", "joint_probe", "response_probe") and controller != "pid_ff":
        raise ValueError("Commissioning probes require pid_ff; MPC follows measured plant validation")
    period = 8. if stage == "fast" else 16. if stage == "small" else 12.
    trajectory = stage if stage in ("small", "normal", "fast") else "hover"
    protocol = validate_response_protocol(response_probe_protocol)
    mixed_protocol = validate_mixed_mpc_protocol(mixed_mpc_protocol)
    if mixed_protocol is not None and (stage not in MIXED_MPC_STAGES or protocol is not None):
        raise ValueError("Frozen mixed MPC protocol requires a comparison stage and excludes response excitation")
    if protocol is not None and (stage != "response_probe" or controller != "pid_ff"):
        raise ValueError("Frozen response protocol requires response_probe / pid_ff")
    config = {
        "schema_version": 1, "scenario": "x650_wholebody_mpc_v1",
        "stage": stage, "controller": controller, "model": MODEL,
        "trajectory": trajectory,
        "trajectory_parameters": None if trajectory == "hover" else {
            "amplitude_x": .075 if stage == "small" else .25,
            "amplitude_z": .03 if stage == "small" else .10,
            "period_s": period, "cycles": 2, "start_s": 2., "ramp_s": 2.},
        "duration_s": (20. if stage == "hover_folded" else 30.) if trajectory == "hover" else 8.+2.*period,
        "hover_height_m": 2., "control_period_s": .02,
        "spawn_pose_enu_xyz_rpy": [0., 0., .375, 0., 0., 0.],
        "nominal_arm_share": .2, "feedforward_enabled": True,
        "simulation_only": True,
        "preparation_report": str(ROOT/"generated/x650_2link_v2_20261021/preparation_report.json"),
        "airframe_source": str(ROOT/"generated/x650_2link_v2_airframe_20261021/4998_gz_x650_2link_v2"),
        "native_recorder_library": str(ROOT/"benchmark/whole_body_closed_loop_v1/libwholebody_recorder.so"),
        "response_model": None, "pid_profile": None, "stage_evidence": {}, "diagnostic_profile": None,
        "hover_reference_profile": None,
        "pid_tuning_candidate": None, "precision_test_profile": None,
        "research_profile": None,
    }
    # Keep archived/default task keysets unchanged. The optional explicit field
    # selects a separately hash-bound experiment rather than a new plant state.
    if protocol is not None:
        config.update(response_probe_protocol=protocol, duration_s=protocol["duration_s"])
        if protocol["role"] == "holdout":
            config.update(frozen_response_candidate=None, frozen_response_candidate_sha256=None)
    if mixed_protocol is not None:
        config.update(mixed_mpc_protocol=mixed_protocol, feedforward_enabled=False)
    return config


def validate_task_config(value):
    if not isinstance(value, dict):
        raise ValueError("Task configuration must be an object")
    value = dict(value)
    value.setdefault("diagnostic_profile", None)  # Previously archived canonical tasks remain valid.
    value.setdefault("hover_reference_profile", None)
    value.setdefault("pid_tuning_candidate", None)
    value.setdefault("precision_test_profile", None)
    value.setdefault("research_profile", None)
    protocol = validate_response_protocol(value.get("response_probe_protocol"))
    mixed_protocol = validate_mixed_mpc_protocol(value.get("mixed_mpc_protocol"))
    expected = build_task_config(value.get("stage"), value.get("controller"),
                                 response_probe_protocol=protocol, mixed_mpc_protocol=mixed_protocol)
    if "response_probe_protocol" in value and protocol is None:
        expected["response_probe_protocol"] = None
    if "mixed_mpc_protocol" in value and mixed_protocol is None:
        expected["mixed_mpc_protocol"] = None
    paths = {"preparation_report", "airframe_source", "native_recorder_library", "response_model", "pid_profile"}
    editable = paths | {"stage_evidence", "diagnostic_profile", "hover_reference_profile", "pid_tuning_candidate", "precision_test_profile", "research_profile"}
    if protocol is not None and protocol["role"] == "holdout":
        editable.update(("frozen_response_candidate", "frozen_response_candidate_sha256"))
    if set(value) != set(expected) or any(value[k] != expected[k] for k in expected if k not in editable):
        raise ValueError("Task presets changed: use a separately reviewed configuration version")
    for key in paths:
        if value[key] is None and key in ("response_model", "pid_profile"):
            continue
        if not isinstance(value[key], str) or not Path(value[key]).is_absolute():
            raise ValueError(key+" must be an absolute path")
    if not isinstance(value["stage_evidence"], dict):
        raise ValueError("stage_evidence must map gate names to evidence paths")
    value["diagnostic_profile"] = validate_profile(value["diagnostic_profile"], value["stage"], value["controller"])
    reference = value["hover_reference_profile"]
    if reference is not None:
        mixed = is_mixed_mpc_research(value)
        reference_stages = MIXED_MPC_STAGES if mixed else UPPER_RESEARCH_STAGES if is_upper_research(value) else ("hover_folded", "joint_probe")
        if (value["stage"] not in reference_stages or (value["controller"] != "pid_ff" and not mixed)
                or not isinstance(reference, dict) or set(reference) != {"name", "world_sdf"}
                or reference["name"] != "fixed_world_position_v1"
                or not isinstance(reference["world_sdf"], str) or not Path(reference["world_sdf"]).is_absolute()):
            raise ValueError("Fixed world hover requires the reviewed profile and an absolute world SDF")
    if value["diagnostic_profile"] == PRECISION_PROFILE and reference is None:
        raise ValueError("Independent EV precision diagnostic requires explicit fixed world reference")
    value["pid_tuning_candidate"] = validate_pid_candidate(value["pid_tuning_candidate"], value["stage"], value["controller"])
    if value["pid_tuning_candidate"] is not None and (reference is None or value["pid_profile"] is not None):
        raise ValueError("PID candidates require fixed-world control and cannot use a frozen formal PID profile")
    if value["precision_test_profile"] is not None:
        if (value["precision_test_profile"] != "hover20_steps36_v1" or value["stage"] != "hover_folded"
                or value["controller"] != "pid_ff" or reference is None):
            raise ValueError("Precision step protocol requires fixed-world hover_folded / pid_ff")
    value["research_profile"] = validate_research_profile(value["research_profile"], value)
    if protocol is not None and (value["research_profile"] or {}).get("name") != UPPER_RESEARCH_PROFILE_NAME:
        raise ValueError("Frozen response protocol requires precision_ev_upper_v2 estimated feedback")
    if (mixed_protocol is not None) != is_mixed_mpc_research(value):
        raise ValueError("Mixed MPC comparison requires both the v3 research profile and its frozen protocol")
    if is_mixed_mpc_research(value) and value["response_model"] is None:
        raise ValueError("Mixed MPC comparison requires the accepted response model for both controllers")
    if protocol is not None and protocol["role"] == "holdout":
        path, digest = value["frozen_response_candidate"], value["frozen_response_candidate_sha256"]
        if (path is None) != (digest is None):
            raise ValueError("Holdout frozen candidate path and SHA-256 must be supplied together")
        if path is not None and (not isinstance(path, str) or not Path(path).is_absolute()
                or not isinstance(digest, str) or len(digest) != 64 or set(digest)-set("0123456789abcdef")):
            raise ValueError("Holdout requires an absolute frozen candidate path and valid SHA-256")
    if is_upper_research(value) and value["stage"] == "tip_hold" and value["response_model"] is None:
        raise ValueError("Upper research tip_hold requires the identified response model")
    return dict(value)


def selected_task_config():
    path = os.environ.get(CONFIG_ENV)
    return validate_task_config(json.loads(Path(path).read_text(encoding="utf-8"))) if path else build_task_config()


def required_gates(cfg):
    stage = cfg["stage"]
    if is_mixed_mpc_research(cfg):
        gates = {"native_ground", "hover_folded", "joint_probe", "response_identification"}
        if cfg["controller"] == "mpc":
            gates.add("pid_baseline")
        if stage in ("small", "normal", "fast"):
            gates.add("tip_hold")
        if stage in ("normal", "fast"):
            gates.add("small")
        if stage == "fast":
            gates.add("normal")
        return sorted(gates)
    gates = set()
    if stage != "hover_folded":
        gates.update(("native_ground", "hover_folded"))
    if stage in ("tip_hold", "small", "normal", "fast"):
        gates.add("joint_probe")
    if is_upper_research(cfg):
        if stage == "response_probe":
            gates.add("joint_probe")
        elif stage == "tip_hold":
            gates.add("response_identification")
    if stage in ("normal", "fast") or (cfg["controller"] == "mpc" and stage in ("tip_hold", "small")):
        gates.update(("response_identification", "pid_baseline"))
    if stage in ("normal", "fast"):
        gates.add("small")
    if stage == "fast":
        gates.add("normal")
    return sorted(gates)


def validate_stage_gates(cfg, provenance):
    """A boolean cannot unlock a flight: bind evidence to model and raw files."""
    validated = {}
    condition = condition_for_task(cfg)
    if condition is not None and condition["model_sha256"] != provenance["model_sha256"]:
        raise ValueError("Research PID profile belongs to another physical model")
    for gate in required_gates(cfg):
        expected_condition, gate_role = (expected_gate_condition(condition, gate, cfg["stage"])
                                         if condition is not None else (None, None))
        value = cfg["stage_evidence"].get(gate, "")
        if not isinstance(value, str):
            raise ValueError("Native stage evidence must be a file path: "+gate)
        path = Path(value)
        if not path.is_file():
            raise ValueError("Missing native stage evidence: "+gate)
        data = json.loads(path.read_text(encoding="utf-8"))
        if (data.get("kind") != "x650_native_stage_evidence" or data.get("stage") != gate
                or data.get("diagnostic_only") is True
                or data.get("eligible_as_stage_gate") is False
                or data.get("complete") is not True or data.get("passed") is not True
                or data.get("simulation_backend") != "gazebo_px4"
                or data.get("model_sha256") != provenance["model_sha256"]
                or data.get("source_urdf_sha256") != provenance["source_urdf_sha256"]):
            raise ValueError("Invalid or mismatched native evidence: "+gate)
        if condition is None:
            if (data.get("ground_truth_aided") is True or data.get("research_profile") is not None
                    or data.get("operating_condition") is not None or data.get("operating_condition_sha256") is not None):
                raise ValueError("Research/truth-aided evidence cannot unlock a legacy GPS stage: "+gate)
        elif (data.get("diagnostic_only") is not False or data.get("ground_truth_aided") is not True
                or data.get("eligible_as_stage_gate") is not True
                or any(data.get(name) is not True for name in ("research_acceptance_verified",
                       "full_window_ulog_verified", "parameter_restore_verified"))
                or data.get("operating_condition") != expected_condition
                or canonical_hash(data.get("operating_condition")) != canonical_hash(expected_condition)
                or data.get("operating_condition_sha256") != canonical_hash(expected_condition)
                or data.get("pid_parameters_sha256") != expected_condition["pid_parameters_sha256"]):
            raise ValueError("Research evidence has a different or missing operating condition: "+gate)
        upper_certificate = (expected_condition is not None
                             and expected_condition["research_profile"] in (UPPER_RESEARCH_PROFILE_NAME, MIXED_MPC_PROFILE_NAME))
        if upper_certificate and any(data.get(name) is not True
                                     for name in ("upper_command_verified", "feedback_contract_verified")):
            raise ValueError("Upper research evidence lacks verified command/feedback contracts: "+gate)
        files = data.get("artifacts")
        if not isinstance(files, dict) or not files:
            raise ValueError("Native evidence must include hashed raw artifacts: "+gate)
        if is_mixed_mpc_research(cfg) and gate == "response_identification":
            accepted = provenance.get("accepted_response_binding")
            if accepted is None:
                accepted = load_accepted_response_model(cfg)["binding"]
            if accepted["evidence"] != {"path": str(path.resolve()), "sha256": sha(path)}:
                raise ValueError("Response gate differs from the accepted-model certificate binding")
        else:
            for name, digest in files.items():
                artifact = Path(name)
                if not artifact.is_absolute():
                    artifact = path.parent/artifact
                if not artifact.is_file() or sha(artifact) != digest:
                    raise ValueError("Native evidence artifact mismatch: "+str(artifact))
        mixed_certificate = expected_condition is not None and expected_condition["research_profile"] == MIXED_MPC_PROFILE_NAME
        if mixed_certificate:
            if (data.get("mixed_mpc_protocol") != cfg["mixed_mpc_protocol"]
                    or data.get("mixed_mpc_protocol_sha256") != mixed_protocol_hash(cfg["mixed_mpc_protocol"])
                    or data.get("controller") not in ("pid_ff", "mpc")
                    or data.get("acceleration_feedforward") != "zero"):
                raise ValueError("Mixed MPC gate lacks the matching frozen comparison protocol: "+gate)
            if gate == "pid_baseline" and (data.get("source_stage") != cfg["stage"] or data["controller"] != "pid_ff"):
                raise ValueError("PID baseline must match this exact trajectory stage")
            if gate in ("small", "normal") and data["controller"] != cfg["controller"]:
                raise ValueError("Trajectory progression requires the same controller: "+gate)
        if condition is not None:
            names = {Path(name).name for name in files}
            audit_name = "mixed_mpc_stage_audit.json" if mixed_certificate else "upper_stage_audit.json" if upper_certificate else "precision_stage_audit.json"
            if (not {"stage_result.json", "controller_metadata.json", audit_name} <= names
                    or not any(Path(name).suffix == ".ulg" for name in files)):
                raise ValueError("Research evidence requires hashed full ULog, original stage/metadata and independent audit: "+gate)
        validated[gate] = {"path": str(path.resolve()), "sha256": sha(path), "evidence": data}
        if is_upper_research(cfg):
            validated[gate].update(role=gate_role,
                source_operating_condition_sha256=canonical_hash(expected_condition),
                target_operating_condition_sha256=canonical_hash(condition))
    return validated


def validate_upper_response_model(cfg, provenance):
    """Bind selected v2 response bytes to the same-condition certified result."""
    if is_mixed_mpc_research(cfg):
        return load_accepted_response_model(cfg)["binding"]
    if not is_upper_research(cfg) or cfg["stage"] != "tip_hold":
        return None
    path = cfg.get("response_model")
    if not isinstance(path, str) or not Path(path).is_absolute() or not Path(path).is_file():
        raise ValueError("Upper research tip_hold requires an existing absolute response model")
    response_path = Path(path).resolve()
    response = json.loads(response_path.read_text(encoding="utf-8"))
    condition = condition_for_task(cfg)
    if (not isinstance(response, dict) or response.get("identified") is not True or response.get("passed") is not True
            or response.get("model_sha256") != condition["model_sha256"]
            or response.get("pid_parameters_sha256") != condition["pid_parameters_sha256"]
            or response.get("operating_condition") != condition
            or canonical_hash(response.get("operating_condition")) != canonical_hash(condition)
            or response.get("operating_condition_sha256") != canonical_hash(condition)):
        raise ValueError("Selected response model differs from the identified v2 operating condition")
    bound = provenance["stage_gates"].get("response_identification")
    if not isinstance(bound, dict) or bound.get("role") != "same_condition":
        raise ValueError("Upper response identification must be a same-condition gate")
    evidence_root = Path(bound["path"]).parent
    artifacts = bound["evidence"]["artifacts"]
    matched = any(Path(name).name == "response_model.json"
                  and (Path(name) if Path(name).is_absolute() else evidence_root/name).resolve() == response_path
                  and digest == sha(response_path) for name, digest in artifacts.items())
    if not matched:
        raise ValueError("Selected response model is not the response_model.json certified by response_identification")
    return {"path": str(response_path), "sha256": sha(response_path),
            "operating_condition_sha256": canonical_hash(condition), "gate_role": "same_condition"}


def validate_frozen_response_candidate(cfg):
    """A holdout cannot fly before training parameters have been frozen on disk."""
    from response_protocol import build_response_protocol
    protocol = validate_response_protocol(cfg.get("response_probe_protocol"))
    if protocol is None or protocol["role"] != "holdout":
        return None
    path = cfg.get("frozen_response_candidate")
    if not isinstance(path, str) or not Path(path).is_absolute() or not Path(path).is_file():
        raise ValueError("Holdout launch requires an existing frozen response candidate")
    path = Path(path).resolve()
    digest = sha(path)
    if digest != cfg.get("frozen_response_candidate_sha256"):
        raise ValueError("Holdout frozen response candidate SHA-256 changed")
    candidate = json.loads(path.read_text(encoding="utf-8"))
    if (candidate.get("kind") != "x650_frozen_response_candidate"
            or candidate.get("schema_version") != 1
            or candidate.get("training_passed") is not True
            or candidate.get("identified") is not False
            or candidate.get("eligible_as_stage_gate") is not False
            or candidate.get("protocols") != {role: build_response_protocol(role) for role in ("train", "holdout")}):
        raise ValueError("Holdout candidate is not the reviewed training-only parameter freeze")
    created = candidate.get("created_unix")
    if (isinstance(created, bool) or not isinstance(created, (int, float)) or not math.isfinite(created)
            or not 0 < created <= time.time()):
        raise ValueError("Holdout candidate creation time is invalid")
    parameters = candidate.get("parameters")
    if not isinstance(parameters, dict) or set(parameters) != {"x", "y", "z"}:
        raise ValueError("Holdout candidate requires exactly the mixed-order axis parameters")
    for axis in "xyz":
        expected = {"tau_s"} if axis == "z" else {"wn_rad_s", "zeta"}
        if not isinstance(parameters[axis], dict) or set(parameters[axis]) != expected:
            raise ValueError("Holdout candidate has unreviewed parameter names")
        for name, number in parameters[axis].items():
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
                raise ValueError("Holdout candidate parameters must be positive finite numbers")
            fit = protocol["parameter_selection"]
            bounds = (fit["z"]["tau_bounds_s"] if name == "tau_s" else
                      fit["xy"]["wn_bounds_rad_s"] if name == "wn_rad_s" else fit["xy"]["zeta_bounds"])
            if not bounds[0] < number < bounds[1]:
                raise ValueError("Holdout candidate parameter is outside the frozen fit bounds")
    identity = candidate.get("identity")
    condition = condition_for_task(cfg)
    if (not isinstance(identity, dict) or identity.get("operating_condition") != condition
            or identity.get("operating_condition_sha256") != canonical_hash(condition)
            or identity.get("pid_parameters_sha256") != condition["pid_parameters_sha256"]):
        raise ValueError("Holdout candidate belongs to another operating condition")
    return {"path": str(path), "sha256": digest, "created_unix": created,
            "role": "training_only_parameter_freeze", "eligible_as_stage_gate": False,
            "identity": identity}


def preflight(cfg, *, check_airframe=True):
    from amsim.modeling.manager import ensure_flight_ready, resolve, validate_px4_airframe
    from x650_wholebody.model import ArmModel
    cfg = validate_task_config(cfg)
    candidate_binding = validate_frozen_response_candidate(cfg)
    assert_no_pending_journal()
    from amsim.scenario_registry import get
    descriptor = get(cfg["scenario"])
    if descriptor.source != "user" or descriptor.module != "simulations/x650_wholebody_mpc_v1/run_case.py":
        raise ValueError("This scenario requires platform-managed user run directory and recording")
    report_path = Path(cfg["preparation_report"])
    arm = ArmModel.from_report(report_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    model = resolve(cfg["model"])
    ensure_flight_ready(model)
    if model.source != "urdf_flight" or model.joints != ["arm_joint_1", "arm_joint_2"] or model.px4_sys_autostart != 4998:
        raise ValueError("Require explicit X650 two-link flight model and airframe 4998")
    manifest = json.loads(model.manifest_path.read_text(encoding="utf-8"))
    source = json.loads((model.sdf_path.parent/"source_import_manifest.json").read_text(encoding="utf-8"))
    source_sha = source["source"]["urdf_sha256"]
    if source_sha != report["derived_urdf_sha256"] or not math.isclose(model.total_mass_kg, arm.total_mass, abs_tol=1e-9):
        raise ValueError("Physics model differs from controller preparation")
    if manifest["profile_sha256"] != report["flight_profile_sha256"]:
        raise ValueError("Physics flight profile differs from preparation")
    airframe = Path(cfg["airframe_source"])
    derivation = json.loads((airframe.parent/"airframe_derivation.json").read_text(encoding="utf-8"))
    if (derivation["airframe_sha256"] != sha(airframe)
            or derivation["composition_report_sha256"] != sha(report_path)
            or derivation["model"] != model.model_name):
        raise ValueError("Airframe is not bound to this composition")
    if check_airframe and sha(validate_px4_airframe(model)) != sha(airframe):
        raise ValueError("Installed PX4 airframe differs from the reviewed input")
    library = Path(cfg["native_recorder_library"])
    if not library.is_file():
        raise ValueError("Independent native physics recorder missing: "+str(library))
    provenance = {
        "model_name": model.model_name, "model_sha256": model.sha256,
        "source_urdf_sha256": source_sha, "total_mass_kg": arm.total_mass,
        "preparation_report_sha256": sha(report_path), "airframe_sha256": sha(airframe),
        "flight_profile_sha256": manifest["profile_sha256"], "px4_sys_autostart": 4998,
        "native_recorder_sha256": sha(library), "stage": cfg["stage"],
        "controller": cfg["controller"], "full_body_torque_nmpc": False,
        "diagnostic_only": any(cfg[k] is not None for k in ("diagnostic_profile", "pid_tuning_candidate", "precision_test_profile")),
        "diagnostic_profile": cfg["diagnostic_profile"],
        "hover_reference_profile": cfg["hover_reference_profile"],
        "pid_tuning_candidate": cfg["pid_tuning_candidate"], "precision_test_profile": cfg["precision_test_profile"],
        "research_profile": cfg["research_profile"],
    }
    condition = condition_for_task(cfg)
    if condition is not None:
        provenance.update(operating_condition=condition, operating_condition_sha256=canonical_hash(condition),
                          pid_parameters_sha256=condition["pid_parameters_sha256"])
    provenance.update(protocol_annotation(cfg))
    provenance.update(mixed_protocol_annotation(cfg))
    if candidate_binding is not None:
        if any(candidate_binding["identity"].get(key) != provenance[key] for key in
               ("model_sha256", "source_urdf_sha256", "airframe_sha256",
                "flight_profile_sha256", "preparation_report_sha256")):
            raise ValueError("Holdout candidate belongs to another prepared physical model")
        provenance["frozen_response_candidate_binding"] = candidate_binding
    if cfg["hover_reference_profile"] is not None:
        world_path = Path(cfg["hover_reference_profile"]["world_sdf"])
        if not world_path.is_file():
            raise ValueError("Fixed hover world datum file is missing")
        provenance["hover_world_sdf_sha256"] = sha(world_path)
    sources = source_files_for_task(cfg)
    provenance["source_files_sha256"] = {p.relative_to(ROOT).as_posix(): sha(p) for p in sources}
    accepted_response = None
    if is_mixed_mpc_research(cfg):
        accepted_response = load_accepted_response_model(cfg)
        if any(accepted_response["model"].get(key) != provenance[key] for key in
               ("model_sha256", "source_urdf_sha256", "airframe_sha256", "flight_profile_sha256", "preparation_report_sha256")):
            raise ValueError("Accepted response belongs to another prepared physical model")
        provenance["accepted_response_binding"] = accepted_response["binding"]
    provenance["stage_gates"] = validate_stage_gates(cfg, provenance)
    if is_upper_research(cfg) and not is_mixed_mpc_research(cfg) and cfg["stage"] == "tip_hold":
        provenance["response_model_binding"] = validate_upper_response_model(cfg, provenance)
    for key in ("pid_profile", "response_model"):
        path = cfg[key]
        if path is not None:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if data.get("model_sha256") != model.sha256:
                raise ValueError(key+" belongs to another plant")
            provenance[key] = {"path": path, "sha256": sha(path)}
    if cfg["controller"] == "mpc" and cfg["stage"] in ("tip_hold", "small", "normal", "fast"):
        if not cfg["response_model"] or not cfg["pid_profile"]:
            raise ValueError("MPC flight requires identified response and frozen PID evidence")
        # Import the actual numerical backend before arming, so ABI failures
        # cannot first surface during the airborne offboard transition.
        from x650_wholebody.mpc import MpcController
        response = json.loads(Path(cfg["response_model"]).read_text(encoding="utf-8"))
        profile = json.loads(Path(cfg["pid_profile"]).read_text(encoding="utf-8"))
        if response.get("identified") is not True or response.get("passed") is not True:
            raise ValueError("Response model has not passed independent native validation")
        bound = provenance["stage_gates"]["response_identification"]
        response_path = Path(cfg["response_model"]).resolve()
        artifacts = bound["evidence"]["artifacts"]
        evidence_root = Path(bound["path"]).parent
        matched = any((Path(name) if Path(name).is_absolute() else evidence_root/name).resolve() == response_path
                      and digest == sha(response_path) for name, digest in artifacts.items())
        if not matched:
            raise ValueError("Selected response model is not the file validated by response_identification")
        params = {k: float(v) for k, v in profile.get("parameters", profile.get("pid_gains", {})).items()
                  if k.startswith(("MC_", "MPC_"))}
        pid_digest = hashlib.sha256(json.dumps(params, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        if response.get("pid_parameters_sha256") != pid_digest:
            raise ValueError("Identified response belongs to different PID parameters")
        selected_config = (controller_config(cfg["mixed_mpc_protocol"], accepted_response["model"])
                           if is_mixed_mpc_research(cfg) else
                           {"base_time_constant_s": response["base_time_constant_s"],
                            "response_identified": True, "failure_policy": "raise"})
        MpcController(arm, profile.get("parameters", profile.get("pid_gains", {})), config=selected_config)
    return model, provenance
