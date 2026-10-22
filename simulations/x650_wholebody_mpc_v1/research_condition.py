"""Explicit operating conditions for new truth-aided commissioning runs.

This is a new research protocol, not a relabeling of archived diagnostics. Its
first interface version permits only native fixed-position hover/joint probes.
Upper-controller response/endpoint commissioning uses a separately identified
v2 condition. The v1 condition bytes remain unchanged for archived certificates.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from diagnostic_parameters import PRECISION_PROFILE, PID_CANDIDATES, load_parameter_evidence
from hover_reference import PINNED_PX4


RESEARCH_PROFILE_NAME = "precision_ev_selected_pid_v1"
RESEARCH_STAGES = ("hover_folded", "joint_probe")
RESEARCH_PID_CANDIDATE = "velocity_high"
UPPER_RESEARCH_PROFILE_NAME = "precision_ev_upper_v2"
UPPER_RESEARCH_STAGES = ("response_probe", "tip_hold")
MIXED_MPC_PROFILE_NAME = "precision_ev_mixed_mpc_v3"
MIXED_MPC_STAGES = ("tip_hold", "small", "normal", "fast")
PREPARATION_GATES = ("native_ground", "hover_folded", "joint_probe")


def is_upper_research(cfg):
    value = cfg.get("research_profile")
    return isinstance(value, dict) and value.get("name") in (UPPER_RESEARCH_PROFILE_NAME, MIXED_MPC_PROFILE_NAME)


def is_mixed_mpc_research(cfg):
    value = cfg.get("research_profile")
    return isinstance(value, dict) and value.get("name") == MIXED_MPC_PROFILE_NAME


def _protocol_fields(name):
    if name == RESEARCH_PROFILE_NAME:
        return {"schema_version": 1, "research_profile": RESEARCH_PROFILE_NAME,
                "scope": "simulation_truth_aided_ev_commissioning",
                "control_interface": "fixed_world_position_v1"}
    if name == UPPER_RESEARCH_PROFILE_NAME:
        return {"schema_version": 2, "research_profile": UPPER_RESEARCH_PROFILE_NAME,
                "scope": "simulation_truth_aided_ev_upper_control",
                "control_interface": "raw_ev_world_pva_v2",
                "feedback_contract": "px4_estimator_receipt_enu_v1"}
    if name == MIXED_MPC_PROFILE_NAME:
        return {"schema_version": 3, "research_profile": MIXED_MPC_PROFILE_NAME,
                "scope": "simulation_truth_aided_ev_mixed_mpc",
                "control_interface": "raw_ev_world_pva_v2",
                "feedback_contract": "px4_estimator_acceleration_receipt_enu_v2"}
    raise ValueError("Unknown research operating-condition version")


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _valid_hash(value):
    return isinstance(value, str) and len(value) == 64 and not set(value)-set("0123456789abcdef")


def frozen_research_pid(cfg):
    """Read the full predeclared controller state; this never writes parameters."""
    path = cfg.get("pid_profile")
    if not isinstance(path, str) or not Path(path).is_absolute() or not Path(path).is_file():
        raise ValueError("Research precision mode requires an existing absolute frozen PID profile")
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not _valid_hash(document.get("model_sha256")):
        raise ValueError("Frozen research PID profile requires a physical model SHA-256")
    values = document.get("parameters")
    required = set(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
    if not isinstance(values, dict) or set(values) != required:
        raise ValueError("Research PID profile requires exactly all 42 canonical MC/MPC parameters")
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
           for value in values.values()):
        raise ValueError("Frozen research PID parameters must be finite numeric readbacks")
    parameters = {name: float(value) for name, value in values.items()}
    for name, expected in PID_CANDIDATES[RESEARCH_PID_CANDIDATE].items():
        if not math.isclose(parameters[name], expected, rel_tol=2e-6, abs_tol=1e-7):
            raise ValueError("Research PID profile differs from selected four-gain catalog: "+name)
    digest = canonical_hash(parameters)
    if document.get("pid_parameters_sha256", digest) != digest:
        raise ValueError("Frozen research PID parameter digest disagrees with its values")
    return document, parameters, digest


def validate_research_profile(value, cfg):
    if value is None:
        return None
    if (not isinstance(value, dict) or set(value) != {"name", "world_sdf"}
            or value.get("name") not in (RESEARCH_PROFILE_NAME, UPPER_RESEARCH_PROFILE_NAME, MIXED_MPC_PROFILE_NAME)
            or not isinstance(value.get("world_sdf"), str) or not Path(value["world_sdf"]).is_absolute()):
        raise ValueError("Research profile requires the reviewed name and an absolute world SDF")
    mixed = value["name"] == MIXED_MPC_PROFILE_NAME
    stages = MIXED_MPC_STAGES if mixed else UPPER_RESEARCH_STAGES if value["name"] == UPPER_RESEARCH_PROFILE_NAME else RESEARCH_STAGES
    if cfg.get("stage") not in stages or cfg.get("controller") not in (("pid_ff", "mpc") if mixed else ("pid_ff",)):
        raise ValueError("Research profile permits only its reviewed commissioning stages with pid_ff; eight-figure/MPC stages are not enabled")
    if any(cfg.get(name) is not None for name in
           ("diagnostic_profile", "pid_tuning_candidate", "precision_test_profile")):
        raise ValueError("Research profile cannot be combined with diagnostic or tuning profiles")
    reference = cfg.get("hover_reference_profile")
    if (not isinstance(reference, dict) or set(reference) != {"name", "world_sdf"}
            or reference.get("name") != "fixed_world_position_v1"
            or not isinstance(reference.get("world_sdf"), str)
            or Path(reference["world_sdf"]).resolve() != Path(value["world_sdf"]).resolve()):
        raise ValueError("Research precision requires the same explicit fixed-world reference and world SDF")
    frozen_research_pid(cfg)
    return dict(value)


def condition_for_task(cfg):
    """Return a relocation-independent identity for the reviewed operating state."""
    profile = validate_research_profile(cfg.get("research_profile"), cfg)
    if profile is None:
        return None
    world = Path(profile["world_sdf"])
    if not world.is_file():
        raise ValueError("Research precision world SDF is missing")
    document, _, digest = frozen_research_pid(cfg)
    return validate_operating_condition({
        "kind": "x650_research_operating_condition", **_protocol_fields(profile["name"]),
        "model_sha256": document["model_sha256"],
        "estimator_parameters": dict(PRECISION_PROFILE),
        "observer": {"rate_hz": 50., "gaussian_noise": 0., "measurement_point": "base_link"},
        "coordinate_contract": "raw_ev_world",
        "px4_source_contract_commit": PINNED_PX4,
        "pid_parameters_sha256": digest,
        "world_sdf_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
        "ground_truth_aided": True,
        "gps_performance_claim": False, "hardware_performance_claim": False,
    })


def validate_operating_condition(value):
    """Reject free-form estimator/control conditions at runtime boundaries."""
    if not isinstance(value, dict):
        raise ValueError("Research operating condition must be an explicit object")
    for name in ("model_sha256", "pid_parameters_sha256", "world_sdf_sha256"):
        if not _valid_hash(value.get(name)):
            raise ValueError("Research operating condition requires a SHA-256: "+name)
    expected = {
        "kind": "x650_research_operating_condition", **_protocol_fields(value.get("research_profile")),
        "model_sha256": value["model_sha256"],
        "estimator_parameters": dict(PRECISION_PROFILE),
        "observer": {"rate_hz": 50., "gaussian_noise": 0., "measurement_point": "base_link"},
        "coordinate_contract": "raw_ev_world",
        "px4_source_contract_commit": PINNED_PX4,
        "pid_parameters_sha256": value["pid_parameters_sha256"],
        "world_sdf_sha256": value["world_sdf_sha256"],
        "ground_truth_aided": True,
        "gps_performance_claim": False, "hardware_performance_claim": False,
    }
    # Canonical bytes also distinguish true/1, false/0, and undeclared fields.
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError("Research operating condition differs from the reviewed protocol")
    return deepcopy(expected)


def expected_gate_condition(condition, gate, stage):
    """Explicit one-way use of v1 physical preparation for v2 upper control.

    Shared model, EV, PID, world and coordinate fields are copied unchanged.
    This mapping cannot upgrade v1 response evidence or unlock an unreviewed
    stage. A preparation certificate remains a v1 certificate in provenance.
    """
    condition = validate_operating_condition(condition)
    if condition["research_profile"] == MIXED_MPC_PROFILE_NAME:
        if stage not in MIXED_MPC_STAGES:
            raise ValueError("Mixed MPC gates are limited to reviewed comparison stages")
        if gate in PREPARATION_GATES or gate == "response_identification":
            source_name = RESEARCH_PROFILE_NAME if gate in PREPARATION_GATES else UPPER_RESEARCH_PROFILE_NAME
            mapped = {key: value for key, value in condition.items() if key not in _protocol_fields(MIXED_MPC_PROFILE_NAME)}
            mapped.update(_protocol_fields(source_name))
            return validate_operating_condition(mapped), "preparation_only" if gate in PREPARATION_GATES else "nominal_response_only"
        if gate in ("pid_baseline", "tip_hold", "small", "normal"):
            return condition, "same_condition"
        raise ValueError("No reviewed mixed MPC gate mapping for "+gate)
    if condition["research_profile"] != UPPER_RESEARCH_PROFILE_NAME:
        return condition, "same_condition"
    if stage not in UPPER_RESEARCH_STAGES:
        raise ValueError("Upper research gates are limited to reviewed response/tip stages")
    if gate in PREPARATION_GATES:
        preparation = {key: value for key, value in condition.items()
                       if key not in _protocol_fields(UPPER_RESEARCH_PROFILE_NAME)}
        preparation.update(_protocol_fields(RESEARCH_PROFILE_NAME))
        return validate_operating_condition(preparation), "preparation_only"
    if stage == "tip_hold" and gate == "response_identification":
        return condition, "same_condition"
    raise ValueError("No reviewed upper research gate mapping for "+gate)


def research_runtime_parameters(cfg):
    profile = validate_research_profile(cfg.get("research_profile"), cfg)
    if profile is None:
        return None, None
    return deepcopy(PRECISION_PROFILE), deepcopy(PID_CANDIDATES[RESEARCH_PID_CANDIDATE])
