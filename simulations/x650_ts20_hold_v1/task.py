"""Separate conditional TS20 plant admission; archived model gates never transfer."""
from __future__ import annotations
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
LEGACY = ROOT / "simulations/x650_wholebody_mpc_v1"
for directory in (ROOT, ROOT / "src", LEGACY, ROOT / "scripts", ROOT / "control_packages/x650_wholebody_v1"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
from simulations.x650_wholebody_mpc_v1 import task as original
from research_condition import canonical_hash, condition_for_task, frozen_research_pid
from mixed_mpc_protocol import build_prediction_chain_protocol
from x650_wholebody.ts20 import FrozenTs20Model

CONFIG_ENV = "X650_TS20_TASK_CONFIG"
MODEL = "amsim_urdf_x650_2link_ts20_v1_flight"
STAGES = ("hover_folded", "joint_probe", "tip_hold")
AIRFRAME = 4997
DAMPED_SCENARIO = "x650_ts20_damped_hold_v1"
DAMPED_MODEL = "amsim_urdf_x650_2link_ts20_damped_v1_flight"
DAMPED_AIRFRAME = 4996


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def hold_protocol():
    return {
        "kind": "x650_ts20_conditional_hold_protocol_v1", "simulation_only": True,
        "stages": list(STAGES), "control_period_s": .02, "max_control_interval_s": .05,
        "max_compute_s": .05, "tip_hold_duration_s": 30.,
        "tip_rms_max_m": .05, "tip_peak_max_m": .10, "tilt_max_deg": 8.,
        "q_lower_rad": [-.75, -.35], "q_upper_rad": [.45, .85],
        "joint_velocity_max_rad_s": [.45, .35],
        "internal_physical_q_tolerance_rad": .0001,
        "new_joint_prediction_q_peak_max_rad": .003,
        "new_joint_prediction_dq_peak_max_rad_s": .05,
        "native_tip_required": True, "whole_declared_window_required": True,
        "motion_capture_mode": "simulation ideal EV; PX4 estimated base feedback",
        "required_new_plant_gates": ["native_ground", "hover_folded", "joint_probe"],
        "base_response_status": "prior_from_old_ideal_joint_plant; not newly identified",
        "joint_prediction_validation": "new complete probe; actual consumed MIT packets and first physical state only",
        "qualification": {"hardware_motor": "TS20-50", "hardware_supply_v": 24,
                          "identified_supply_v": 20, "supply_24v_qualified": False,
                          "actual_aerial_arm_load_qualified": False,
                          "physical_torque_scale_qualified": False},
        "disturbance": {"kind": "external_world_force", "link": "base_link",
                        "start_s": 8., "duration_s": .3, "force_n": [1., 0., 0.]},
        "trajectory_stages_enabled": False,
    }


def build_task_config(stage="hover_folded", controller=None, *, disturbed=False):
    if stage not in STAGES:
        raise ValueError("Conditional TS20 commissioning only permits hover_folded/joint_probe/tip_hold")
    controller = controller or ("mpc" if stage == "tip_hold" else "pid_ff")
    if controller != ("mpc" if stage == "tip_hold" else "pid_ff") or (disturbed and stage != "tip_hold"):
        raise ValueError("TS20 probes use pid_ff; tip_hold uses MPC; disturbance only in tip_hold")
    cfg = original.build_task_config(stage, controller)
    cfg.update(scenario="x650_ts20_hold_v1", model=MODEL, feedforward_enabled=False,
        preparation_report=str(ROOT / "generated/x650_2link_ts20_v1_20261022/preparation_report.json"),
        airframe_source=str(ROOT / "generated/x650_2link_ts20_v1_airframe_20261022/4997_gz_x650_2link_ts20_v1"),
        pid_profile=str(ROOT / "generated/x650_2link_ts20_v1_20261022/pid_profile.json"),
        ts20_protocol=hold_protocol(),
        ts20_inputs={"source_dir": str(ROOT / "artifacts/ts20_integration_20261022/upstream_snapshot"),
            "source_manifest": str(ROOT / "artifacts/ts20_integration_20261022/upstream_snapshot/source_manifest.json"),
            "plant_manifest": str(ROOT / "generated/imported_models" / MODEL / "plant_manifest.json"),
            "oracle_validation": None, "base_response_prior": None,
            "native_library": None, "native_library_sha256": None},
        disturbance_enabled=bool(disturbed), mpc_config={
            **deepcopy(build_prediction_chain_protocol()["common_controller_config"]),
            **deepcopy(build_prediction_chain_protocol()["mpc_controller_config"])})
    # These existing names select only the unchanged estimator/PID plumbing.
    # They are not the TS20 plant identity and cannot issue TS20 certificates.
    cfg["research_profile"] = {"name": "precision_ev_mixed_mpc_v3" if stage == "tip_hold" else "precision_ev_selected_pid_v1",
                               "world_sdf": str(ROOT / "generated/x650_2link_ts20_v1_20261022/world.sdf")}
    cfg["hover_reference_profile"] = {"name": "fixed_world_position_v1", "world_sdf": cfg["research_profile"]["world_sdf"]}
    return cfg


def damped_hold_protocol():
    value = deepcopy(hold_protocol())
    value.update(kind="x650_ts20_damped_conditional_hold_protocol_v1",
                 mit_gain_profile={"name": "mit_kp6_kd025_v1", "kp": 6., "kd": .25,
                                   "bus_period_s": .005, "gain_semantics": "candidate effective normalized MIT gains; not hardware readback"},
                 joint_probe_velocity_max_rad_s=.2,
                 gain_selection="b+Kd exceeds 1.4 times maximum negative Stribeck friction slope; fixed before new flights")
    return value


def build_damped_task_config(stage="hover_folded", controller=None, *, disturbed=False):
    cfg = build_task_config(stage, controller, disturbed=disturbed)
    cfg.update(scenario=DAMPED_SCENARIO, model=DAMPED_MODEL, ts20_protocol=damped_hold_protocol(),
        preparation_report=str(ROOT / "generated/x650_2link_ts20_damped_v1_20261022/preparation_report.json"),
        airframe_source=str(ROOT / "generated/x650_2link_ts20_damped_v1_airframe_20261022/4996_gz_x650_2link_ts20_damped_v1"),
        pid_profile=str(ROOT / "generated/x650_2link_ts20_damped_v1_20261022/pid_profile.json"))
    cfg["ts20_inputs"]["plant_manifest"] = str(ROOT / "generated/imported_models" / DAMPED_MODEL / "plant_manifest.json")
    world = str(ROOT / "generated/x650_2link_ts20_damped_v1_20261022/world.sdf")
    cfg["research_profile"]["world_sdf"] = world
    cfg["hover_reference_profile"]["world_sdf"] = world
    cfg["mpc_config"].pop("prediction_chain", None)
    return cfg


def variant(cfg):
    if cfg.get("scenario") == "x650_ts20_hold_v1":
        return {"model": MODEL, "airframe": AIRFRAME, "kp": 6., "kd": 3276 / 32768, "bus_period_s": .005,
                "oracle_protocol": "historical_kp6_kd00999755859375_v1"}
    if cfg.get("scenario") == DAMPED_SCENARIO:
        return {"model": DAMPED_MODEL, "airframe": DAMPED_AIRFRAME, "kp": 6., "kd": .25, "bus_period_s": .005,
                "oracle_protocol": "mit_kp6_kd025_v1"}
    raise ValueError("Explicit historical or damped TS20 scenario required")


def predictor_from_bound_plant(cfg, motion_model):
    """Both runtime and audit use explicit hash-bound gains and bus period."""
    from x650_wholebody.ts20_prediction import Ts20PredictionModel
    actual = read(cfg["ts20_inputs"]["plant_manifest"])["parameters"]
    selected = variant(cfg)
    for key in ("kp", "kd", "bus_period_s"):
        if actual[key] != selected[key]:
            raise ValueError("Plant MIT gain/bus differs from selected variant: " + key)
    model = Ts20PredictionModel(motion_model, kp=actual["kp"], kd=actual["kd"], bus_period_s=actual["bus_period_s"])
    model.enable_native(cfg["ts20_inputs"]["native_library"], expected_sha256=cfg["ts20_inputs"]["native_library_sha256"])
    return model


def validate_task_config(value):
    if not isinstance(value, dict):
        raise ValueError("TS20 task object required")
    cfg = deepcopy(value)
    if type(cfg.get("disturbance_enabled")) is not bool:
        raise ValueError("disturbance_enabled must be a boolean")
    selected = variant(cfg)
    builder = build_damped_task_config if cfg["scenario"] == DAMPED_SCENARIO else build_task_config
    expected = builder(cfg.get("stage"), cfg.get("controller"), disturbed=cfg.get("disturbance_enabled", False))
    paths = {"preparation_report", "airframe_source", "native_recorder_library", "pid_profile"}
    editable = paths | {"stage_evidence", "research_profile", "hover_reference_profile", "ts20_inputs"}
    if (set(cfg) != set(expected) or canonical_hash({k: cfg[k] for k in expected if k not in editable})
            != canonical_hash({k: expected[k] for k in expected if k not in editable})):
        raise ValueError("TS20 frozen task/limits changed; a separately reviewed protocol is required")
    for key in paths:
        if not isinstance(cfg[key], str) or not Path(cfg[key]).is_absolute():
            raise ValueError("Absolute input path required: " + key)
    inputs = cfg["ts20_inputs"]
    if not isinstance(inputs, dict) or set(inputs) != set(expected["ts20_inputs"]):
        raise ValueError("Explicit TS20 source, plant, oracle and base-prior paths required")
    for key, path in inputs.items():
        if key == "native_library_sha256":
            if not isinstance(path, str) or len(path) != 64 or set(path) - set("0123456789abcdef"):
                raise ValueError("Explicit native predictor library SHA-256 required")
            continue
        if path is None and key == "base_response_prior" and cfg["stage"] != "tip_hold":
            continue
        if not isinstance(path, str) or not Path(path).is_absolute():
            raise ValueError("Absolute TS20 input required: " + key)
    world = cfg["research_profile"].get("world_sdf") if isinstance(cfg["research_profile"], dict) else None
    if (cfg["research_profile"].get("name") != expected["research_profile"]["name"]
            or not isinstance(world, str) or not Path(world).is_absolute()
            or cfg["hover_reference_profile"] != {"name": "fixed_world_position_v1", "world_sdf": world}):
        raise ValueError("Explicit same-world raw-EV research configuration required")
    if not isinstance(cfg["stage_evidence"], dict):
        raise ValueError("stage_evidence must map new TS20 stage certificates to paths")
    return cfg


def selected_task_config():
    path = os.environ.get(CONFIG_ENV)
    return validate_task_config(read(path)) if path else validate_task_config(build_task_config())


def required_gates(cfg):
    return [] if cfg["stage"] == "hover_folded" else ["hover_folded"] if cfg["stage"] == "joint_probe" else ["hover_folded", "joint_probe"]


def condition(cfg, *, model_sha256, source_urdf_sha256):
    inputs = cfg["ts20_inputs"]
    return {"kind": "x650_ts20_conditional_operating_condition_v1",
            "model_sha256": model_sha256, "source_urdf_sha256": source_urdf_sha256,
            "pid_parameters_sha256": frozen_research_pid(cfg)[2],
            "world_sha256": sha(cfg["research_profile"]["world_sdf"]),
            "source_manifest_sha256": sha(inputs["source_manifest"]),
            "plant_manifest_sha256": sha(inputs["plant_manifest"]),
            "oracle_validation_sha256": sha(inputs["oracle_validation"]),
            "native_predictor_library_sha256": inputs["native_library_sha256"],
            "protocol_sha256": canonical_hash(cfg["ts20_protocol"]),
            "qualification": deepcopy(cfg["ts20_protocol"]["qualification"])}


def validate_stage_gates(cfg, provenance):
    identity = condition(cfg, model_sha256=provenance["model_sha256"], source_urdf_sha256=provenance["source_urdf_sha256"])
    result = {}
    for gate in required_gates(cfg):
        path = cfg["stage_evidence"].get(gate)
        if not isinstance(path, str) or not Path(path).is_file():
            raise ValueError("Missing new TS20 plant certificate: " + gate)
        doc = read(path)
        if (doc.get("kind") != "x650_ts20_conditional_stage_certificate_v1" or doc.get("stage") != gate
                or doc.get("ts20_operating_condition") != identity
                or any(doc.get(key) is not True for key in ("complete", "passed", "full_window_ulog_verified",
                       "parameter_restore_verified", "native_ground_verified", "actuator_log_verified"))):
            raise ValueError("Old-plant, incomplete or mismatched TS20 certificate: " + gate)
        if gate == "joint_probe" and doc.get("new_joint_prediction_verified") is not True:
            raise ValueError("New TS20 probe lacks independent complete joint prediction")
        files = doc.get("artifacts")
        if not isinstance(files, dict) or not files:
            raise ValueError("TS20 certificate requires complete hashed original artifacts")
        for name, expected in files.items():
            artifact = Path(name) if Path(name).is_absolute() else Path(path).parent / name
            if not artifact.is_file() or sha(artifact) != expected:
                raise ValueError("TS20 certificate artifact changed: " + name)
        result[gate] = {"path": str(Path(path).resolve()), "sha256": sha(path), "evidence": doc}
    return result


def source_files_for_task(cfg):
    sources = list(dict.fromkeys([*original.source_files_for_task(cfg),
        *sorted(Path(__file__).parent.glob("*.py")),
        ROOT / "scripts/prepare_x650_ts20_plant.py",
        ROOT / "scripts/ts20_interface_system.cpp",
        ROOT / "benchmark/whole_body_closed_loop_v1/wholebody_recorder.cpp",
        ROOT / "benchmark/whole_body_closed_loop_v1/build_recorder.py",
        ROOT / "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp",
        ROOT / "scripts/validate_x650_ts20_prediction.py"]))
    if cfg["scenario"] == DAMPED_SCENARIO:
        sources.append(ROOT / "configs/actuators/ts20_50_24v_damped_v1_candidate.json")
    return sources


def preflight(cfg, *, check_airframe=True):
    from amsim.modeling.manager import ensure_flight_ready, resolve, validate_px4_airframe
    from x650_wholebody.model import ArmModel
    from diagnostic_parameters import assert_no_pending_journal
    cfg = validate_task_config(cfg)
    assert_no_pending_journal()
    from amsim.scenario_registry import get
    descriptor = get(cfg["scenario"])
    if descriptor.source != "user" or descriptor.module != "simulations/x650_ts20_hold_v1/run_case.py":
        raise ValueError("New TS20 platform-managed scenario registration required")
    model = resolve(cfg["model"])
    ensure_flight_ready(model)
    selected = variant(cfg)
    if (model.model_name != selected["model"] or model.source != "urdf_flight"
            or model.px4_sys_autostart != selected["airframe"] or model.joints != ["arm_joint_1", "arm_joint_2"]):
        raise ValueError("New conditional TS20 imported model and dedicated airframe required")
    report = read(cfg["preparation_report"])
    arm = ArmModel.from_report(cfg["preparation_report"])
    manifest = read(model.manifest_path)
    source = read(model.sdf_path.parent / "source_import_manifest.json")
    source_sha = source["source"]["urdf_sha256"]
    if (source_sha != report["derived_urdf_sha256"] or manifest["profile_sha256"] != report["flight_profile_sha256"]
            or not math.isclose(arm.total_mass, model.total_mass_kg, abs_tol=1e-9)):
        raise ValueError("New TS20 physics and controller geometry disagree")
    derivation = read(Path(cfg["airframe_source"]).parent / "installation_manifest.json")
    if (derivation["airframe_sha256"] != sha(cfg["airframe_source"])
            or derivation["preparation_report_sha256"] != sha(cfg["preparation_report"])
            or derivation["model"] != model.model_name
            or (check_airframe and sha(validate_px4_airframe(model)) != sha(cfg["airframe_source"]))):
        raise ValueError("Dedicated TS20 airframe binding/installation mismatch")
    inputs = cfg["ts20_inputs"]
    motor = FrozenTs20Model(inputs["source_dir"], read(inputs["source_manifest"]))
    oracle = read(inputs["oracle_validation"])
    from validate_x650_ts20_prediction import protocol as oracle_protocol, digest
    if cfg["scenario"] == DAMPED_SCENARIO:
        from validate_x650_ts20_prediction import protocol_for_gain
        oracle_protocol = lambda: protocol_for_gain("mit_kp6_kd025_v1")
    core = ROOT / "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction.py"
    native = oracle.get("native_backend", {})
    if (oracle.get("complete") is not True or oracle.get("pass") is not True
            or oracle.get("protocol_sha256") != digest(oracle_protocol())
            or oracle.get("candidate_sha256") != sha(core)
            or len(oracle.get("cases", oracle.get("case_metrics", []))) != 8):
        raise ValueError("Current TS20 predictor lacks the frozen eight-case oracle acceptance")
    native_source = ROOT / "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp"
    if (not Path(inputs["native_library"]).is_file()
            or sha(inputs["native_library"]) != inputs["native_library_sha256"]
            or native.get("sha256") != inputs["native_library_sha256"]
            or native.get("cpp_sha256") != sha(native_source) or native.get("abi") != 1):
        raise ValueError("Same-run oracle acceptance does not bind current native predictor source/library")
    plant = read(inputs["plant_manifest"])
    # Complete plant preparation has its own manifest.  Its exact checks are
    # performed by the preparation tool; binding the bytes prevents borrowing
    # a ground-only plugin experiment or an old ideal JointController model.
    if (plant.get("schema") != "x650_ts20_conditional_plant_v1" or plant.get("model_name") != selected["model"]
            or plant.get("model_sha256") != model.sha256
            or plant.get("preparation_report_sha256") != sha(cfg["preparation_report"])
            or plant.get("source_manifest_sha256") != sha(inputs["source_manifest"])
            or plant.get("native_validation", {}).get("passed") is not True
            or plant.get("prior_flight_gates_reused") is not False):
        raise ValueError("Complete imported TS20 flight-plant manifest required")
    for key in ("kp", "kd", "bus_period_s"):
        if plant["parameters"][key] != selected[key]:
            raise ValueError("Actual plant gain/bus disagrees with the frozen scenario variant: " + key)
    if (not Path(plant["plugin_library"]).is_file() or sha(plant["plugin_library"]) != plant["plugin_library_sha256"]
            or sha(ROOT / "scripts/ts20_interface_system.cpp") != plant["plugin_source_sha256"]):
        raise ValueError("Current loaded-plant plugin source or library changed")
    if cfg["scenario"] == DAMPED_SCENARIO:
        profile = ROOT / "configs/actuators/ts20_50_24v_damped_v1_candidate.json"
        binding = oracle.get("operating_condition", {})
        if (oracle.get("gain_profile") != "mit_kp6_kd025_v1"
                or binding.get("sha256") != sha(profile)
                or binding.get("sha256") != plant["candidate_profile_sha256"]
                or binding.get("profile_name") != "ts20_50_24v_damped_v1"
                or any(binding.get(key) != selected[key] for key in ("kp", "kd", "bus_period_s"))):
            raise ValueError("Damped gain-specific oracle does not bind the exact new candidate plant profile")
    profile, _, pid_hash = frozen_research_pid(cfg)
    if profile["model_sha256"] != model.sha256:
        raise ValueError("Frozen PID byte values must be explicitly rebound to this new plant")
    base_condition = condition_for_task(cfg)
    provenance = {"model_name": model.model_name, "model_sha256": model.sha256,
        "source_urdf_sha256": source_sha, "total_mass_kg": model.total_mass_kg,
        "preparation_report_sha256": sha(cfg["preparation_report"]),
        "airframe_sha256": sha(cfg["airframe_source"]), "flight_profile_sha256": manifest["profile_sha256"],
        "px4_sys_autostart": selected["airframe"], "native_recorder_sha256": sha(cfg["native_recorder_library"]),
        "stage": cfg["stage"], "controller": cfg["controller"],
        "operating_condition": base_condition, "operating_condition_sha256": canonical_hash(base_condition),
        "pid_parameters_sha256": pid_hash, "full_body_torque_nmpc": False,
        "base_response_is_prior": True, "ts20_model_metadata": motor.metadata()}
    provenance["ts20_operating_condition"] = condition(cfg, model_sha256=model.sha256, source_urdf_sha256=source_sha)
    provenance["ts20_operating_condition_sha256"] = canonical_hash(provenance["ts20_operating_condition"])
    provenance["stage_gates"] = validate_stage_gates(cfg, provenance)
    provenance["hover_world_sdf_sha256"] = sha(cfg["research_profile"]["world_sdf"])
    if cfg["stage"] == "tip_hold":
        prior = read(inputs["base_response_prior"])
        if prior.get("kind") != "x650_mixed_upper_velocity_response_model" or prior.get("pid_parameters_sha256") != pid_hash:
            raise ValueError("Explicit old-plant base-response prior with matching PX4 PID required")
        provenance["base_response_prior"] = {"path": inputs["base_response_prior"], "sha256": sha(inputs["base_response_prior"]),
            "source_model_sha256": prior.get("model_sha256"), "new_plant_identified": False}
    sources = source_files_for_task(cfg)
    provenance["source_files_sha256"] = {p.relative_to(ROOT).as_posix(): sha(p) for p in sources}
    return model, provenance
