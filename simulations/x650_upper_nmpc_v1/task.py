"""Exact paired tasks derived from the existing 25/50 Hz quiet-hold profile.

Historical artifacts establish plant/PID preparation only. A new NMPC task
never inherits the old MPC flight's acceptance; each backend has its own new
hold -> normal -> fast chain. No functions in this module start a simulator.
"""
from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path

from simulations.x650_cpp_multirate_v1 import profile
from simulations.x650_cpp_multirate_v1 import task as multirate
from simulations.x650_cpp_normal_v1 import task as normal
from simulations.x650_cpp_fast_v1 import task as fast

ROOT, CONFIG_ENV = multirate.ROOT, multirate.CONFIG_ENV
read, sha, canonical_hash = multirate.read, multirate.sha, multirate.canonical_hash
bound_document = normal.bound_document
CASES = ("hold", "normal", "fast")
BACKENDS = ("nmpc", "mpc_callable")
MODEL = "amsim_urdf_x650_2link_ts20_damped_v1_flight"
KIND = "x650_upper_controller_task_v1"


def scenario_name(case_id, backend):
    if case_id not in CASES or backend not in BACKENDS:
        raise ValueError("Explicit hold/normal/fast and nmpc/mpc_callable required")
    label = "mpc" if backend == "mpc_callable" else "nmpc"
    return f"x650_upper_{label}_{case_id}_v1"


def original_config(binding):
    cfg = multirate.validate_task_config(bound_document(binding, recurse=False))
    if (cfg["multirate"]["case_id"] != "tip_hold" or cfg["stage"] != "tip_hold"
            or cfg["controller"] != "mpc" or cfg["model"] != MODEL
            or cfg["duration_s"] != 30. or cfg["disturbance_enabled"]
            or cfg["timing_profile"] != profile.NAME):
        raise ValueError("Exact original 30 s quiet multirate hold task required")
    return cfg


def original(cfg):
    return original_config(cfg["upper_controller"]["original_task"])


def baseline_config(cfg):
    """Original damped-plant adapter config, not a new flight certificate."""
    return multirate.baseline_config(original(cfg))


def trajectory(cfg):
    case_id = cfg["upper_controller"]["case_id"]
    if case_id == "hold":
        return {"kind": "hold"}
    if case_id == "normal":
        return normal.trajectory()
    if case_id == "fast":
        return fast.trajectory()
    raise ValueError("Unknown upper-controller trajectory")


def source_parameters(cfg):
    value = trajectory(cfg)
    if value["kind"] == "hold":
        return {"trajectory_kind": "hold"}
    return {"trajectory_kind": "figure8", **{
        key: item for key, item in value.items() if key not in {"kind", "duration_s"}}}


def build_task_config(case_id, backend, original_task_binding, *, prior_stage_certificate=None):
    scenario = scenario_name(case_id, backend)
    cfg = original_config(original_task_binding)
    if case_id == "hold":
        if prior_stage_certificate is not None:
            raise ValueError("New hold cannot borrow a prior physical acceptance")
    else:
        # Verify binding and bytes, without treating a legacy certificate as a
        # new physical gate. Its contents are audited at preflight below.
        bound_document(prior_stage_certificate, recurse=False)
    cfg["upper_controller"] = {
        "kind": KIND,
        "backend": backend,
        "case_id": case_id,
        "original_task": deepcopy(original_task_binding),
        "prior_stage_certificate": deepcopy(prior_stage_certificate),
        "timing_profile": profile.NAME,
        "simulation_only": True,
        "physical_acceptance_inherited": False,
        "hardware_qualified": False,
        "controller_field_scope": "legacy shared lifecycle selector; actual solver is backend",
    }
    value = trajectory(cfg)
    cfg.update(scenario=scenario, stage="tip_hold" if case_id == "hold" else case_id,
               trajectory="hover" if case_id == "hold" else case_id,
               duration_s=30. if case_id == "hold" else value["duration_s"],
               trajectory_parameters=None if case_id == "hold" else {
                   key: item for key, item in value.items() if key not in {"kind", "duration_s"}})
    # Preserve all existing physical/controller values. The source derivation
    # remains visible; upper_controller describes the new experiment and gate.
    if case_id != "hold":
        cfg["figure8_protocol"] = {
            "kind": "x650_upper_controller_figure8_protocol_v1",
            "trajectory": value,
            "tail_hold_s": 4.,
            "target_alignment": "same world frame and timestamp; no fitted translation or phase",
            "controller_parameters": "same original multirate model/PID/weights/hard limits",
        }
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or not isinstance(value.get("upper_controller"), dict):
        raise ValueError("Explicit upper-controller task derivation required")
    item = value["upper_controller"]
    expected = build_task_config(item.get("case_id"), item.get("backend"),
        item.get("original_task"), prior_stage_certificate=item.get("prior_stage_certificate"))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError("Undeclared upper model/PID/weights/limits/reference/timing change")
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def protocol(cfg):
    cfg = validate_task_config(cfg)
    item = cfg["upper_controller"]
    value = multirate.protocol(original(cfg))
    value.update(kind="x650_upper_controller_protocol_v1", backend=item["backend"],
        case_id=item["case_id"], duration_s=cfg["duration_s"], trajectory=trajectory(cfg),
        trajectory_kind="hold" if item["case_id"] == "hold" else "figure8",
        required_prior_stage={"hold": "new_backend_software", "normal": "new_same_backend_hold",
                              "fast": "new_same_backend_normal"}[item["case_id"]],
        order=["software_and_actual_worker", "complete_hold", "normal_figure_eight", "fast_figure_eight"],
        qualification_scope="New same-interface backend and physical stage; no inherited flight acceptance",
        diagnostic_policy="bounded actual input/output and nonlinear decision records; auxiliary persistence; no loss",
        controller_parameters="original multirate physical model/PID/weights/limits unchanged")
    value["active_pipeline"] = ["cpp_raw_estimated_state_and_actual_joint_history", "cpp_admission",
        "cpp_upper_" + item["backend"], "cpp_periodic_dispatch", "cpp_actual_consumption"]
    value["claims_excluded"] = ["hardware_24v_validation", "hard_realtime", "full_body_torque_nmpc",
        "nonlinear_nlp_convergence", "software_replay_as_new_physical_hold"]
    return value


def physical_condition(cfg, *, check_airframe=True):
    return multirate.physical_condition(original(cfg), check_airframe=check_airframe)


def condition(cfg, **kwargs):
    from simulations.x650_ts20_hold_v1 import task as hold
    return hold.condition(baseline_config(cfg), **kwargs)


def validate_stage_gates(cfg, provenance):
    item = cfg["upper_controller"]
    result = {
        "historical_plant_preparation": deepcopy(item["original_task"]),
        "historical_scope": "Plant/PID preparation only; no new backend acceptance inherited",
        "backend": item["backend"],
        "new_actual_ground_required": True,
        "timing_profile": profile.NAME,
    }
    if item["case_id"] != "hold":
        certificate = bound_document(item["prior_stage_certificate"], recurse=False)
        previous = "hold" if item["case_id"] == "normal" else "normal"
        if (certificate.get("kind") != "x650_upper_closed_loop_audit_v1"
                or certificate.get("backend") != item["backend"]
                or certificate.get("case_id") != previous
                or any(certificate.get(key) is not True for key in
                       ("passed", "execution_passed", "performance_passed"))):
            raise ValueError("A new accepted same-backend " + previous + " stage is required")
        # The new qualification module verifies bound task/config/model/launch
        # identities and raw evidence; this is not a boolean-only admission.
        from .qualification import verify_prior_stage
        result["new_same_backend_" + previous] = verify_prior_stage(
            item["prior_stage_certificate"], cfg, provenance)
    return result


def execution_sources(cfg):
    scenario = scenario_name(cfg["upper_controller"]["case_id"], cfg["upper_controller"]["backend"])
    return (
        "simulations/x650_upper_nmpc_v1/__init__.py",
        "simulations/x650_upper_nmpc_v1/task.py",
        "simulations/x650_upper_nmpc_v1/run_case.py",
        f"simulations/{scenario}/__init__.py",
        f"simulations/{scenario}/run_case.py",
        f"scenarios/{scenario}.yaml",
    )


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    model, provenance = multirate.preflight(original(cfg), check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(cfg["scenario"])
    if (descriptor.source != "user" or descriptor.module != "simulations/" + cfg["scenario"] + "/run_case.py"
            or descriptor.config["model"] != cfg["model"]):
        raise ValueError("Dedicated upper-controller scenario/model registration required")
    provenance = deepcopy(provenance)
    provenance.update(stage=cfg["stage"], upper_controller=deepcopy(cfg["upper_controller"]),
        timing_profile=profile.NAME, effective_timing_profile=profile.effective(),
        trajectory=trajectory(cfg), physical_acceptance_inherited=False)
    provenance["stage_gates"] = validate_stage_gates(cfg, provenance)
    for relative in execution_sources(cfg):
        path = ROOT / relative
        if not path.is_file():
            raise ValueError("Missing new upper-controller execution source: " + relative)
        provenance["source_files_sha256"][relative] = sha(path)
    return model, provenance
