"""Explicit trajectory admission after the complete native endpoint hold."""
from copy import deepcopy
import os
from pathlib import Path

from simulations.x650_ts20_hold_v1 import task as hold

ROOT = hold.ROOT
CONFIG_ENV = "X650_TS20_FIGURE8_CONFIG"
SCENARIO = DAMPED_SCENARIO = "x650_ts20_figure8_v1"
BASELINE_SHA256 = "f8d96cb8d70b55a9885b4723f5ea614d1978f4e50663921a2e9fe440b4357ae1"
HOLD_CERTIFICATE_SHA256 = "3a02609b58ef462590b76138131003fdeaed93aa6bea3528a3761e14e4564602"
sha, read, canonical_hash = hold.sha, hold.read, hold.canonical_hash


def protocol():
    return {"kind": "x650_frozen_native_figure8_v1", "stages": ["normal", "fast"],
            "baseline_task_sha256": BASELINE_SHA256,
            "hold_certificate_sha256": HOLD_CERTIFICATE_SHA256,
            "amplitude_x_m": .25, "amplitude_z_m": .10, "cycles": 2,
            "start_s": 2., "ramp_s": 2., "normal_period_s": 12., "fast_period_s": 8.,
            "tail_hold_s": 4., "tip_rms_max_m": .05, "tip_peak_max_m": .10,
            "tilt_max_deg": 8., "max_control_interval_s": .05, "max_compute_s": .05,
            "no_previous_plan_fallback": True, "full_window_required": True,
            "target_alignment": "same world frame and timestamp; no fitted translation or phase",
            "controller_parameters": "exact complete-hold task values; no retuning",
            "trajectory_authority": "this explicit protocol; retained ts20_protocol describes prior hold admission"}


def bound_document(binding, expected=None):
    if not isinstance(binding, dict) or set(binding) != {"path", "sha256"}:
        raise ValueError("Explicit path and SHA256 binding required")
    p = Path(binding["path"])
    if not p.is_absolute() or not p.is_file() or sha(p) != binding["sha256"]:
        raise ValueError("Bound experiment input changed or missing")
    if expected is not None and binding["sha256"] != expected:
        raise ValueError("Experiment must use the accepted frozen version")
    return read(p)


def baseline_config(binding):
    return hold.validate_task_config(bound_document(binding, BASELINE_SHA256))


def build_task_config(stage, baseline_binding, gate_bindings):
    if stage not in ("normal", "fast"):
        raise ValueError("Only normal then fast figure eight is admitted")
    cfg = baseline_config(baseline_binding)
    p = protocol()
    period = p[stage + "_period_s"]
    cfg.update(scenario=SCENARIO, stage=stage, trajectory=stage,
               trajectory_parameters={"amplitude_x": p["amplitude_x_m"],
                   "amplitude_z": p["amplitude_z_m"], "period_s": period,
                   "cycles": p["cycles"], "start_s": p["start_s"], "ramp_s": p["ramp_s"]},
               duration_s=8. + 2. * period, frozen_hold=deepcopy(baseline_binding),
               trajectory_gates=deepcopy(gate_bindings), figure8_protocol=p)
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict):
        raise ValueError("Figure-eight task must be an object")
    expected = build_task_config(value.get("stage"), value.get("frozen_hold"), value.get("trajectory_gates"))
    if canonical_hash(expected) != canonical_hash(value):
        raise ValueError("Frozen backend, controller parameters or trajectory protocol changed")
    gates = value["trajectory_gates"]
    required = {"tip_hold"} if value["stage"] == "normal" else {"tip_hold", "normal"}
    if not isinstance(gates, dict) or set(gates) != required:
        raise ValueError("Complete hold is required before normal; normal is required before fast")
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def condition(cfg, **kwargs):
    return hold.condition(baseline_config(cfg["frozen_hold"]), **kwargs)


def validate_stage_gates(cfg, provenance):
    baseline = baseline_config(cfg["frozen_hold"])
    result = hold.validate_stage_gates(baseline, provenance)
    identity = condition(cfg, model_sha256=provenance["model_sha256"],
                         source_urdf_sha256=provenance["source_urdf_sha256"])
    for name, binding in cfg["trajectory_gates"].items():
        doc = bound_document(binding, HOLD_CERTIFICATE_SHA256 if name == "tip_hold" else None)
        kind = ("x650_ts20_conditional_stage_certificate_v1" if name == "tip_hold"
                else "x650_ts20_figure8_stage_certificate_v1")
        if (doc.get("kind") != kind or doc.get("stage") != name
                or doc.get("ts20_operating_condition") != identity
                or any(doc.get(k) is not True for k in ("complete", "passed", "full_window_ulog_verified",
                       "parameter_restore_verified", "native_ground_verified", "actuator_log_verified"))):
            raise ValueError("Missing complete matching trajectory gate: " + name)
        if name == "normal" and (doc.get("figure8_protocol") != protocol()
                or doc.get("frozen_hold") != cfg["frozen_hold"]):
            raise ValueError("Normal and fast must share the same frozen backend and protocol")
        if not doc.get("artifacts"):
            raise ValueError("Gate lacks original artifacts")
        for file, expected in doc["artifacts"].items():
            p = Path(file)
            p = p if p.is_absolute() else Path(binding["path"]).parent / p
            if not p.is_file() or sha(p) != expected:
                raise ValueError("Trajectory gate artifact changed: " + str(p))
        result[name] = {**binding, "evidence": doc}
    return result


def source_files_for_task(cfg):
    return list(dict.fromkeys([*hold.source_files_for_task(baseline_config(cfg["frozen_hold"])),
        *sorted(Path(__file__).parent.glob("*.py")), ROOT / "scenarios" / (SCENARIO + ".yaml")]))


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    baseline = baseline_config(cfg["frozen_hold"])
    model, provenance = hold.preflight(baseline, check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(SCENARIO)
    if descriptor.source != "user" or descriptor.module != "simulations/x650_ts20_figure8_v1/run_case.py":
        raise ValueError("Dedicated trajectory entry point required")
    provenance.update(stage=cfg["stage"], frozen_hold=cfg["frozen_hold"],
                      figure8_protocol_sha256=canonical_hash(protocol()))
    provenance["stage_gates"] = validate_stage_gates(cfg, provenance)
    provenance["source_files_sha256"] = {p.relative_to(ROOT).as_posix(): sha(p) for p in source_files_for_task(cfg)}
    return model, provenance
