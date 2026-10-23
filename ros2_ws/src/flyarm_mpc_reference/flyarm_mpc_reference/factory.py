"""One-time, hash-pinned construction from the real TS20 task artifacts.

No synthetic test model, default motor, PID fallback, ScenarioContext or flight
launch is selected here. A synthetic initialization is allowed only when it is
explicitly labelled; it does not become a historical replay or hardware claim.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

from .engine import ReferenceEngine, SnapshotContext, finite_scalar, vector


def _pinned_json(path, expected):
    path = Path(path).resolve(strict=True)
    content = path.read_bytes()
    if (not isinstance(expected, str) or len(expected) != 64
            or set(expected)-set("0123456789abcdef") or hashlib.sha256(content).hexdigest() != expected):
        raise ValueError("Caller-pinned input SHA-256 differs: " + str(path))
    return json.loads(content), {"path": str(path), "sha256": expected}


def create_engine(repo_root, task_config, task_config_sha256, initialization_manifest,
                  initialization_sha256, session_id, epoch):
    root = Path(repo_root).resolve(strict=True)
    for directory in (root, root/"src", root/"scripts", root/"simulations/x650_wholebody_mpc_v1",
                      root/"control_packages/x650_wholebody_v1"):
        if not directory.is_dir():
            raise ValueError("Explicit existing reference repository required: " + str(directory))
        if str(directory) not in sys.path:
            sys.path.insert(0, str(directory))
    cfg, cfg_identity = _pinned_json(task_config, task_config_sha256)
    initialization, init_identity = _pinned_json(initialization_manifest, initialization_sha256)
    if (type(initialization.get("schema_version")) is not int or initialization.get("schema_version") != 1
            or type(initialization.get("synthetic_software_fixture_only")) is not bool):
        raise ValueError("Explicit schema-1 initialization and synthetic/historical scope required")
    from simulations.x650_ts20_hold_v1 import task
    from simulations.x650_wholebody_mpc_v1.research_condition import frozen_research_pid
    from simulations.x650_wholebody_mpc_v1.run_case import ConstantTarget
    from x650_wholebody.model import ArmModel
    from x650_wholebody.api import ControlState, EndpointTarget
    from x650_wholebody.ts20 import FrozenTs20Model
    from x650_wholebody.ts20_mpc import Ts20MpcController
    from x650_wholebody.adapter import ControllerAdapter
    if task.ROOT.resolve() != root:
        raise ValueError("An already imported TS20 task belongs to another repository")
    cfg = task.validate_task_config(cfg)
    if cfg["stage"] != "tip_hold" or cfg["controller"] != "mpc" or cfg["disturbance_enabled"]:
        raise ValueError("Phase 1 worker permits only the explicit quiet tip-hold MPC task")
    _, provenance = task.preflight(cfg, check_airframe=False)
    model = ArmModel.from_report(cfg["preparation_report"])
    _, pid, _ = frozen_research_pid(cfg)
    inputs = cfg["ts20_inputs"]
    motor = FrozenTs20Model(inputs["source_dir"], task.read(inputs["source_manifest"]))
    predictor = task.predictor_from_bound_plant(cfg, motor.metadata()["motion_model"])
    binding = provenance["base_response_prior"]
    if task.sha(binding["path"]) != binding["sha256"]:
        raise ValueError("Hash-bound base-response prior changed during construction")
    prior = task.read(binding["path"])
    config = deepcopy(cfg["mpc_config"])
    config.pop("prediction_chain", None)
    config.update(response_identified=False,
                  response_model={"kind": "xy_second_order_z_first_order_v1", "parameters": prior["parameters"]})
    origin = finite_scalar(initialization["task_origin_sim_s"], "task_origin_sim_s")
    if origin < 0:
        raise ValueError("Task origin must be nonnegative")
    p0 = vector(initialization["anchor_position_enu"], 3, "anchor position")
    R0 = vector(initialization["anchor_rotation_row_major"], 9, "anchor rotation").reshape(3, 3)
    q0 = vector(initialization["anchor_q"], 2, "anchor q")
    center = vector(initialization["target_position_enu"], 3, "hold target")
    context = SnapshotContext()
    controller, options = Ts20MpcController, {}
    if "numerical_backend" in cfg:
        from x650_wholebody.native_mpc import NativeTs20MpcController
        controller = NativeTs20MpcController
        options = {"native_library": cfg["numerical_backend"]["library"],
                   "native_sha256": cfg["numerical_backend"]["library_sha256"]}
    allocator = controller(model, pid, trajectory=ConstantTarget(center), config=config,
                           predictor=predictor, context_provider=context, **options)
    adapter = ControllerAdapter(allocator, model, pid, acceleration_feedforward_policy="zero")
    adapter.reset(p0, R0, q0)
    sent = initialization["published_prestream_history"]
    if not isinstance(sent, list) or not sent:
        raise ValueError("Actual published five-axis prestream history is required")
    history = [(finite_scalar(item["control_time_s"], "published control time"),
                vector(item["command"], 5, "published five-axis command")) for item in sent]
    if abs(history[-1][0]-origin) > 1e-9:
        raise ValueError("Task origin must be the last actual prestream publication")
    allocator.prime_command_history(history, origin)
    # Construction reads artifacts once. Per-snapshot processing only binds
    # the typed source state/context and calls the same adapter.step method.
    return ReferenceEngine(adapter, context, task_origin=origin, position_gains=adapter.position_gains,
        target_position=center, session_id=session_id, epoch=epoch,
        configuration_sha256=bytes.fromhex(task_config_sha256),
        state_factory=ControlState, target_factory=EndpointTarget,
        metadata={"repo_root": str(root), "task_config": cfg_identity, "initialization": init_identity,
                  "synthetic_software_fixture_only": initialization["synthetic_software_fixture_only"],
                  "historical_reproduction_claimed": False,
                  "physical_flight_qualification": False, "native_predictor": predictor.metadata()["native_accelerator"],
                  "numerical_backend": cfg.get("numerical_backend", {"kind": "python_reference"}),
                  "scope": "offline/shadow process; explicitly selected numerical backend; no real-time or flight claim"})
