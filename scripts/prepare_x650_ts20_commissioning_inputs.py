#!/usr/bin/env python3
"""Copy a declared Gazebo world and rebind unchanged PID values to a NEW plant.

No parameter tuning, controller execution, simulator startup or hardware I/O.
All output files must be new; old plant certificates are never copied.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
CONDITIONAL_PLANTS = {
    "amsim_urdf_x650_2link_ts20_v1_flight": 4997,
    "amsim_urdf_x650_2link_ts20_damped_v1_flight": 4996,
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def prepare(source_world, source_pid, model_manifest, preparation_dir):
    sys.path[:0] = [str(ROOT / "src"), str(ROOT / "simulations/x650_wholebody_mpc_v1")]
    from amsim.modeling.manager import resolve
    from diagnostic_parameters import load_parameter_evidence
    from hover_reference import read_world_datum
    source_world, source_pid, model_manifest = [Path(p).resolve(strict=True) for p in (source_world, source_pid, model_manifest)]
    target = Path(preparation_dir).resolve(strict=True)
    model = resolve(str(model_manifest))
    if CONDITIONAL_PLANTS.get(model.model_name) != model.px4_sys_autostart:
        raise ValueError("Expected an explicitly paired TS20 conditional model/airframe")
    report_path = target / "preparation_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("conditional_actuator_plant", {}).get("sdf_sha256") != model.sha256:
        raise ValueError("Preparation report is not bound to this conditional plant")
    outputs = {name: target / name for name in ("world.sdf", "pid_profile.json", "commissioning_inputs_manifest.json")}
    if any(p.exists() for p in outputs.values()):
        raise FileExistsError("Refusing to replace an existing world, PID profile or inputs manifest")
    read_world_datum(source_world)
    root = ET.parse(source_world).getroot()
    if root.tag != "sdf" or len(root.findall("world")) != 1:
        raise ValueError("One declared native Gazebo world is required")
    checked = subprocess.run(["gz", "sdf", "-k", str(source_world)], capture_output=True, text=True, timeout=60)
    validation = (checked.stdout + checked.stderr).strip()
    if checked.returncode or "Valid." not in validation:
        raise ValueError("Source world failed native Gazebo validation: " + validation[-4000:])
    profile = json.loads(source_pid.read_text(encoding="utf-8"))
    parameters = profile.get("parameters")
    required = set(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
    if (not isinstance(parameters, dict) or set(parameters) != required or len(parameters) != 42
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in parameters.values())):
        raise ValueError("Source PID profile must contain all 42 finite canonical control readbacks")
    normalized = {k: float(v) for k, v in parameters.items()}
    digest = canonical(normalized)
    if profile.get("pid_parameters_sha256") != digest:
        raise ValueError("Source PID hash does not agree with unchanged numeric values")
    rebound = deepcopy(profile)
    rebound.update(model_sha256=model.sha256, parent_pid_profile=str(source_pid),
                   parent_pid_profile_sha256=sha(source_pid), parameter_values_unchanged=True,
                   new_plant_tuned=False, hardware_performance_claim=False)
    world_bytes = source_world.read_bytes()
    pid_bytes = (json.dumps(rebound, indent=2, ensure_ascii=False) + "\n").encode("utf-8")
    # Exclusive creation also protects an existing run from accidental reruns.
    for name, data in (("world.sdf", world_bytes), ("pid_profile.json", pid_bytes)):
        with outputs[name].open("xb") as stream:
            stream.write(data)
    manifest = {"schema": "x650_ts20_commissioning_inputs_v1", "simulation_only": True,
                "model": model.model_name, "model_sha256": model.sha256,
                "preparation_report_sha256": sha(report_path),
                "source_world": str(source_world), "source_world_sha256": sha(source_world),
                "source_pid": str(source_pid), "source_pid_sha256": sha(source_pid),
                "pid_parameters_sha256": digest, "parameter_values_unchanged": True,
                "new_plant_tuned": False, "prior_flight_gates_reused": False,
                "world_native_validation": {"passed": True, "output": validation},
                "generator_sha256": sha(__file__),
                "artifacts": {name: sha(outputs[name]) for name in ("world.sdf", "pid_profile.json")}}
    with outputs["commissioning_inputs_manifest.json"].open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source-world", "source-pid", "model-manifest", "preparation-dir"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source_world, args.source_pid, args.model_manifest, args.preparation_dir), indent=2))


if __name__ == "__main__":
    main()
