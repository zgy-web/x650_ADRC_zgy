#!/usr/bin/env python3
"""Unpowered native TS20 interface/direction probe; never starts PX4.

This checks velocity-interface plumbing, not a new flight-stage certificate or
the loaded response of a real TS20. Existing simulator processes are refused.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def probe(manifest, output):
    sys.path.insert(0, str(ROOT / "src"))
    from amsim.modeling.manager import resolve
    model = resolve(str(Path(manifest).resolve(strict=True)))
    if model.model_name != "amsim_urdf_x650_2link_ts20_v1_flight":
        raise ValueError("Explicit new conditional TS20 plant required")
    for process in ("px4", "MicroXRCEAgent"):
        if subprocess.run(["pgrep", "-x", process], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError("Existing " + process + " process; refuse probe")
    if subprocess.run(["pgrep", "-f", "gz sim|gz-sim.*server"], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError("Existing Gazebo runtime; refuse probe")
    out = Path(output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    name = model.model_name
    root = ET.Element("sdf", version="1.9")
    world = ET.SubElement(root, "world", name="ts20_interface_ground")
    physics = ET.SubElement(world, "physics", name="physics", type="ignored")
    ET.SubElement(physics, "max_step_size").text = ".004"
    ET.SubElement(physics, "real_time_factor").text = "1"
    for filename, plugin in (("gz-sim-physics-system", "gz::sim::systems::Physics"),
                              ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands")):
        ET.SubElement(world, "plugin", filename=filename, name=plugin)
    ET.SubElement(world, "gravity").text = "0 0 -9.80665"
    ground = ET.SubElement(world, "model", name="ground_plane")
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", name="link")
    collision = ET.SubElement(link, "collision", name="collision")
    plane = ET.SubElement(ET.SubElement(collision, "geometry"), "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "100 100"
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://" + name
    ET.SubElement(include, "name").text = name
    ET.SubElement(include, "pose").text = "0 0 .375 0 0 0"
    world_path = out / "world.sdf"
    ET.ElementTree(root).write(world_path, encoding="utf-8", xml_declaration=True)
    env = os.environ.copy()
    env.update(AMSIM_TS20_LOG_DIR=str(out), GZ_PARTITION="ts20_ground_probe_" + str(os.getpid()),
               GZ_SIM_RESOURCE_PATH=str(model.sdf_path.parent.parent))
    log = (out / "gz.log").open("w")
    process = subprocess.Popen(["gz", "sim", "-s", "-r", str(world_path), "-v", "2"], env=env,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    schedule = [(2., [.03, 0.]), (3., [-.03, 0.]), (4., [0., 0.]),
                (5., [0., .03]), (6., [0., -.03]), (9., [0., 0.])]
    edges, error, index, latest = [], None, 0, None
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Gazebo exited " + str(process.returncode))
            path = out / "ts20_state.csv"
            if path.exists():
                with path.open() as stream:
                    rows = [row for row in csv.DictReader(stream) if row.get("active_dqref2") not in (None, "")]
                if rows:
                    latest = float(rows[-1]["sim_s"])
                    if index < len(schedule) and latest >= schedule[index][0]:
                        when, command = schedule[index]
                        for j, value in enumerate(command, 1):
                            subprocess.run(["gz", "topic", "-t", "/model/" + name + "/joint/arm_joint_" + str(j) + "/cmd_vel",
                                            "-m", "gz.msgs.Double", "-p", "data: " + str(value)],
                                           env=env, capture_output=True, text=True, timeout=5, check=True)
                        edges.append({"requested_at_sim_s": when, "observed_send_begin_sim_s": latest, "command": command})
                        index += 1
                    if latest >= 11. and index == len(schedule):
                        break
            time.sleep(.03)
        else:
            raise RuntimeError("Ground probe wall timeout; last simulation time=" + str(latest))
    except BaseException as exc:
        error = repr(exc)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
    status = {"error": error, "edges": edges, "no_px4_started": True,
              "no_rotor_command": True, "complete": error is None}
    (out / "probe_status.json").write_text(json.dumps(status, indent=2) + "\n")
    if error:
        raise RuntimeError(error)
    states = np.genfromtxt(out / "ts20_state.csv", delimiter=",", names=True)
    packets = np.genfromtxt(out / "ts20_packets.csv", delimiter=",", names=True)
    result = {"kind": "ts20_native_physics_interface_probe", "complete": True, "simulation_only": True,
              "new_model_flight_validated": False, "no_px4_started": True, "no_rotor_command": True,
              "model_sha256": model.sha256, "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "native_samples": len(states), "native_step_s": float(np.median(states["step_dt_s"])),
              "bus_packets": len(packets), "bus_interval_max_error_s": float(np.max(abs(np.diff(packets["send_s"]) - .005))),
              "per_joint": {}}
    for j in (1, 2):
        q, internal, dq = (states[key] for key in (f"q{j}_physical", f"q{j}_internal", f"dq{j}_physical"))
        result["per_joint"][str(j)] = {
            "q_internal_physical_max_error_rad": float(np.max(abs(q - internal))),
            "dq_physical_range_rad_s": [float(np.min(dq)), float(np.max(dq))],
            "both_movement_directions_observed": bool(np.max(dq) > .01 and np.min(dq) < -.01)}
    result["passed"] = result["bus_interval_max_error_s"] < 1e-10 and all(
        v["q_internal_physical_max_error_rad"] <= 1e-4 and v["both_movement_directions_observed"]
        for v in result["per_joint"].values())
    result["scope"] = "Conditional interface plumbing/direction only; no flight or 24V/loaded qualification."
    result["artifacts"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
    (out / "probe_analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = probe(args.model_manifest, args.output)
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
