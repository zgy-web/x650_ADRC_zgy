#!/usr/bin/env python3
"""Build and derive an explicitly conditional TS20 velocity-interface plant.

Never overwrites an imported model or runs hardware. The upstream motion source
is a caller-owned local, hash-pinned dependency, not vendored by this script.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
CPP = Path(__file__).with_name("ts20_interface_system.cpp")
PACKAGES = ("gz-sim8", "gz-plugin2-register", "gz-transport13", "gz-msgs10")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def number(value, name, *, lower=None, upper=None):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(name + " must be an explicit finite number")
    result = float(value)
    if (lower is not None and result < lower) or (upper is not None and result > upper):
        raise ValueError(name + " is outside the reviewed range")
    return result


def verify_model(source_root, source_manifest, motion_json):
    import sys
    package = ROOT / "control_packages/x650_wholebody_v1"
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))
    from x650_wholebody.ts20 import FrozenTs20Model
    manifest = json.loads(source_manifest.read_text(encoding="utf-8"))
    model = FrozenTs20Model(source_root, manifest)
    expected = source_root / "models/ts20_50/motion.json"
    if motion_json.resolve() != expected.resolve() or sha(motion_json) != manifest["files"]["models/ts20_50/motion.json"]:
        raise ValueError("motion-json must be the verified local dependency motion.json")
    data = json.loads(motion_json.read_text(encoding="utf-8"))
    return data, model.metadata(), manifest


def build_library(build_dir, cxx):
    if build_dir.exists():
        raise FileExistsError("Refusing to replace existing build directory: " + str(build_dir))
    compiler, pkg_config = shutil.which(cxx), shutil.which("pkg-config")
    if compiler is None or pkg_config is None:
        raise ValueError("An installed C++17 compiler and pkg-config are required; no download is performed")
    flags = subprocess.run([pkg_config, "--cflags", "--libs", *PACKAGES], capture_output=True, text=True, check=True)
    build_dir.mkdir(parents=True)
    library = build_dir / "libts20_interface.so"
    core = build_dir / "ts20_core_self_test"
    compile_log = build_dir / "compile.log"
    commands = [
        [compiler, "-std=c++17", "-O2", "-DTS20_CORE_SELF_TEST", str(CPP), "-o", str(core)],
        [compiler, "-std=c++17", "-O2", "-fPIC", "-shared", "-Wl,--no-undefined", str(CPP), "-o", str(library), *shlex.split(flags.stdout)],
    ]
    with compile_log.open("w", encoding="utf-8") as stream:
        for command in commands:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError("TS20 build failed; inspect " + str(compile_log))
    tested = subprocess.run([str(core)], capture_output=True, text=True, check=True)
    metadata = {"source": str(CPP), "source_sha256": sha(CPP), "library": str(library),
                "library_sha256": sha(library), "commands": commands,
                "self_test_output": tested.stdout.strip(), "self_test_passed": True,
                "compiler": subprocess.run([compiler, "--version"], capture_output=True, text=True, check=True).stdout.splitlines()[0],
                "packages": {name: subprocess.run([pkg_config, "--modversion", name], capture_output=True, text=True, check=True).stdout.strip() for name in PACKAGES}}
    (build_dir / "build_manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return library, metadata


def derive(args):
    source = args.source_sdf.resolve(strict=True)
    destination = args.output_dir.resolve()
    source_root = args.source_root.resolve(strict=True)
    source_manifest = args.source_manifest.resolve(strict=True)
    motion_json = args.motion_json.resolve(strict=True)
    profile_path = args.candidate_profile.resolve(strict=True)
    if destination.exists():
        raise FileExistsError("Refusing to overwrite an existing plant: " + str(destination))
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", args.model_name):
        raise ValueError("Use an explicit safe SDF model name")
    if source.parent == destination or source.parent in destination.parents:
        raise ValueError("Output must be outside the source model directory")
    motion, provenance, manifest = verify_model(source_root, source_manifest, motion_json)
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if (profile.get("schema") != "x650_ts20_candidate_profile_v1"
            or profile.get("status") != "conditional_simulation_only"
            or profile.get("hardware_confirmed", {}).get("motor") != "TS20-50"
            or profile.get("hardware_confirmed", {}).get("joints") != 2
            or profile.get("hardware_confirmed", {}).get("supply_v") != 24
            or profile.get("upstream", {}).get("commit") != manifest["upstream_commit"]):
        raise ValueError("Expected the explicit TS20-50 / 24 V candidate profile and matching pinned source")
    candidate = profile["mit_candidate"]
    if candidate.get("tau_ff") != 0:
        raise ValueError("Only zero nominal feedforward is supported by this candidate plant")
    p, f = motion["fixed_parameters"], motion["friction_parameters"]
    parameters = {
        "normalized_inertia": number(p["J_total_over_ka"], "normalized inertia", lower=1e-12),
        "normalized_viscous": number(p["b_over_ka"], "normalized viscous", lower=0),
        "normalized_coulomb": number(p["Fc_over_ka"], "normalized Coulomb", lower=0),
        "normalized_offset": number(p["offset_over_ka"], "normalized offset"),
        "combined_interface_delay_s": number(p["combined_interface_delay_s"], "combined interface delay", lower=0, upper=.1),
        "static_threshold": number(f["static_threshold"], "static threshold", lower=0),
        "delta_static": number(f["delta_static"], "Stribeck delta", lower=0),
        "stribeck_velocity": number(f["stribeck_velocity"], "Stribeck velocity", lower=1e-12),
        "bus_period_s": number(candidate["bus_period_s"], "bus period", lower=.001, upper=.01),
        "kp": number(candidate["kp"], "Kp", lower=0),
        "kd": number(candidate["kd"], "Kd", lower=0),
    }
    if parameters["static_threshold"] < parameters["normalized_coulomb"] + parameters["delta_static"]:
        raise ValueError("Static threshold must cover the zero-speed sliding limit")
    tree = ET.parse(source)
    model = tree.getroot().find("model")
    if model is None or len(tree.getroot().findall("model")) != 1 or model.findall("model"):
        raise ValueError("Expected one non-nested model")
    old_name = model.get("name")
    required = {"arm_joint_1", "arm_joint_2"}
    found = []
    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::JointController":
            name = plugin.findtext("joint_name")
            if name not in required:
                raise ValueError("Unexpected additional JointController: " + str(name))
            found.append(name)
            model.remove(plugin)
        elif "ts20" in (plugin.get("filename", "") + plugin.get("name", "")).lower():
            raise ValueError("Source already includes a TS20 system")
    if set(found) != required or len(found) != 2:
        raise ValueError("Expected exactly one old velocity JointController per arm joint")
    if len([p for p in model.findall("plugin") if p.get("name") == "gz::sim::systems::JointStatePublisher"]) != 1:
        raise ValueError("Expected one existing joint-state publisher")
    library, build_metadata = (build_library(args.build_dir.resolve(), args.cxx) if args.build else
                               (args.library.resolve(strict=True), None))
    log_dir = args.log_dir.resolve()
    log_dir.mkdir(parents=True, exist_ok=True)
    state_csv, packet_csv = log_dir / "ts20_state.csv", log_dir / "ts20_packets.csv"
    if state_csv.exists() or packet_csv.exists():
        raise FileExistsError("Refusing to replace existing TS20 state/packet evidence")
    plugin = ET.SubElement(model, "plugin", filename=str(library), name="ts20interface::TS20Interface")
    for key, value in parameters.items():
        ET.SubElement(plugin, key).text = format(value, ".17g")
    ET.SubElement(plugin, "state_csv").text = str(state_csv)
    ET.SubElement(plugin, "packet_csv").text = str(packet_csv)
    model.set("name", args.model_name)
    for node in model.iter():
        for key in ("relative_to", "expressed_in", "attached_to"):
            value = node.get(key)
            if value == old_name:
                node.set(key, args.model_name)
            elif value and value.startswith(old_name + "::"):
                node.set(key, args.model_name + value[len(old_name):])
        if node.tag == "uri" and node.text and node.text.startswith("model://" + old_name + "/"):
            node.text = "model://" + args.model_name + "/" + node.text[len("model://" + old_name + "/"):]
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".ts20-plant-", dir=destination.parent) as temporary:
        staged = Path(temporary) / args.model_name
        # Mesh and texture resources stay byte-identical; no import manifest is
        # copied under a misleading old SDF hash.
        for path in source.parent.rglob("*"):
            if path.is_symlink():
                raise ValueError("Source model contains a symlink: " + str(path))
        shutil.copytree(source.parent, staged)
        for name in ("import_manifest.json", "model.config"):
            path = staged / name
            if path.exists():
                path.unlink()
        ET.indent(tree, space="  ")
        tree.write(staged / "model.sdf", encoding="utf-8", xml_declaration=True)
        config = ET.Element("model")
        ET.SubElement(config, "name").text = args.model_name
        ET.SubElement(config, "version").text = "1.0"
        ET.SubElement(config, "sdf", version=tree.getroot().get("version", "1.9")).text = "model.sdf"
        ET.SubElement(config, "description").text = "Conditional TS20-50 interface surrogate; no physical torque or 24V loaded validation."
        ET.ElementTree(config).write(staged / "model.config", encoding="utf-8", xml_declaration=True)
        env = os.environ.copy()
        for key in ("GZ_SIM_RESOURCE_PATH", "SDF_PATH"):
            env[key] = str(staged.parent) + (os.pathsep + env[key] if env.get(key) else "")
        checked = subprocess.run(["gz", "sdf", "-k", str(staged / "model.sdf")], capture_output=True, text=True, env=env, timeout=60)
        validation = (checked.stdout + checked.stderr).strip()
        if checked.returncode or "Valid." not in validation:
            raise RuntimeError("Native SDF validation failed: " + validation[-4000:])
        evidence = {
            "schema": "x650_ts20_conditional_plant_v1", "simulation_only": True,
            "model_name": args.model_name, "model_sha256": sha(staged / "model.sdf"),
            "source_sdf": str(source), "source_sdf_sha256": sha(source),
            "plugin_source": str(CPP), "plugin_source_sha256": sha(CPP),
            "plugin_library": str(library), "plugin_library_sha256": sha(library),
            "build": build_metadata, "candidate_profile_sha256": sha(profile_path),
            "source_manifest_sha256": sha(source_manifest), "upstream": provenance,
            "parameters": parameters, "state_csv": str(state_csv), "packet_csv": str(packet_csv),
            "removed_velocity_controllers": sorted(found), "joint_state_publisher_preserved": True,
            "native_validation": {"passed": True, "output": validation},
            "clock_contract": {
                "physics_interval": "PreUpdate integrates [info.simTime-info.dt,info.simTime]; PostUpdate logs actual physical state at interval end",
                "command_receipt": "first physics PreUpdate consuming latest transport callback; multiple callbacks within a step collapse to latest and are counted",
                "upper_reference": "continuous ZOH upper-velocity integral from one initial physical q; never reset to measured q",
                "bus": "fixed origin+n*period grid; exact event splitting, right-open physics interval",
                "delay": "combined interface delay applied after nominal bus send, plus actual external transport; not separately identified firmware delay",
                "mechanics": "RK4 <=0.25ms within packet events; bisection true velocity-zero, static equality sticks",
                "physical_actuation": "JointVelocityCmd=(internal_q_end-internal_q_begin)/physics_dt",
                "physical_velocity": "interval mean; distinct from internal endpoint dq and independently logged",
                "reset": "sim-time reset/gap rejected; use fresh plugin instance and fresh evidence paths"},
            "claims": {"physical_torque_model": False, "actual_loaded_arm_qualified": False,
                       "24v_qualified": False, "reference_event_solver_bit_exact": False,
                       "joint_inertias_changed": False, "mass_inertias_preserved": True},
            "limits": "No hidden clipping; normal task/native supervisor must enforce position and physical speed envelopes",
            "artifacts": {p.relative_to(staged).as_posix(): sha(p) for p in staged.rglob("*") if p.is_file()},
        }
        (staged / "plant_manifest.json").write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        staged.rename(destination)
    return evidence


def complete_import(args, evidence):
    """Keep original rigid geometry certificates; bind a NEW execution plant.

    These copies certify unchanged geometry only. No old ground/flight or
    response-validation certificate is promoted to the new actuator plant.
    """
    from amsim.modeling.manager import resolve, install
    from x650_wholebody.model import ArmModel
    destination = args.output_dir.resolve()
    source_model = args.source_sdf.resolve().parent
    source_prep = args.source_preparation.resolve(strict=True)
    prep = args.preparation_dir.resolve()
    airframe_dir = args.airframe_dir.resolve()
    px4 = args.px4_dir.resolve(strict=True)
    if prep.exists() or airframe_dir.exists():
        raise FileExistsError("New preparation and airframe output directories must not exist")
    if type(args.airframe_id) is not int or not 1 <= args.airframe_id <= 999999:
        raise ValueError("A new positive explicit airframe ID is required")
    airframe_source = args.source_airframe.resolve(strict=True)
    runtime_airframes = [px4 / "ROMFS/px4fmu_common/init.d-posix/airframes",
                        px4 / "build/px4_sitl_default/etc/init.d-posix/airframes"]
    for folder in runtime_airframes:
        if not folder.is_dir():
            raise ValueError("Missing PX4 airframe directory: " + str(folder))
        conflicts = list(folder.glob(str(args.airframe_id) + "_*"))
        if conflicts:
            raise FileExistsError("New airframe ID is already occupied: " + str(conflicts))
    # Explicitly verify every source resource, before inheriting its topology.
    old_spec = resolve(str(source_model / "import_manifest.json"))
    old_manifest = json.loads((source_model / "import_manifest.json").read_text(encoding="utf-8"))
    old_tree = ET.parse(source_model / "model.sdf").getroot().find("model")
    new_tree = ET.parse(destination / "model.sdf").getroot().find("model")
    def rigid_contract(model):
        normalized = deepcopy(model)
        for node in normalized.iter("uri"):
            if node.text and node.text.startswith("model://" + model.get("name") + "/"):
                node.text = "model://UNCHANGED_RESOURCE/" + node.text[len("model://" + model.get("name") + "/"):]
        return [(link.get("name"), ET.tostring(link.find("inertial")),
                 [ET.tostring(collision) for collision in link.findall("collision")])
                for link in normalized.findall("link")]
    if rigid_contract(old_tree) != rigid_contract(new_tree):
        raise ValueError("Conditional actuator derivation changed a rigid inertia or collision")
    if any(p.get("name") == "gz::sim::systems::JointController" for p in new_tree.findall("plugin")):
        raise ValueError("Competing velocity controller remains in conditional plant")
    shutil.copytree(source_prep.parent, prep)
    original_report = json.loads(source_prep.read_text(encoding="utf-8"))
    profile = yaml.safe_load((source_model / "flight_profile.yaml").read_text(encoding="utf-8"))
    profile["px4_sys_autostart"] = args.airframe_id
    profile["notes"] += " Conditional TS20-50 free-motion interface surrogate; 24V/loaded-arm dynamics and shaft torque unqualified."
    profile_text = yaml.safe_dump(profile, sort_keys=False, allow_unicode=True)
    (prep / "flight_profile.yaml").write_text(profile_text, encoding="utf-8")
    (destination / "flight_profile.yaml").write_text(profile_text, encoding="utf-8")
    (destination / "parent_flight_import_manifest.json").write_bytes((source_model / "import_manifest.json").read_bytes())
    report = deepcopy(original_report)
    report.update(model_name=args.model_name.removeprefix("amsim_urdf_").removesuffix("_flight"),
                  px4_sys_autostart=args.airframe_id, flight_profile_sha256=sha(prep / "flight_profile.yaml"),
                  parent_composition_preparation_sha256=sha(source_prep),
                  generator_sha256=sha(Path(__file__)), flight_validated=False,
                  native_ground_contact_validated=False,
                  conditional_actuator_plant={
                      "schema": evidence["schema"], "sdf_sha256": evidence["model_sha256"],
                      "plugin_library_sha256": evidence["plugin_library_sha256"],
                      "source_manifest_sha256": evidence["source_manifest_sha256"],
                      "rigid_geometry_reused_unchanged": True,
                      "prior_flight_gates_reused": False, "24v_loaded_qualified": False})
    report["controller_model"]["profile_name"] = report["model_name"]
    report["scope"] = ("Original rigid geometry/URDF/inertia and collision certificates preserved byte-identically. "
                       "New conditional TS20 interface plant; native ground, hover, probes, prediction and endpoint gates are new prerequisites.")
    (prep / "preparation_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    ArmModel.from_report(prep / "preparation_report.json")
    evidence.update(preparation_report=str(prep / "preparation_report.json"),
                    preparation_report_sha256=sha(prep / "preparation_report.json"),
                    runtime_log_directory_env="AMSIM_TS20_LOG_DIR", px4_sys_autostart=args.airframe_id,
                    geometry_inertia_collision_preserved=True, prior_flight_gates_reused=False)
    evidence["artifacts"] = {p.relative_to(destination).as_posix(): sha(p) for p in destination.rglob("*")
                             if p.is_file() and p.name not in ("plant_manifest.json", "import_manifest.json")}
    (destination / "plant_manifest.json").write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    imported = deepcopy(old_manifest)
    artifacts = {p.relative_to(destination).as_posix(): sha(p) for p in destination.rglob("*")
                 if p.is_file() and p.name != "import_manifest.json"}
    imported.update(model_name=args.model_name, model_sha256=artifacts["model.sdf"], sdf_sha256=artifacts["model.sdf"],
                    px4_sys_autostart=args.airframe_id, profile_sha256=artifacts["flight_profile.yaml"],
                    resources=artifacts, artifacts=artifacts, flight_validated=False,
                    native_validation=evidence["native_validation"],
                    flight_ready_meaning="Simulation binding complete for conditional TS20 interface; new native gates and actual 24V/loaded qualification remain pending.",
                    conditional_actuator_plant_manifest_sha256=artifacts["plant_manifest.json"],
                    parent_flight_manifest_sha256=sha(source_model / "import_manifest.json"),
                    prior_flight_gates_reused=False)
    (destination / "import_manifest.json").write_text(json.dumps(imported, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    spec = resolve(str(destination / "import_manifest.json"))
    airframe_dir.mkdir(parents=True)
    label = getattr(args, "airframe_label", "gz_x650_2link_ts20_v1")
    if not isinstance(label, str) or re.fullmatch(r"gz_[a-z][a-z0-9_]{0,60}", label) is None:
        raise ValueError("Airframe label must be a safe lowercase gz_ identifier")
    airframe_name = str(args.airframe_id) + "_" + label
    airframe = airframe_dir / airframe_name
    content = airframe_source.read_text(encoding="utf-8").replace(old_spec.model_name, args.model_name)
    content = content.replace("# AMSIM X650 original two-link commissioning;", "# AMSIM X650 conditional TS20 two-link commissioning;")
    content += "\n# TS20 change is in the Gazebo joint I/O plant only; normalized response is not physical shaft torque.\n"
    airframe.write_text(content, encoding="utf-8")
    installed_airframes = []
    installed_model = None
    if args.install:
        for folder in runtime_airframes:
            target = folder / airframe_name
            with target.open("xb") as stream:
                stream.write(airframe.read_bytes())
            target.chmod(0o755)
            installed_airframes.append(str(target))
        installed_model = str(install(spec))
    result = {"model": args.model_name, "model_manifest": str(destination / "import_manifest.json"),
              "model_sha256": spec.sha256, "preparation_report": str(prep / "preparation_report.json"),
              "preparation_report_sha256": sha(prep / "preparation_report.json"),
              "airframe": str(airframe), "airframe_sha256": sha(airframe),
              "px4_sys_autostart": args.airframe_id, "installed_airframes": installed_airframes,
              "installed_model": installed_model, "new_plant_flight_validated": False,
              "source_rigid_model_unchanged": sha(source_model / "model.sdf") == evidence["source_sdf_sha256"],
              "prior_flight_gates_reused": False}
    (airframe_dir / "installation_manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-sdf", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model-name", required=True)
    parser.add_argument("--motion-json", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--source-manifest", type=Path, required=True)
    parser.add_argument("--candidate-profile", type=Path, default=ROOT / "configs/actuators/ts20_50_24v_candidate.json")
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--library", type=Path)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "generated/ts20_interface_build")
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"))
    parser.add_argument("--complete-import", action="store_true")
    parser.add_argument("--source-preparation", type=Path)
    parser.add_argument("--preparation-dir", type=Path)
    parser.add_argument("--source-airframe", type=Path)
    parser.add_argument("--airframe-dir", type=Path)
    parser.add_argument("--airframe-id", type=int)
    parser.add_argument("--airframe-label", default="gz_x650_2link_ts20_v1")
    parser.add_argument("--px4-dir", type=Path)
    parser.add_argument("--install", action="store_true")
    args = parser.parse_args()
    if re.fullmatch(r"gz_[a-z][a-z0-9_]{0,60}", args.airframe_label) is None:
        parser.error("--airframe-label must be a safe lowercase gz_ identifier")
    if args.build == (args.library is not None):
        parser.error("Choose exactly one of --build and --library")
    if args.complete_import:
        for name in ("source_preparation", "preparation_dir", "source_airframe", "airframe_dir", "airframe_id", "px4_dir"):
            if getattr(args, name) is None:
                parser.error("--complete-import requires --" + name.replace("_", "-"))
        import sys
        sys.path.insert(0, str(ROOT / "src"))
    elif args.install:
        parser.error("--install requires --complete-import")
    evidence = derive(args)
    if args.complete_import:
        evidence = complete_import(args, evidence)
    print(json.dumps(evidence, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

