"""Explicit, auditable Gazebo/PX4 binding for a geometry-only URDF import.

This module never invents propulsion coefficients or an airframe number. A
successful binding means the simulation configuration is complete; it does not
mean the airframe, rotor mapping, or flight dynamics have been validated.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..paths import PROJECT_ROOT


@dataclass(frozen=True)
class FlightModel:
    name: str
    output_dir: Path
    sdf_path: Path
    sha256: str
    joints: list[str]
    links: list[str]
    manifest_path: Path
    px4_sys_autostart: int
    initial_joint_positions: dict[str, float]
    base_link: str = "base_link"


_MOTOR_FIELDS = {
    "motor_constant": ("motorConstant", True),
    "moment_constant": ("momentConstant", False),
    "max_rot_velocity": ("maxRotVelocity", True),
    "time_constant_up": ("timeConstantUp", True),
    "time_constant_down": ("timeConstantDown", True),
    "rotor_drag_coefficient": ("rotorDragCoefficient", False),
    "rolling_moment_coefficient": ("rollingMomentCoefficient", False),
    "rotor_velocity_slowdown_sim": ("rotorVelocitySlowdownSim", True),
}
_IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _number(value, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be an explicit finite number")
    value = float(value)
    if not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{label} must be finite and {'> 0' if positive else '>= 0'}")
    return value


def _vec(value, n: int, label: str) -> list[float]:
    if isinstance(value, str):
        value = value.split()
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError(f"{label} must contain {n} numbers")
    try:
        values = [float(x) for x in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain {n} finite numbers") from exc
    if not all(math.isfinite(x) for x in values):
        raise ValueError(f"{label} contains non-finite values")
    return values


def _multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _rotate(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def _rpy(values):
    r, p, y = values
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def _pose(element: ET.Element):
    pose = element.find("pose")
    if pose is None:
        return [0.] * 6, None
    if pose.get("rotation_format", "euler_rpy") != "euler_rpy":
        raise ValueError("flight binding supports only SDF Euler-rpy poses")
    values = _vec(pose.text or "0 0 0 0 0 0", 6, "SDF pose")
    if pose.get("degrees", "false").lower() in {"true", "1"}:
        values[3:] = [math.radians(x) for x in values[3:]]
    return values, pose.get("relative_to")


def _frame_rotations(model: ET.Element):
    """Resolve the SDF pose graph, including the child-frame joint default."""
    elements = {e.get("name"): e for tag in ("link", "joint", "frame") for e in model.findall(tag)}
    if len(elements) != sum(len(model.findall(tag)) for tag in ("link", "joint", "frame")):
        raise ValueError("duplicate or ambiguous SDF frame names")
    cache = {"__model__": _IDENTITY, model.get("name"): _IDENTITY}
    active = set()

    def resolve(name):
        own_prefix = str(model.get("name")) + "::"
        if name.startswith(own_prefix):
            name = name[len(own_prefix):]
        if name in cache:
            return cache[name]
        if name in active or name not in elements:
            raise ValueError(f"unresolved/cyclic SDF pose frame: {name}")
        element = elements[name]
        values, reference = _pose(element)
        if not reference:
            reference = (element.findtext("child") if element.tag == "joint" else
                         element.get("attached_to", "__model__") if element.tag == "frame" else "__model__")
        active.add(name)
        result = _multiply(resolve(reference), _rpy(values[3:]))
        active.remove(name)
        cache[name] = result
        return result

    return resolve


def _is_up(vector, label):
    norm = math.sqrt(sum(x*x for x in vector))
    if norm < 1e-12 or any(abs(vector[i]/norm - (1. if i == 2 else 0.)) > 1e-6 for i in range(3)):
        raise ValueError(f"{label} must point along body +Z; tilted/downward rotors are unsupported")


def _write_xml(tree: ET.ElementTree, path: Path):
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _plugin(model, filename, name, values):
    node = ET.SubElement(model, "plugin", filename=filename, name=name)
    for key, value in values.items():
        ET.SubElement(node, key).text = str(value)
    return node


def _source_manifest(directory: Path):
    path = directory / "import_manifest.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    schema = data.get("schema", {})
    kind = schema.get("kind") if isinstance(schema, dict) else None
    if (kind or data.get("kind")) != "urdf_import" or data.get("flight_ready") is not False:
        raise ValueError("source must be a geometry-only urdf_import with flight_ready=false")
    return path, data


def _validate_profile(profile, manifest, model):
    if not isinstance(profile, dict):
        raise ValueError("flight profile must be a YAML mapping")
    allowed = {"schema_version", "type", "base_link", "body_frame", "px4_sys_autostart", "controlled_joints", "rotors", "notes"}
    unknown = set(profile) - allowed
    if unknown:
        raise ValueError(f"unknown flight profile fields: {sorted(unknown)}")
    if type(profile.get("schema_version")) is not int or profile["schema_version"] != 1:
        raise ValueError("flight profile schema_version must be 1")
    if profile.get("type") != "px4_multicopter":
        raise ValueError("flight profile type must be px4_multicopter")
    if profile.get("body_frame") != "FLU":
        raise ValueError("body_frame: FLU must explicitly assert the root frame convention")
    if profile.get("base_link") != "base_link" or manifest.get("root_link") != "base_link":
        raise ValueError("PX4 binding currently requires the original root named base_link")
    autostart = profile.get("px4_sys_autostart")
    if type(autostart) is not int or autostart <= 0:
        raise ValueError("px4_sys_autostart must explicitly name a positive, configured PX4 airframe ID")
    if not isinstance(profile.get("notes"), str) or not profile["notes"].strip():
        raise ValueError("notes must document the provenance of motor mapping and propulsion parameters")
    if model.find(".//plugin") is not None or model.find(".//sensor") is not None:
        raise ValueError("source model already contains plugins/sensors; refusing duplicate execution")
    joints = {j.get("name"): j for j in model.findall("joint")}
    links = {link.get("name"): link for link in model.findall("link")}
    source_joints = {j["name"]: j for j in manifest.get("joints", [])}
    controlled = profile.get("controlled_joints")
    if not isinstance(controlled, list) or any(not isinstance(j, str) for j in controlled) or len(set(controlled)) != len(controlled):
        raise ValueError("controlled_joints must be an explicit list of unique joint names (or [])")
    for name in controlled:
        if name not in source_joints or name not in joints:
            raise ValueError(f"unsupported or missing controlled joint: {name}")
        if source_joints[name].get("type") not in {"revolute", "continuous"} or joints[name].get("type") != "revolute":
            raise ValueError(
                f"controlled joint {name} must be revolute/continuous; prismatic ArmAPI control is unsupported "
                "because the current position/velocity API uses angular units (geometry import remains supported)"
            )
        for bound, compare in (("lower", lambda x: x > 0), ("upper", lambda x: x < 0)):
            text = joints[name].findtext(f"axis/limit/{bound}")
            if text is not None and compare(float(text)):
                raise ValueError(f"controlled joint {name} cannot be initialized at zero")
    if "base_link" not in links:
        raise ValueError("base_link is absent from converted SDF")
    base_pose, base_relative = _pose(links["base_link"])
    if any(abs(x) > 1e-8 for x in base_pose) or base_relative not in {None, "", "__model__", model.get("name")}:
        raise ValueError("base_link must coincide with the SDF model frame for PX4/odometry")
    rotors = profile.get("rotors")
    if not isinstance(rotors, list) or len(rotors) < 3 or any(not isinstance(r, dict) for r in rotors):
        raise ValueError("rotors must explicitly describe at least three motors")
    numbers = [r.get("motor_number") for r in rotors]
    if any(type(n) is not int for n in numbers) or sorted(numbers) != list(range(len(rotors))):
        raise ValueError("motor_number values must be unique and contiguous from 0")
    if len({r.get("joint") for r in rotors}) != len(rotors) or len({r.get("link") for r in rotors}) != len(rotors):
        raise ValueError("each rotor must have a distinct joint and link")
    resolve_rotation = _frame_rotations(model)
    conversions = []
    velocity_checks = []
    for rotor in rotors:
        allowed_rotor = set(_MOTOR_FIELDS) | {"motor_number", "joint", "link", "turning_direction", "allow_continuous_conversion", "notes"}
        if set(rotor) - allowed_rotor:
            raise ValueError(f"unknown rotor fields: {sorted(set(rotor) - allowed_rotor)}")
        name, link_name = rotor.get("joint"), rotor.get("link")
        if name in controlled:
            raise ValueError(f"rotor {name} cannot also be a controlled joint")
        source, joint = source_joints.get(name), joints.get(name)
        if source is None or joint is None or link_name not in links:
            raise ValueError(f"rotor references missing joint/link: {name}/{link_name}")
        if source.get("parent") != "base_link" or source.get("child") != link_name or source.get("type") not in {"revolute", "continuous"}:
            raise ValueError(f"rotor {name} must be a root-attached revolute/continuous joint")
        if "axis_in_root_at_zero" in source:
            _is_up(_vec(source["axis_in_root_at_zero"], 3, f"rotor {name} source axis"), f"rotor {name} source joint axis")
        if joint.get("type") != "revolute" or joint.findtext("parent") != "base_link" or joint.findtext("child") != link_name:
            raise ValueError(f"rotor {name} SDF joint disagrees with source topology")
        if rotor.get("turning_direction") not in {"cw", "ccw"}:
            raise ValueError(f"rotor {name} turning_direction must be cw or ccw")
        if "allow_continuous_conversion" in rotor and type(rotor["allow_continuous_conversion"]) is not bool:
            raise ValueError("allow_continuous_conversion must be boolean")
        for key, (_, positive) in _MOTOR_FIELDS.items():
            _number(rotor.get(key), f"rotor {name}.{key}", positive=positive)
        # Gazebo's motor system spins the simulated joint at physical rotor
        # speed / slowdown. Widening angle limits must not conceal a CAD joint
        # speed limit that prevents the configured maximum thrust.
        simulated_max = float(rotor["max_rot_velocity"]) / float(rotor["rotor_velocity_slowdown_sim"])
        if not math.isfinite(simulated_max):
            raise ValueError(f"rotor {name} maximum simulated joint speed is not finite")
        source_velocity = source.get("limits", {}).get("velocity")
        sdf_velocity = joint.findtext("axis/limit/velocity")
        for label, value in (("source", source_velocity), ("SDF", sdf_velocity)):
            if value is None:
                continue
            try:
                limit_velocity = float(value)
            except (ValueError, TypeError) as exc:
                raise ValueError(f"rotor {name} {label} velocity limit must be finite") from exc
            if not math.isfinite(limit_velocity) or limit_velocity < -1 or -1 < limit_velocity < 0:
                raise ValueError(f"rotor {name} {label} velocity limit must be nonnegative or -1 (unlimited)")
            if limit_velocity >= 0 and simulated_max > limit_velocity + 1e-9 * max(1., simulated_max):
                raise ValueError(
                    f"rotor {name} {label} velocity limit {limit_velocity:g} rad/s is below max_rot_velocity / "
                    f"rotor_velocity_slowdown_sim = {simulated_max:g} rad/s; correct the explicitly documented "
                    "source limit or motor profile, then re-import (continuous conversion only changes angle limits)"
                )
        velocity_checks.append({"joint": name, "source_velocity_limit_rad_s": source_velocity,
                                "sdf_velocity_limit_rad_s": None if sdf_velocity is None else float(sdf_velocity),
                                "maximum_simulated_joint_speed_rad_s": simulated_max})
        xyz = joint.find("axis/xyz")
        if xyz is None:
            raise ValueError(f"rotor {name} has no explicit SDF joint axis")
        parent_frame = joint.findtext("axis/use_parent_model_frame", "false").lower() in {"true", "1"}
        axis_frame = xyz.get("expressed_in") or ("__model__" if parent_frame else name)
        _is_up(_rotate(resolve_rotation(axis_frame), _vec(xyz.text, 3, f"rotor {name} axis")), f"rotor {name} joint axis")
        _is_up(_rotate(resolve_rotation(link_name), [0., 0., 1.]), f"rotor {name} thrust axis (link +Z)")
        bounds = [joint.findtext(f"axis/limit/{key}") for key in ("lower", "upper")]
        limited = source.get("type") == "revolute" or any(x is not None and abs(float(x)) < 1e15 for x in bounds)
        if limited:
            if rotor.get("allow_continuous_conversion") is not True:
                raise ValueError(f"rotor {name} has finite source angle limits; explicit allow_continuous_conversion: true is required")
            conversions.append({"joint": name, "original_source_limits": source.get("limits"), "original_sdf_limits": bounds, "new_limits": [-1e16, 1e16]})
    return joints, links, conversions, velocity_checks


def bind_flight_profile(import_directory: Path, profile_path: Path, output_root: Path | None = None) -> FlightModel:
    """Create an independent ``*_flight`` model, requiring native SDF validation.

    ``output_root`` is a parent directory. Existing destinations are never
    replaced, and the geometry-only import is never changed. Callers must check
    that ``px4_sys_autostart`` exists in their configured PX4 installation.
    """
    directory, profile_path = Path(import_directory).resolve(), Path(profile_path).resolve()
    manifest_path, source = _source_manifest(directory)
    source_sdf = directory / "model.sdf"
    artifacts = source.get("artifacts", {})
    if not isinstance(artifacts, dict):
        raise ValueError("source manifest artifacts must be a path-to-SHA256 mapping")
    expected_sha = artifacts.get("model.sdf") or source.get("model_sha256") or source.get("sdf_sha256") or source.get("sha256")
    if not expected_sha or _sha(source_sdf) != expected_sha:
        raise ValueError("source model.sdf hash differs from import manifest")
    for relative, digest in artifacts.items():
        if not isinstance(relative, str) or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("invalid source artifact path/SHA256 entry")
        artifact = (directory / relative).resolve()
        if directory not in artifact.parents or not artifact.is_file() or _sha(artifact) != digest:
            raise ValueError(f"source artifact missing, outside import, or hash differs: {relative}")
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    tree = ET.parse(source_sdf)
    model = tree.getroot().find("model")
    if model is None or len(tree.getroot().findall("model")) != 1 or model.findall("model"):
        raise ValueError("flight binding requires one non-nested SDF model")
    joints, links, conversions, velocity_checks = _validate_profile(profile, source, model)
    original_name = model.get("name", "")
    if not re.fullmatch(r"amsim_urdf_[A-Za-z0-9_]+", original_name) or original_name.endswith("_flight"):
        raise ValueError("source model name must use the amsim_urdf_ namespace")
    name = original_name + "_flight"
    output_root = Path(output_root).resolve() if output_root else directory.parent
    if output_root == directory or directory in output_root.parents:
        raise ValueError("output_root must be outside the source import directory")
    target = output_root / name
    if target.exists():
        raise FileExistsError(f"flight model already exists: {target}")
    # Resolve and check sensor configuration before creating any output.
    sensor_path = PROJECT_ROOT / "assets" / "models" / "x500_underfold" / "model.sdf"
    sensor_model = ET.parse(sensor_path).getroot().find("model")
    sensor_base = sensor_model.find("link[@name='base_link']") if sensor_model is not None else None
    sensors = list(sensor_base.findall("sensor")) if sensor_base is not None else []
    if {s.get("type") for s in sensors} != {"air_pressure", "magnetometer", "imu", "navsat"} or len(sensors) != 4:
        raise ValueError("reference simulation sensor configuration is incomplete or ambiguous")
    for sensor in sensors:
        links["base_link"].append(copy.deepcopy(sensor))
    for conversion in conversions:
        axis = joints[conversion["joint"]].find("axis")
        limit = axis.find("limit")
        if limit is None:
            limit = ET.SubElement(axis, "limit")
        for key, value in (("lower", "-1e16"), ("upper", "1e16")):
            node = limit.find(key)
            if node is None:
                node = ET.SubElement(limit, key)
            node.text = value
    for rotor in sorted(profile["rotors"], key=lambda r: r["motor_number"]):
        values = {"jointName": rotor["joint"], "linkName": rotor["link"], "turningDirection": rotor["turning_direction"]}
        values.update({tag: format(float(rotor[key]), ".17g") for key, (tag, _) in _MOTOR_FIELDS.items()})
        values.update(commandSubTopic="command/motor_speed", motorNumber=rotor["motor_number"], motorType="velocity")
        _plugin(model, "gz-sim-multicopter-motor-model-system", "gz::sim::systems::MulticopterMotorModel", values)
    for joint in profile["controlled_joints"]:
        _plugin(model, "gz-sim-joint-controller-system", "gz::sim::systems::JointController", {"joint_name": joint, "initial_velocity": 0})
    if profile["controlled_joints"]:
        _plugin(model, "gz-sim-joint-state-publisher-system", "gz::sim::systems::JointStatePublisher", {"update_rate": 50})
    _plugin(model, "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher", {
        "dimensions": 3, "odom_publish_frequency": 50, "odom_topic": "/ground_truth/odometry",
        "odom_frame": "world", "robot_base_frame": f"{name}/model_frame", "gaussian_noise": 0,
        "odom_covariance_topic": "/ground_truth/odometry_with_covariance", "tf_topic": "/ground_truth/pose",
    })
    model.set("name", name)
    for node in model.iter():
        for key in ("relative_to", "expressed_in", "attached_to"):
            value = node.get(key)
            if value == original_name:
                node.set(key, name)
            elif value and value.startswith(original_name + "::"):
                node.set(key, name + value[len(original_name):])
        if node.tag == "uri" and node.text and node.text.startswith(f"model://{original_name}/"):
            node.text = f"model://{name}/" + node.text[len(f"model://{original_name}/"):]
    output_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".amsim-flight-", dir=output_root) as temp:
        stage = Path(temp) / name
        for resource in directory.rglob("*"):
            if resource.is_symlink():
                raise ValueError(f"source import contains a symlink: {resource.relative_to(directory)}")
        shutil.copytree(directory, stage)
        shutil.copy2(profile_path, stage / "flight_profile.yaml")
        shutil.copy2(manifest_path, stage / "source_import_manifest.json")
        _write_xml(tree, stage / "model.sdf")
        config = ET.Element("model")
        ET.SubElement(config, "name").text = name
        ET.SubElement(config, "version").text = "1.0"
        ET.SubElement(config, "sdf", version=tree.getroot().get("version", "1.9")).text = "model.sdf"
        ET.SubElement(config, "description").text = "URDF-derived PX4 configuration; flight dynamics not validated."
        _write_xml(ET.ElementTree(config), stage / "model.config")
        # Resolve model://<new-flight-name>/ resources while the independent
        # model is still staged, just as the URDF importer does during -p/-k.
        validation_env = os.environ.copy()
        for key in ("GZ_SIM_RESOURCE_PATH", "SDF_PATH"):
            previous = validation_env.get(key, "")
            validation_env[key] = str(stage.parent) + (os.pathsep + previous if previous else "")
        try:
            checked = subprocess.run(["gz", "sdf", "-k", str(stage / "model.sdf")], capture_output=True, text=True, env=validation_env, timeout=60, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError("native Gazebo SDF validation is required (gz sdf -k)") from exc
        detail = (checked.stdout + checked.stderr).strip()
        if checked.returncode != 0 or "Valid." not in detail:
            raise RuntimeError("flight SDF validation failed:\n" + detail[-5000:])
        model_sha = _sha(stage / "model.sdf")
        resources = {p.relative_to(stage).as_posix(): _sha(p) for p in sorted(stage.rglob("*")) if p.is_file() and p.name != "import_manifest.json"}
        result = {
            "schema": {"kind": "urdf_flight", "version": 1}, "schema_version": 1, "kind": "urdf_flight",
            "model_name": name, "root_link": "base_link", "base_link": "base_link",
            "body_frame": "FLU", "flight_ready": True, "flight_validated": False,
            "flight_ready_meaning": "Explicit simulation binding complete; propulsion calibration, PX4 allocation and flight remain unvalidated.",
            "source_manifest_sha256": _sha(manifest_path), "profile_sha256": _sha(profile_path),
            "model_sha256": model_sha, "sdf_sha256": model_sha,
            "px4_sys_autostart": profile["px4_sys_autostart"], "joints": list(profile["controlled_joints"]),
            "controlled_joints": list(profile["controlled_joints"]), "links": sorted(links),
            "initial_joint_positions": {j: 0. for j in profile["controlled_joints"]},
            "continuous_conversions": conversions, "rotor_velocity_checks": velocity_checks,
            "rotors": profile["rotors"], "notes": profile["notes"],
            "sensor_configuration": {"source": "assets/models/x500_underfold/model.sdf", "source_sha256": _sha(sensor_path),
                                     "copied": "Only four base_link sensor configurations; no mass, inertia, propulsion values or PID parameters copied."},
            "native_validation": {"command": ["gz", "sdf", "-k", "model.sdf"], "passed": True, "output": detail},
            "resources": resources, "artifacts": resources,
        }
        (stage / "import_manifest.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        stage.rename(target)
    return FlightModel(name, target, target / "model.sdf", model_sha, list(profile["controlled_joints"]), sorted(links),
                       target / "import_manifest.json", profile["px4_sys_autostart"], result["initial_joint_positions"])
