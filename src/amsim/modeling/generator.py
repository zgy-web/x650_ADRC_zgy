from __future__ import annotations

import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..paths import PROJECT_ROOT

@dataclass
class BuiltModel:
    name: str
    config_name: str
    output_dir: Path
    sdf_path: Path
    sha256: str
    joints: list[str]
    links: list[str]
    total_arm_mass_kg: float

def _fmt(x: float) -> str:
    return f"{x:.12g}"

def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def _vec(value, n: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != n:
        raise ValueError(f"{label} must be a list of length {n}")
    out = [float(x) for x in value]
    if not all(math.isfinite(x) for x in out):
        raise ValueError(f"{label} contains non-finite values")
    return out

def _positive(x, label: str) -> float:
    v = float(x)
    if not math.isfinite(v) or v <= 0:
        raise ValueError(f"{label} must be finite and > 0")
    return v

def _box_mass_and_inertia(link: dict) -> tuple[float, tuple[float, float, float]]:
    g = link["geometry"]
    if g.get("type") != "box":
        raise ValueError("V1 configurable model generator supports box links only")
    L = _positive(g["length_m"], "length_m")
    W = _positive(g["width_m"], "width_m")
    H = _positive(g["height_m"], "height_m")

    mcfg = link["mass"]
    has_mass = "mass_kg" in mcfg
    has_density = "density_kg_m3" in mcfg
    if has_mass == has_density:
        raise ValueError(
            f"{link['name']}: specify exactly one of mass_kg or density_kg_m3"
        )
    if has_mass:
        mass = _positive(mcfg["mass_kg"], "mass_kg")
    else:
        density = _positive(mcfg["density_kg_m3"], "density_kg_m3")
        mass = density * L * W * H

    ixx = mass * (W * W + H * H) / 12.0
    iyy = mass * (L * L + H * H) / 12.0
    izz = mass * (L * L + W * W) / 12.0
    return mass, (ixx, iyy, izz)

def _validate(config: dict) -> None:
    if int(config.get("schema_version", 0)) != 1:
        raise ValueError("schema_version must be 1")
    if config.get("mode") != "generated_serial_arm":
        raise ValueError("mode must be generated_serial_arm")
    if not config.get("model_name"):
        raise ValueError("model_name is required")

    arm = config["arm"]
    _vec(arm["mount_xyz_m"], 3, "mount_xyz_m")
    _vec(arm.get("joint_axis_xyz", [0, 1, 0]), 3, "joint_axis_xyz")

    links = arm.get("links")
    if not isinstance(links, list) or not links:
        raise ValueError("arm.links must contain at least one link")

    names = set()
    joints = set()
    for i, link in enumerate(links, start=1):
        name = str(link.get("name", ""))
        if not name or name in names:
            raise ValueError(f"invalid or duplicate link name at index {i}: {name}")
        names.add(name)

        _box_mass_and_inertia(link)
        rgba = _vec(link.get("material_rgba", [0.5, 0.5, 0.5, 1]), 4, "material_rgba")
        if any(x < 0 or x > 1 for x in rgba):
            raise ValueError(f"{name}: material_rgba values must be in [0,1]")

        joint = link["joint"]
        jname = str(joint.get("name", ""))
        if not jname or jname in joints:
            raise ValueError(f"invalid or duplicate joint name: {jname}")
        joints.add(jname)

        initial = float(joint["initial_angle_deg"])
        lo, hi = _vec(joint["physical_limits_deg"], 2, "physical_limits_deg")
        if not lo < hi:
            raise ValueError(f"{jname}: physical lower limit must be < upper")
        if not lo <= initial <= hi:
            raise ValueError(
                f"{jname}: initial_angle_deg={initial} outside [{lo}, {hi}]"
            )
        _positive(joint["effort_limit"], "effort_limit")
        _positive(joint["velocity_limit_rad_s"], "velocity_limit_rad_s")
        if float(joint.get("damping", 0)) < 0 or float(joint.get("friction", 0)) < 0:
            raise ValueError(f"{jname}: damping/friction must be >= 0")
        if joint.get("controller", "velocity") != "velocity":
            raise ValueError("V1 generator supports velocity joint controller only")

def _remove_old_arm(model: ET.Element) -> None:
    for child in list(model):
        if child.tag == "link" and child.get("name", "").startswith("arm_link_"):
            model.remove(child)
        elif child.tag == "joint" and child.get("name", "").startswith("arm_joint_"):
            model.remove(child)
        elif child.tag == "plugin":
            filename = child.get("filename", "")
            jn = child.findtext("joint_name", "")
            if (
                "joint-controller" in filename and jn.startswith("arm_joint_")
            ) or "joint-state-publisher" in filename:
                model.remove(child)

def _add_text(parent: ET.Element, tag: str, text: str) -> ET.Element:
    e = ET.SubElement(parent, tag)
    e.text = text
    return e

def build(config_path: Path, output_root: Path | None = None) -> BuiltModel:
    config_path = config_path.resolve()
    config = yaml.safe_load(config_path.read_text())
    _validate(config)

    template = Path(config["template_model"])
    if not template.is_absolute():
        template = PROJECT_ROOT / template
    template = template.resolve()
    if not template.is_file():
        raise FileNotFoundError(f"template_model not found: {template}")

    tree = ET.parse(template)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError("template has no <model>")

    _remove_old_arm(model)
    model_name = str(config["model_name"])
    model.set("name", model_name)

    self_collide = model.find("self_collide")
    if self_collide is not None:
        self_collide.text = str(bool(config["arm"].get("self_collide", False))).lower()

    arm = config["arm"]
    mount = _vec(arm["mount_xyz_m"], 3, "mount_xyz_m")
    axis = _vec(arm.get("joint_axis_xyz", [0, 1, 0]), 3, "joint_axis_xyz")
    links = arm["links"]

    # Link frames sit at proximal joints. Initial joint angles are encoded into
    # the initial link poses. Gazebo joint coordinates are therefore zero at
    # the configured initial physical pose.
    origin = mount[:]
    cumulative = 0.0
    parent = "base_link"
    total_mass = 0.0
    joint_names: list[str] = []
    link_names: list[str] = []

    for item in links:
        lname = str(item["name"])
        joint = item["joint"]
        jname = str(joint["name"])
        initial_deg = float(joint["initial_angle_deg"])
        cumulative += math.radians(initial_deg)

        g = item["geometry"]
        L = float(g["length_m"])
        W = float(g["width_m"])
        H = float(g["height_m"])
        mass, (ixx, iyy, izz) = _box_mass_and_inertia(item)
        total_mass += mass

        link = ET.SubElement(model, "link", {"name": lname})
        _add_text(
            link,
            "pose",
            f"{_fmt(origin[0])} {_fmt(origin[1])} {_fmt(origin[2])} 0 {_fmt(cumulative)} 0",
        )

        inertial = ET.SubElement(link, "inertial")
        _add_text(inertial, "mass", _fmt(mass))
        inertia = ET.SubElement(inertial, "inertia")
        _add_text(inertia, "ixx", _fmt(ixx))
        _add_text(inertia, "iyy", _fmt(iyy))
        _add_text(inertia, "izz", _fmt(izz))
        _add_text(inertia, "ixy", "0")
        _add_text(inertia, "ixz", "0")
        _add_text(inertia, "iyz", "0")
        _add_text(inertial, "pose", f"{_fmt(L/2)} 0 0 0 0 0")

        for tag in ("visual", "collision"):
            element = ET.SubElement(link, tag, {"name": f"{lname}_{tag}"})
            _add_text(element, "pose", f"{_fmt(L/2)} 0 0 0 0 0")
            geom = ET.SubElement(element, "geometry")
            box = ET.SubElement(geom, "box")
            _add_text(box, "size", f"{_fmt(L)} {_fmt(W)} {_fmt(H)}")
            if tag == "visual":
                material = ET.SubElement(element, "material")
                rgba = _vec(item.get("material_rgba", [0.5,0.5,0.5,1]), 4, "material_rgba")
                color = " ".join(_fmt(x) for x in rgba)
                _add_text(material, "ambient", color)
                _add_text(material, "diffuse", color)

        joint_el = ET.SubElement(model, "joint", {"name": jname, "type": "revolute"})
        _add_text(joint_el, "parent", parent)
        _add_text(joint_el, "child", lname)
        _add_text(joint_el, "pose", "0 0 0 0 0 0").set("relative_to", lname)
        axis_el = ET.SubElement(joint_el, "axis")
        xyz = _add_text(axis_el, "xyz", " ".join(_fmt(x) for x in axis))
        xyz.set("expressed_in", "base_link")

        initial = float(joint["initial_angle_deg"])
        phys_lo, phys_hi = [float(x) for x in joint["physical_limits_deg"]]
        limit = ET.SubElement(axis_el, "limit")
        _add_text(limit, "lower", _fmt(math.radians(phys_lo - initial)))
        _add_text(limit, "upper", _fmt(math.radians(phys_hi - initial)))
        _add_text(limit, "effort", _fmt(float(joint["effort_limit"])))
        _add_text(limit, "velocity", _fmt(float(joint["velocity_limit_rad_s"])))

        dynamics = ET.SubElement(axis_el, "dynamics")
        _add_text(dynamics, "damping", _fmt(float(joint.get("damping", 0))))
        _add_text(dynamics, "friction", _fmt(float(joint.get("friction", 0))))

        plugin = ET.SubElement(
            model,
            "plugin",
            {
                "filename": "gz-sim-joint-controller-system",
                "name": "gz::sim::systems::JointController",
            },
        )
        _add_text(plugin, "joint_name", jname)
        _add_text(plugin, "initial_velocity", "0")

        joint_names.append(jname)
        link_names.append(lname)

        origin = [
            origin[0] + L * math.cos(cumulative),
            origin[1],
            origin[2] - L * math.sin(cumulative),
        ]
        parent = lname

    state_plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-joint-state-publisher-system",
            "name": "gz::sim::systems::JointStatePublisher",
        },
    )
    _add_text(state_plugin, "update_rate", _fmt(float(arm.get("joint_state_rate_hz", 50))))

    if output_root is None:
        output_root = PROJECT_ROOT / "generated" / "models"
    output_dir = output_root / model_name
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    ET.indent(tree, space="  ")
    sdf_path = output_dir / "model.sdf"
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)

    (output_dir / "model.config").write_text(
        "<model>"
        f"<name>{model_name}</name>"
        "<version>1.0</version>"
        '<sdf version="1.9">model.sdf</sdf>'
        "<description>Generated by aerial_manipulator_sim</description>"
        "</model>\n"
    )

    metadata = {
        "schema_version": 1,
        "config_name": config["name"],
        "model_name": model_name,
        "source_config": str(config_path),
        "template_model": str(template),
        "link_count": len(links),
        "links": link_names,
        "joints": joint_names,
        "total_arm_mass_kg": total_mass,
        "model_sha256": _sha256(sdf_path),
    }
    (output_dir / "model_manifest.json").write_text(json.dumps(metadata, indent=2))

    return BuiltModel(
        name=model_name,
        config_name=str(config["name"]),
        output_dir=output_dir,
        sdf_path=sdf_path,
        sha256=metadata["model_sha256"],
        joints=joint_names,
        links=link_names,
        total_arm_mass_kg=total_mass,
    )

def list_configs() -> list[dict]:
    root = PROJECT_ROOT / "configs" / "models"
    rows = []
    for path in sorted(root.glob("*.yaml")):
        cfg = yaml.safe_load(path.read_text()) or {}
        rows.append({
            "name": cfg.get("name", path.stem),
            "model_name": cfg.get("model_name"),
            "mode": cfg.get("mode"),
            "links": len((cfg.get("arm") or {}).get("links") or []),
            "config": str(path),
        })
    return rows
